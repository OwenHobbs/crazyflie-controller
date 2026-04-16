from __future__ import annotations

import threading
import time
from typing import Optional

from cflib.utils import uri_helper

from crazyflie.crazyflie_client import CrazyflieClient
from crazyflie.crazyflie_telemetry import CrazyflieTelemetry
from mocap.mocap_client import MocapClient
from .flight_action import FlightAction, FlightActionIdle

from .flight_control import ControlCommand, PIDGains, PIDPositionController
from .flight_logger import FlightLogger

"""
This module provides the FlightService class that manages the background control loop
for the Crazyflie drone. It integrates with the flight, crazyflie, and mocap
packages to coordinate tracking data, PID control, and command execution.
"""

# Service class for managing the drone control loop
class FlightService:
    """Owns the background mocap/control/send loop.

    Main (or any other thread) only needs to call set_action().
    """

    # Initialize the flight service with drone and mocap configuration
    def __init__(
        self,
        crazyflie_uri: str,
        drone_object_name: str,
        mocap_client: MocapClient,
        log_output_dir: str | None = None,
        gains: PIDGains | None = None
    ):
        self._cf_client = CrazyflieClient(uri_helper.uri_from_env(default=crazyflie_uri))
        self._mocap_client = mocap_client
        self._controller = PIDPositionController(gains=gains)
        if log_output_dir is None:
            log_output_dir = f'flight_logs/{drone_object_name}'
        self._logger = FlightLogger(base_output_dir=log_output_dir)
        self.drone_object_name = drone_object_name
        self._telemetry_client = CrazyflieTelemetry(self._cf_client.cf)

        self._action_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._action: FlightAction = FlightActionIdle()
        self._latest_frame: dict | None = None
        self._latest_runtime: float = 0.0
        self._last_command: Optional[ControlCommand] = None
        self._last_seen_frame_id: int = 0

        self._start_time: float | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False

    # Start the flight service and background control loop
    def start(self) -> None:
        print("Starting flight service...")
        if self._started:
            return

        self._controller.reset()
        self._last_seen_frame_id = 0

        self._cf_client.open_link()
        if not self._cf_client.wait_until_connected(timeout=10.0):
            raise RuntimeError('Timed out waiting for Crazyflie connection')

        if self._telemetry_client is not None:
            self._telemetry_client.start()

        self._cf_client.unlock_thrust_protection()

        self._stop_event.clear()
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._run_loop, name=f'{self.drone_object_name}_Service', daemon=True)
        self._thread.start()
        self._started = True

    # Stop the flight service and close connections
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        try:
            self._cf_client.stop()
        finally:
            self._logger.save_all()
            if self._telemetry_client is not None:
                self._telemetry_client.stop()
            self._cf_client.close()
            self._started = False

    # Set a new control action for the drone
    def set_action(self, action: FlightAction) -> None:
        with self._action_lock:
            self._action = action

    # Get the current control action
    def get_action(self) -> FlightAction:
        with self._action_lock:
            return self._action

    # Get the latest motion capture frame
    def get_latest_frame(self) -> dict:
        with self._state_lock:
            if self._latest_frame is None:
                return {}
            return dict(self._latest_frame)

    # Get the latest pose of a specific rigid body
    def get_latest_pose(self, rigid_body_name: str):
        with self._state_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.get(rigid_body_name)

    # Get the elapsed runtime since the service started
    def get_runtime(self) -> float:
        with self._state_lock:
            return self._latest_runtime

    # Get the last control command sent to the drone
    def get_last_command(self) -> Optional[ControlCommand]:
        with self._state_lock:
            return self._last_command

    # Background loop that processes mocap data and sends control commands
    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            result = self._mocap_client.wait_for_new_frame(
                last_frame_id=self._last_seen_frame_id,
                timeout=0.5,
            )
            if result is None:
                continue

            frame_id, frame = result
            # thread.lock not needed for self._last_seen_frame_id
            # since it is not accessed by external threads
            self._last_seen_frame_id = frame_id
            runtime = time.time() - self._start_time if self._start_time is not None else 0.0

            with self._state_lock:
                self._latest_frame = frame
                self._latest_runtime = runtime

            drone_pose = frame.get(self.drone_object_name)
            if drone_pose is None:
                continue

            self._controller.add_sample(
                x=drone_pose.x,
                y=drone_pose.y,
                z=drone_pose.z,
                yaw=drone_pose.yaw,
                timestamp=drone_pose.timestamp,
            )

            # Determine goal based on flight action
            with self._action_lock:
                goal, new_action = self._action.execute(
                    drone_pose=drone_pose,
                    frame=frame,
                    flight_runtime=runtime,
                )
                # Update the next action if applicable
                # TODO: new_action should be removed in favor of a queue of actions
                if new_action is not None:
                    self._action = new_action
                    # TODO: could notify listeners that action is finished
                    # skip the current action and avoid killing drone
                    continue

            if goal is None:
                self._cf_client.send_setpoint(0.0, 0.0, 0.0, 0)
                continue

            command = self._controller.compute_command(goal)

            telemetry = self._telemetry_client.get_telemetry() if self._telemetry_client is not None else None

            self._logger.log_sample(
                runtime=runtime,
                drone_pose=drone_pose,
                goal=goal,
                command=command,
                telemetry=telemetry,
            )

            self._cf_client.send_setpoint(
                command.roll,
                command.pitch,
                command.yaw_rate,
                command.thrust,
            )

            with self._state_lock:
                self._last_command = command
