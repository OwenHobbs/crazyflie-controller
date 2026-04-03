from __future__ import annotations

import threading
import time
from typing import Optional

from cflib.utils import uri_helper

from control import ControlCommand, Goal, PIDPositionController
from crazyflie_client import CrazyflieClient
from crazyflie_telemetry import CrazyflieTelemetry
from flight_logger import FlightLogger
from vicon_motion import ViconMotionClient


class FlightService:
    """Owns the background mocap/control/send loop.

    Main (or any other thread) only needs to call set_goal().
    """

    def __init__(
        self,
        crazyflie_uri: str,
        drone_object_name: str,
        mocap_client: ViconMotionClient,
        log_output_dir: str,
        yaw_rate_command: float = 0.0,
    ):
        self._cf_client = CrazyflieClient(uri_helper.uri_from_env(default=crazyflie_uri))
        self._mocap_client = mocap_client
        self._controller = PIDPositionController()
        self._logger = FlightLogger(base_output_dir=log_output_dir)
        self._drone_object_name = drone_object_name
        self._telemetry_client = CrazyflieTelemetry(self._cf_client.cf)
        self._yaw_rate_command = yaw_rate_command

        self._goal_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._goal: Optional[Goal] = None
        self._latest_frame: dict | None = None
        self._latest_runtime: float = 0.0
        self._last_command: Optional[ControlCommand] = None
        self._last_seen_frame_id: int = 0

        self._start_time: float | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False

    def start(self) -> None:
        print("Starting flight service...")
        if self._started:
            return

        self._controller.reset()
        self._last_seen_frame_id: int = 0

        self._cf_client.open_link()
        if not self._cf_client.wait_until_connected(timeout=10.0):
            raise RuntimeError('Timed out waiting for Crazyflie connection')

        if self._telemetry_client is not None:
            self._telemetry_client.start()

        self._cf_client.unlock_thrust_protection()

        self._stop_event.clear()
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._run_loop, name=self._drone_object_name, daemon=True)
        self._thread.start()
        self._started = True

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

    def set_goal(self, goal: Goal) -> None:
        with self._goal_lock:
            self._goal = goal

    def clear_goal(self) -> None:
        with self._goal_lock:
            self._goal = None

    def get_goal(self) -> Optional[Goal]:
        with self._goal_lock:
            return self._goal

    def get_latest_frame(self) -> dict:
        with self._state_lock:
            if self._latest_frame is None:
                return {}
            return dict(self._latest_frame)

    def get_latest_pose(self, rigid_body_name: str):
        with self._state_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.get(rigid_body_name)

    def get_runtime(self) -> float:
        with self._state_lock:
            return self._latest_runtime

    def get_last_command(self) -> Optional[ControlCommand]:
        with self._state_lock:
            return self._last_command

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            result = self._mocap_client.wait_for_new_frame(
                last_frame_id=self._last_seen_frame_id,
                timeout=0.5,
            )
            if result is None:
                continue

            frame_id, frame = result
            self._last_seen_frame_id = frame_id

            runtime = time.time() - self._start_time if self._start_time is not None else 0.0
            drone_pose = frame.get(self._drone_object_name)

            with self._state_lock:
                self._latest_frame = frame
                self._latest_runtime = runtime

            if drone_pose is None:
                continue

            with self._goal_lock:
                goal = self._goal

            if goal is None:
                self._cf_client.send_setpoint(0.0, 0.0, self._yaw_rate_command, 0)
                continue

            self._controller.add_sample(
                x=drone_pose.x,
                y=drone_pose.y,
                z=drone_pose.z,
                timestamp=drone_pose.timestamp,
            )

            command = self._controller.compute_command(
                current_yaw=drone_pose.yaw,
                goal_x=goal.x,
                goal_y=goal.y,
                goal_z=goal.z,
            )

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
                self._yaw_rate_command,
                command.thrust,
            )

            with self._state_lock:
                self._last_command = command