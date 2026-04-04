from __future__ import annotations

import math
import threading
import time
from typing import Optional

from cflib.utils import uri_helper

from flight_control import ControlCommand, Goal, PIDPositionController
from crazyflie_client import CrazyflieClient
from crazyflie_telemetry import CrazyflieTelemetry
from flight_logger import FlightLogger
from vicon_motion import ViconMotionClient


class FlightService:
    """Background mocap/control/send loop with internal rate-limited reference."""

    def __init__(
        self,
        crazyflie_uri: str,
        drone_object_name: str,
        mocap_client: ViconMotionClient,
        log_output_dir: str,
    ):
        self._cf_client = CrazyflieClient(uri_helper.uri_from_env(default=crazyflie_uri))
        self._mocap_client = mocap_client
        self._controller = PIDPositionController()
        self._logger = FlightLogger(base_output_dir=log_output_dir)
        self._drone_object_name = drone_object_name
        self._telemetry_client = CrazyflieTelemetry(self._cf_client.cf)

        self._goal_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._goal: Optional[Goal] = None           # raw user goal
        self._commanded_goal: Optional[Goal] = None # smoothed internal reference

        self._latest_frame: dict | None = None
        self._latest_runtime: float = 0.0
        self._last_command: Optional[ControlCommand] = None
        self._last_seen_frame_id: int = 0
        self._last_loop_timestamp: float | None = None

        self._start_time: float | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False

        # Reference slew limits.
        self._xy_rate_limit_mps = 0.45
        self._z_up_rate_limit_mps = 0.35
        self._z_down_rate_limit_mps = 0.45
        self._heading_rate_limit_dps = 40.0

    def start(self) -> None:
        print('Starting flight service...')
        if self._started:
            return

        self._controller.reset()
        self._last_seen_frame_id = 0
        self._last_loop_timestamp = None
        self._commanded_goal = None

        self._cf_client.open_link()
        if not self._cf_client.wait_until_connected(timeout=10.0):
            raise RuntimeError('Timed out waiting for Crazyflie connection')

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
            self._telemetry_client.stop()
            self._cf_client.close()
            self._started = False

    def set_goal(self, goal: Goal) -> None:
        with self._goal_lock:
            self._goal = goal

    def clear_goal(self) -> None:
        with self._goal_lock:
            self._goal = None
            self._commanded_goal = None

    def get_goal(self) -> Optional[Goal]:
        with self._goal_lock:
            return self._goal

    def get_latest_frame(self) -> dict:
        with self._state_lock:
            return {} if self._latest_frame is None else dict(self._latest_frame)

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

            now = drone_pose.timestamp
            dt = 0.0 if self._last_loop_timestamp is None else max(0.0, now - self._last_loop_timestamp)
            self._last_loop_timestamp = now

            with self._goal_lock:
                raw_goal = self._goal

            if raw_goal is None:
                self._cf_client.send_setpoint(0.0, 0.0, 0.0, 0)
                self._commanded_goal = None
                continue

            if self._commanded_goal is None:
                self._commanded_goal = Goal(
                    x=drone_pose.x,
                    y=drone_pose.y,
                    z=drone_pose.z,
                    heading=math.degrees(drone_pose.yaw) if raw_goal.heading is not None else None,
                )

            commanded_goal = self._slew_goal(self._commanded_goal, raw_goal, dt)
            self._commanded_goal = commanded_goal

            self._controller.add_sample(
                x=drone_pose.x,
                y=drone_pose.y,
                z=drone_pose.z,
                yaw=drone_pose.yaw,
                timestamp=drone_pose.timestamp,
            )
            command = self._controller.compute_command(commanded_goal)
            telemetry = self._telemetry_client.get_telemetry()

            self._logger.log_sample(
                runtime=runtime,
                drone_pose=drone_pose,
                raw_goal=raw_goal,
                commanded_goal=commanded_goal,
                command=command,
                telemetry=telemetry,
            )

            self._cf_client.send_setpoint(command.roll, command.pitch, command.yaw_rate, command.thrust)
            with self._state_lock:
                self._last_command = command

    def _slew_goal(self, current: Goal, target: Goal, dt: float) -> Goal:
        if dt <= 0.0:
            return current

        xy_step = self._xy_rate_limit_mps * dt
        z_step_up = self._z_up_rate_limit_mps * dt
        z_step_down = self._z_down_rate_limit_mps * dt
        heading_step = self._heading_rate_limit_dps * dt

        x = self._step_scalar(current.x, target.x, xy_step)
        y = self._step_scalar(current.y, target.y, xy_step)

        z_step = z_step_up if target.z >= current.z else z_step_down
        z = self._step_scalar(current.z, target.z, z_step)

        heading = current.heading
        if target.heading is None:
            heading = None
        elif heading is None:
            heading = target.heading
        else:
            heading = self._step_heading_deg(heading, target.heading, heading_step)

        return Goal(x=x, y=y, z=z, heading=heading)

    @staticmethod
    def _step_scalar(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    @staticmethod
    def _step_heading_deg(current: float, target: float, max_step_deg: float) -> float:
        error = (target - current + 180.0) % 360.0 - 180.0
        if abs(error) <= max_step_deg:
            return target
        return current + math.copysign(max_step_deg, error)
