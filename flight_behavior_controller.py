from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Optional

from flight_pid_controller import Goal
from flight_service import FlightService
from vicon_motion import Pose

"""
This module provides a higher-level non-blocking behavior layer on top of FlightService.
Each BehaviorController owns a lightweight background thread that can generate and update
high-level goals over time for a single drone.
"""


@dataclass
class _BehaviorCommand:
    mode: str
    target: Goal
    speed: float | None = None


class BehaviorController:
    """Non-blocking high-level motion layer for one drone.

    This class does not replace FlightService. FlightService still owns the real-time
    mocap/control/send loop. BehaviorController only generates and updates Goal objects
    over time and feeds them into FlightService.set_goal().
    """

    def __init__(
        self,
        flight_service: FlightService,
        drone_object_name: str,
        update_period: float = 0.05,
        position_tolerance: float = 0.05,
        heading_tolerance_deg: float = 5.0,
    ):
        self._flight_service = flight_service
        self._drone_object_name = drone_object_name
        self._update_period = update_period
        self._position_tolerance = position_tolerance
        self._heading_tolerance_deg = heading_tolerance_deg

        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._command: Optional[_BehaviorCommand] = None
        self._active_goal: Optional[Goal] = None
        self._busy = False
        self._home_pose: Optional[Pose] = None

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False

    def start(self) -> None:
        if self._started:
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            name=f'{self._drone_object_name}_Behavior',
            daemon=True,
        )
        self._thread.start()
        self._started = True

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

        if self._thread is not None:
            self._thread.join(timeout=2.0)

        with self._condition:
            self._command = None
            self._busy = False
            self._condition.notify_all()

        self._started = False

    def hover(
        self,
        z: float | None = None,
        heading: float | None = None,
    ) -> None:
        pose = self._require_pose()
        if pose is None:
            return

        goal = Goal(
            x=pose.x,
            y=pose.y,
            z=pose.z if z is None else z,
            heading=self._resolve_heading_deg(pose, heading),
        )
        self._submit_command(_BehaviorCommand(mode='hold', target=goal))

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        heading: float | None = None,
        speed: float | None = None,
    ) -> None:
        goal = Goal(x=x, y=y, z=z, heading=heading)
        self._submit_command(_BehaviorCommand(mode='move_to', target=goal, speed=speed))

    def move_by(
        self,
        dx: float,
        dy: float,
        dz: float,
        heading: float | None = None,
        speed: float | None = None,
    ) -> None:
        pose = self._require_pose()
        if pose is None:
            return

        goal = Goal(
            x=pose.x + dx,
            y=pose.y + dy,
            z=pose.z + dz,
            heading=self._resolve_heading_deg(pose, heading),
        )
        self._submit_command(_BehaviorCommand(mode='move_to', target=goal, speed=speed))

    def rotate_to(
        self,
        heading: float,
        speed: float | None = None,
    ) -> None:
        pose = self._require_pose()
        if pose is None:
            return

        goal = Goal(x=pose.x, y=pose.y, z=pose.z, heading=heading)
        self._submit_command(_BehaviorCommand(mode='move_to', target=goal, speed=speed))

    def takeoff(
        self,
        height: float,
        heading: float | None = None,
        speed: float | None = None,
    ) -> None:
        pose = self._require_pose()
        if pose is None:
            return

        goal = Goal(
            x=pose.x,
            y=pose.y,
            z=pose.z + height,
            heading=self._resolve_heading_deg(pose, heading),
        )
        self._submit_command(_BehaviorCommand(mode='move_to', target=goal, speed=speed))

    def land(
        self,
        descent_speed: float = 0.2,
        final_z: float | None = None,
        heading: float | None = None,
    ) -> None:
        pose = self._require_pose()
        if pose is None:
            return

        if final_z is None:
            if self._home_pose is not None:
                final_z = self._home_pose.z
            else:
                final_z = pose.z

        goal = Goal(
            x=pose.x,
            y=pose.y,
            z=final_z,
            heading=self._resolve_heading_deg(pose, heading),
        )
        self._submit_command(_BehaviorCommand(mode='land', target=goal, speed=descent_speed))

    def cancel(self) -> None:
        with self._condition:
            self._command = None
            self._active_goal = None
            self._busy = False
            self._condition.notify_all()

    def stop_and_hold(self) -> None:
        self.hover()

    def is_busy(self) -> bool:
        with self._lock:
            return self._busy

    def wait_until_idle(self, timeout: float | None = None) -> bool:
        with self._condition:
            finished = self._condition.wait_for(
                lambda: not self._busy or self._stop_event.is_set(),
                timeout=timeout,
            )
            return bool(finished and not self._busy)

    def get_current_pose(self) -> Optional[Pose]:
        pose = self._flight_service.get_latest_pose(self._drone_object_name)
        if pose is not None and self._home_pose is None:
            with self._lock:
                if self._home_pose is None:
                    self._home_pose = pose
        return pose

    def get_active_goal(self) -> Optional[Goal]:
        with self._lock:
            return self._active_goal

    def _submit_command(self, command: _BehaviorCommand) -> None:
        with self._condition:
            self._command = command
            self._active_goal = None
            self._busy = True
            self._condition.notify_all()

    def _run_loop(self) -> None:
        last_update_time = time.time()

        while not self._stop_event.is_set():
            with self._condition:
                command = self._command
                if command is None:
                    self._condition.wait(timeout=self._update_period)
                    last_update_time = time.time()
                    continue

            pose = self.get_current_pose()
            if pose is None:
                time.sleep(self._update_period)
                last_update_time = time.time()
                continue

            now = time.time()
            dt = max(now - last_update_time, self._update_period)
            last_update_time = now

            goal_to_send = self._compute_goal_for_step(
                pose=pose,
                command=command,
                dt=dt,
            )
            self._flight_service.set_goal(goal_to_send)

            with self._lock:
                self._active_goal = goal_to_send

            if self._is_goal_reached(pose, command.target):
                with self._condition:
                    if self._command is command:
                        self._command = None
                        self._busy = False
                        self._active_goal = command.target
                    self._condition.notify_all()
            else:
                with self._condition:
                    if self._command is command:
                        self._busy = True

            time.sleep(self._update_period)

    def _compute_goal_for_step(
        self,
        pose: Pose,
        command: _BehaviorCommand,
        dt: float,
    ) -> Goal:
        target = command.target
        speed = command.speed
        if speed is None or speed <= 0.0:
            return target

        start_goal = self._get_step_start_goal(pose, target)

        max_step = max(speed * dt, 1.0e-6)
        dx = target.x - start_goal.x
        dy = target.y - start_goal.y
        dz = target.z - start_goal.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= max_step:
            return target

        scale = max_step / distance
        return Goal(
            x=start_goal.x + dx * scale,
            y=start_goal.y + dy * scale,
            z=start_goal.z + dz * scale,
            heading=target.heading,
        )

    def _get_step_start_goal(self, pose: Pose, target: Goal) -> Goal:
        with self._lock:
            active_goal = self._active_goal

        if active_goal is None:
            return Goal(
                x=pose.x,
                y=pose.y,
                z=pose.z,
                heading=target.heading,
            )

        return active_goal

    def _is_goal_reached(self, pose: Pose, goal: Goal) -> bool:
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        dz = goal.z - pose.z
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        if position_error > self._position_tolerance:
            return False

        if goal.heading is None:
            return True

        pose_heading_deg = math.degrees(pose.yaw)
        heading_error_deg = self._wrap_degrees(goal.heading - pose_heading_deg)
        return abs(heading_error_deg) <= self._heading_tolerance_deg

    def _require_pose(self) -> Optional[Pose]:
        return self.get_current_pose()

    @staticmethod
    def _wrap_degrees(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _resolve_heading_deg(pose: Pose, heading: float | None) -> float | None:
        if heading is not None:
            return heading
        return math.degrees(pose.yaw)
