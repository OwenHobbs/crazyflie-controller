from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Goal:
    x: float
    y: float
    z: float
    heading: float | None = None  # degrees in Vicon world frame


@dataclass
class ControlCommand:
    roll: float
    pitch: float
    yaw_rate: float  # deg/s sent to Crazyflie commander
    thrust: int


@dataclass
class PIDGains:
    kpx: float = 1.5291
    kpy: float = 1.5291
    kpz: float = 2.4
    kdx: float = 0.8155
    kdy: float = 0.8155
    kdz: float = 0.4104

    # Heading control in the Vicon/world frame.
    # yaw_command_sign exists because the Crazyflie yaw-rate sign can be opposite
    # to the sign of the yaw angle coming from Vicon.
    kp_heading: float = 0.6
    kd_heading: float = 0.15
    heading_deadband_deg: float = 4.0
    max_yaw_rate_deg: float = 25.0
    yaw_command_sign: float = -1.0

    max_roll_deg: float = 15.0
    max_pitch_deg: float = 15.0


class PIDPositionController:
    """Position + heading controller using the same basic logic as the original script."""

    def __init__(self, window_size: int = 5, gains: PIDGains | None = None):
        self.window_size = window_size
        self.gains = gains or PIDGains()
        self.reset()

    def reset(self) -> None:
        self._x_history = deque(maxlen=self.window_size)
        self._y_history = deque(maxlen=self.window_size)
        self._z_history = deque(maxlen=self.window_size)
        self._yaw_history = deque(maxlen=self.window_size)  # radians from Vicon
        self._time_history = deque(maxlen=self.window_size)

    def add_sample(self, x: float, y: float, z: float, yaw: float, timestamp: float) -> None:
        self._x_history.append(x)
        self._y_history.append(y)
        self._z_history.append(z)
        self._yaw_history.append(yaw)
        self._time_history.append(timestamp)

    def compute_command(self, goal: Goal) -> ControlCommand:
        x_pos = np.asarray(self._x_history, dtype=float)
        y_pos = np.asarray(self._y_history, dtype=float)
        z_pos = np.asarray(self._z_history, dtype=float)
        yaw_pos = np.asarray(self._yaw_history, dtype=float)
        times = np.asarray(self._time_history, dtype=float)

        if len(x_pos) == 0:
            return ControlCommand(roll=0.0, pitch=0.0, yaw_rate=0.0, thrust=0)

        x_error_global = goal.x - x_pos
        y_error_global = goal.y - y_pos
        z_error = goal.z - z_pos

        x_error_local = np.empty_like(x_error_global)
        y_error_local = np.empty_like(y_error_global)

        for i in range(len(x_error_global)):
            cos_yaw = math.cos(yaw_pos[i])
            sin_yaw = math.sin(yaw_pos[i])
            rotation = np.array([
                [cos_yaw, sin_yaw, 0.0],
                [-sin_yaw, cos_yaw, 0.0],
                [0.0, 0.0, 1.0],
            ])
            global_errors = np.array([[x_error_global[i]], [y_error_global[i]], [z_error[i]]])
            local_errors = rotation @ global_errors
            x_error_local[i] = local_errors[0, 0]
            y_error_local[i] = local_errors[1, 0]

        dx = self._safe_derivative(x_error_local, times)
        dy = self._safe_derivative(y_error_local, times)
        dz = self._safe_derivative(z_error, times)

        theta = self.gains.kpx * x_error_local[-1] + self.gains.kdx * dx
        phi = self.gains.kpy * y_error_local[-1] + self.gains.kdy * dy
        thrust = self.gains.kpz * z_error[-1] + self.gains.kdz * dz

        pitch_deg = np.rad2deg(theta)
        roll_deg = -np.rad2deg(phi)

        pitch_deg = float(np.clip(pitch_deg, -self.gains.max_pitch_deg, self.gains.max_pitch_deg))
        roll_deg = float(np.clip(roll_deg, -self.gains.max_roll_deg, self.gains.max_roll_deg))

        thrust_cmd = thrust * (50000 / 2.3346) + 10001
        thrust_cmd = int(np.clip(thrust_cmd, 0, 0xFFFF))

        yaw_rate_deg = 0.0
        if goal.heading is not None:
            goal_heading_rad = math.radians(goal.heading)
            current_heading_rad = yaw_pos[-1]
            heading_error_rad = self._wrap_angle(goal_heading_rad - current_heading_rad)
            heading_error_deg = math.degrees(heading_error_rad)

            # Use unwrapped heading history to estimate the actual world-frame yaw rate.
            yaw_unwrapped = np.unwrap(yaw_pos)
            current_yaw_rate_rad_s = self._safe_derivative(yaw_unwrapped, times)

            effective_heading_error_deg = self._apply_deadband_deg(
                heading_error_deg,
                self.gains.heading_deadband_deg,
            )
            effective_heading_error_rad = math.radians(effective_heading_error_deg)

            desired_world_yaw_rate_rad_s = (
                self.gains.kp_heading * effective_heading_error_rad
                - self.gains.kd_heading * current_yaw_rate_rad_s
            )

            yaw_rate_deg = self.gains.yaw_command_sign * math.degrees(desired_world_yaw_rate_rad_s)
            yaw_rate_deg = float(np.clip(
                yaw_rate_deg,
                -self.gains.max_yaw_rate_deg,
                self.gains.max_yaw_rate_deg,
            ))

        return ControlCommand(
            roll=roll_deg,
            pitch=pitch_deg,
            yaw_rate=yaw_rate_deg,
            thrust=thrust_cmd,
        )

    @staticmethod
    def _safe_derivative(values: np.ndarray, times: np.ndarray) -> float:
        if len(values) < 2 or len(times) < 2:
            return 0.0

        dt = np.gradient(times)
        if np.any(np.isclose(dt, 0.0)):
            return 0.0

        return float(np.mean(np.gradient(values) / dt))

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _apply_deadband_deg(value_deg: float, deadband_deg: float) -> float:
        if abs(value_deg) <= deadband_deg:
            return 0.0
        return math.copysign(abs(value_deg) - deadband_deg, value_deg)
