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
    yaw_rate: float  # deg/s
    thrust: int


@dataclass
class PIDGains:
    # Horizontal position control. Use body-frame position error and measured
    # body-frame velocity damping.
    kpx: float = 1.35  
    kpy: float = 0.95  
    kdx: float = 1.00
    kdy: float = 1.15

    # Small alignment trim to compensate for slight Vicon/body-frame mismatch.
    body_heading_offset_deg: float = 0.0  # Reverted

    # Altitude control in raw Crazyflie thrust units.
    hover_thrust: int = 53000  # Drone 1 = 54850, Drone 2 = 53000
    kpz_thrust_per_m: float = 7000.0  # Reverted
    kdz_thrust_per_mps: float = 7000.0  # Reverted
    kiz_thrust_per_m_s: float = 800.0  # Reduced from 1500.0
    z_integral_limit: float = 0.40  # Reverted
    z_integral_active_error_m: float = 0.30  # Reverted
    z_integral_bleed: float = 0.997  # Reverted
    min_thrust_cmd: int = 20000
    max_thrust_cmd: int = 65535

    # Heading control.
    kp_heading: float = 0.6
    kd_heading: float = 0.15
    heading_deadband_deg: float = 4.0
    max_yaw_rate_deg: float = 25.0
    yaw_command_sign: float = -1.0

    # Attitude command limits.
    max_roll_deg: float = 12.0
    max_pitch_deg: float = 12.0


class PIDPositionController:
    """Position + heading controller.

    x/y use body-frame PD control with measured body-frame velocity damping.
    z uses a hover-thrust-centered PID with conditional integration.
    """

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

        self._z_integral = 0.0

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

        current_x = x_pos[-1]
        current_y = y_pos[-1]
        current_z = z_pos[-1]
        current_yaw = yaw_pos[-1] + math.radians(self.gains.body_heading_offset_deg)

        x_error_global = goal.x - current_x
        y_error_global = goal.y - current_y
        z_error = goal.z - current_z

        x_error_local, y_error_local = self._rotate_world_to_body(
            x_error_global,
            y_error_global,
            current_yaw,
        )

        vx_world = self._safe_derivative(x_pos, times)
        vy_world = self._safe_derivative(y_pos, times)
        vz_world = self._safe_derivative(z_pos, times)
        vx_local, vy_local = self._rotate_world_to_body(vx_world, vy_world, current_yaw)

        theta = self.gains.kpx * x_error_local - self.gains.kdx * vx_local
        phi = self.gains.kpy * y_error_local - self.gains.kdy * vy_local

        pitch_deg = float(np.rad2deg(theta))
        roll_deg = float(-np.rad2deg(phi))
        pitch_deg = float(np.clip(pitch_deg, -self.gains.max_pitch_deg, self.gains.max_pitch_deg))
        roll_deg = float(np.clip(roll_deg, -self.gains.max_roll_deg, self.gains.max_roll_deg))

        thrust_cmd = self._compute_thrust_command(z_error=z_error, z_velocity=vz_world)
        yaw_rate_deg = self._compute_yaw_rate_command(goal_heading_deg=goal.heading, yaw_pos=yaw_pos, times=times)

        return ControlCommand(
            roll=roll_deg,
            pitch=pitch_deg,
            yaw_rate=yaw_rate_deg,
            thrust=thrust_cmd,
        )

    def _compute_thrust_command(self, z_error: float, z_velocity: float) -> int:
        dt = self._latest_dt()

        p_term = self.gains.kpz_thrust_per_m * z_error
        d_term = -self.gains.kdz_thrust_per_mps * z_velocity
        i_term = self.gains.kiz_thrust_per_m_s * self._z_integral
        unsat_thrust_cmd = self.gains.hover_thrust + p_term + d_term + i_term

        thrust_saturated = (
            unsat_thrust_cmd <= self.gains.min_thrust_cmd
            or unsat_thrust_cmd >= self.gains.max_thrust_cmd
        )

        if dt > 0.0:
            if abs(z_error) < self.gains.z_integral_active_error_m and not thrust_saturated:
                self._z_integral += z_error * dt
                self._z_integral = float(np.clip(
                    self._z_integral,
                    -self.gains.z_integral_limit,
                    self.gains.z_integral_limit,
                ))
            else:
                self._z_integral *= self.gains.z_integral_bleed

        i_term = self.gains.kiz_thrust_per_m_s * self._z_integral
        thrust_cmd = int(round(self.gains.hover_thrust + p_term + d_term + i_term))
        thrust_cmd = int(np.clip(thrust_cmd, self.gains.min_thrust_cmd, self.gains.max_thrust_cmd))
        return thrust_cmd

    def _compute_yaw_rate_command(
        self,
        goal_heading_deg: float | None,
        yaw_pos: np.ndarray,
        times: np.ndarray,
    ) -> float:
        if goal_heading_deg is None or len(yaw_pos) == 0:
            return 0.0

        yaw_unwrapped = np.unwrap(yaw_pos)
        current_yaw_unwrapped = yaw_unwrapped[-1]
        current_yaw_wrapped = self._wrap_angle(current_yaw_unwrapped)

        goal_heading_rad = math.radians(goal_heading_deg)
        heading_error_wrapped = self._wrap_angle(goal_heading_rad - current_yaw_wrapped)

        if abs(math.degrees(heading_error_wrapped)) <= self.gains.heading_deadband_deg:
            return 0.0

        goal_heading_unwrapped = current_yaw_unwrapped + heading_error_wrapped
        yaw_rate_measured = self._safe_derivative(yaw_unwrapped, times)

        yaw_rate_rad = (
            self.gains.kp_heading * (goal_heading_unwrapped - current_yaw_unwrapped)
            - self.gains.kd_heading * yaw_rate_measured
        )
        yaw_rate_deg = self.gains.yaw_command_sign * math.degrees(yaw_rate_rad)
        yaw_rate_deg = float(np.clip(
            yaw_rate_deg,
            -self.gains.max_yaw_rate_deg,
            self.gains.max_yaw_rate_deg,
        ))
        return yaw_rate_deg

    def _latest_dt(self) -> float:
        if len(self._time_history) < 2:
            return 0.0
        dt = float(self._time_history[-1] - self._time_history[-2])
        return dt if dt > 0.0 else 0.0

    @staticmethod
    def _rotate_world_to_body(x_world: float, y_world: float, yaw_rad: float) -> tuple[float, float]:
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        x_body = cos_yaw * x_world + sin_yaw * y_world
        y_body = -sin_yaw * x_world + cos_yaw * y_world
        return x_body, y_body

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