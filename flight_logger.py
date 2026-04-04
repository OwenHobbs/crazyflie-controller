from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np


class FlightLogger:
    """Logs flight data, shows optional live plots, and saves files after the run."""

    def __init__(self, base_output_dir: str = "flight_logs", run_timestamp: str | None = None):
        timestamp = run_timestamp or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.base_output_dir = Path(base_output_dir)
        self.output_dir = self.base_output_dir / timestamp
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.rows: list[dict] = []

    def log_sample(
        self,
        runtime,
        drone_pose,
        goal,
        command,
        telemetry: Optional[object] = None,
    ) -> None:
        telemetry_dict = telemetry.to_dict() if telemetry is not None else {}

        row = {
            "runtime": float(runtime),
            "drone_x": float(drone_pose.x),
            "drone_y": float(drone_pose.y),
            "drone_z": float(drone_pose.z),
            "drone_yaw": float(drone_pose.yaw),
            "goal_x": float(goal.x),
            "goal_y": float(goal.y),
            "goal_z": float(goal.z),
            "goal_heading_deg": "" if goal.heading is None else float(goal.heading),
            "roll_cmd": float(command.roll),
            "pitch_cmd": float(command.pitch),
            "yaw_rate_cmd": float(command.yaw_rate),
            "thrust_cmd": int(command.thrust),
            "cf_host_time": telemetry_dict.get("host_time", ""),
            "cf_time_ms": telemetry_dict.get("cf_time_ms", ""),
            "cf_vbat": telemetry_dict.get("vbat", ""),
            "cf_battery_level": telemetry_dict.get("battery_level", ""),
            "cf_pm_state": telemetry_dict.get("pm_state", ""),
            "cf_stabilizer_thrust": telemetry_dict.get("stabilizer_thrust", ""),
            "cf_roll": telemetry_dict.get("roll", ""),
            "cf_pitch": telemetry_dict.get("pitch", ""),
            "cf_yaw": telemetry_dict.get("yaw", ""),
            "cf_motor_m1": telemetry_dict.get("motor_m1", ""),
            "cf_motor_m2": telemetry_dict.get("motor_m2", ""),
            "cf_motor_m3": telemetry_dict.get("motor_m3", ""),
            "cf_motor_m4": telemetry_dict.get("motor_m4", ""),
        }
        self.rows.append(row)

    def save_all(self) -> None:
        if not self.rows:
            print(f"FlightLogger: no samples to save in {self.output_dir.resolve()}")
            return

        self._save_csv()
        self._save_position_plot()
        self._save_command_plot()
        self._save_crazyflie_telemetry_plot()
        print(f"FlightLogger: saved {len(self.rows)} samples to {self.output_dir.resolve()}")

    def _save_csv(self) -> None:
        csv_path = self.output_dir / "flight_log.csv"
        fieldnames = list(self.rows[0].keys())

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

    def _save_position_plot(self) -> None:
        t = np.asarray([row["runtime"] for row in self.rows], dtype=float)
        drone_heading_deg_wrapped = np.rad2deg(np.asarray([row["drone_yaw"] for row in self.rows], dtype=float))
        drone_heading_deg = self._unwrap_deg_array(drone_heading_deg_wrapped)
        goal_heading_deg_wrapped = np.asarray([self._num(row["goal_heading_deg"]) for row in self.rows], dtype=float)
        goal_heading_deg = self._nearest_equivalent_goal_series(goal_heading_deg_wrapped, drone_heading_deg)

        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

        axes[0].plot(t, [row["drone_x"] for row in self.rows], label="x")
        axes[0].plot(t, [row["goal_x"] for row in self.rows], "--", label="x goal")
        axes[0].set_ylabel("x (m)")
        axes[0].set_title("Position and Heading vs Goal")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(t, [row["drone_y"] for row in self.rows], label="y")
        axes[1].plot(t, [row["goal_y"] for row in self.rows], "--", label="y goal")
        axes[1].set_ylabel("y (m)")
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(t, [row["drone_z"] for row in self.rows], label="z")
        axes[2].plot(t, [row["goal_z"] for row in self.rows], "--", label="z goal")
        axes[2].set_ylabel("z (m)")
        axes[2].legend()
        axes[2].grid(True)

        axes[3].plot(t, drone_heading_deg, label="heading")
        axes[3].plot(t, goal_heading_deg, "--", label="heading goal")
        axes[3].set_xlabel("Time (s)")
        axes[3].set_ylabel("Heading (deg)")
        axes[3].legend()
        axes[3].grid(True)

        fig.tight_layout()
        fig.savefig(self.output_dir / "position_vs_goal.png", dpi=150)
        plt.close(fig)

    def _save_command_plot(self) -> None:
        t = np.asarray([row["runtime"] for row in self.rows], dtype=float)

        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

        axes[0].plot(t, [row["roll_cmd"] for row in self.rows], label="roll cmd (deg)")
        axes[0].plot(t, [row["pitch_cmd"] for row in self.rows], label="pitch cmd (deg)")
        axes[0].set_ylabel("Tilt (deg)")
        axes[0].set_title("Controller Commands")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(t, [row["yaw_rate_cmd"] for row in self.rows], label="yaw rate cmd (deg/s)")
        axes[1].set_ylabel("Yaw rate (deg/s)")
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(t, [row["thrust_cmd"] for row in self.rows], label="thrust cmd")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel("Thrust")
        axes[2].legend()
        axes[2].grid(True)

        fig.tight_layout()
        fig.savefig(self.output_dir / "commands.png", dpi=150)
        plt.close(fig)

    def _save_crazyflie_telemetry_plot(self) -> None:
        t = np.asarray([row["runtime"] for row in self.rows], dtype=float)
        vbat = np.asarray([self._num(row["cf_vbat"]) for row in self.rows], dtype=float)
        battery_level = np.asarray([self._num(row["cf_battery_level"]) for row in self.rows], dtype=float)
        pm_state = np.asarray([self._num(row["cf_pm_state"]) for row in self.rows], dtype=float)
        stabilizer_thrust = np.asarray([self._num(row["cf_stabilizer_thrust"]) for row in self.rows], dtype=float)
        roll = np.asarray([self._num(row["cf_roll"]) for row in self.rows], dtype=float)
        pitch = np.asarray([self._num(row["cf_pitch"]) for row in self.rows], dtype=float)
        yaw = np.asarray([self._num(row["cf_yaw"]) for row in self.rows], dtype=float)
        m1 = np.asarray([self._num(row["cf_motor_m1"]) for row in self.rows], dtype=float)
        m2 = np.asarray([self._num(row["cf_motor_m2"]) for row in self.rows], dtype=float)
        m3 = np.asarray([self._num(row["cf_motor_m3"]) for row in self.rows], dtype=float)
        m4 = np.asarray([self._num(row["cf_motor_m4"]) for row in self.rows], dtype=float)

        fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True)

        axes[0].plot(t, vbat, linewidth=2, label="battery voltage (V)")
        axes[0].set_ylabel("Voltage (V)")
        axes[0].set_title("Crazyflie Telemetry")
        axes[0].grid(True)

        valid_vbat = vbat[np.isfinite(vbat)]
        if valid_vbat.size > 0:
            vmin = float(np.min(valid_vbat))
            vmax = float(np.max(valid_vbat))
            span = max(vmax - vmin, 0.05)
            pad = max(0.03, 0.2 * span)
            axes[0].set_ylim(vmin - pad, vmax + pad)
        else:
            axes[0].set_ylim(3.0, 4.5)

        ax0_right = axes[0].twinx()
        ax0_right.plot(t, battery_level, "--", alpha=0.75, label="battery level (%)")
        ax0_right.set_ylabel("Battery (%)")
        ax0_right.set_ylim(0, 100)

        left_handles, left_labels = axes[0].get_legend_handles_labels()
        right_handles, right_labels = ax0_right.get_legend_handles_labels()
        axes[0].legend(left_handles + right_handles, left_labels + right_labels, loc="upper right")

        axes[1].step(t, pm_state, where="post", label="pm.state")
        axes[1].set_ylabel("PM state")
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(t, stabilizer_thrust, label="stabilizer thrust")
        axes[2].set_ylabel("CF thrust")
        axes[2].legend()
        axes[2].grid(True)

        axes[3].plot(t, roll, label="roll (deg)")
        axes[3].plot(t, pitch, label="pitch (deg)")
        axes[3].plot(t, yaw, label="yaw (deg)")
        axes[3].set_ylabel("Attitude (deg)")
        axes[3].legend()
        axes[3].grid(True)

        axes[4].plot(t, m1, label="m1")
        axes[4].plot(t, m2, label="m2")
        axes[4].plot(t, m3, label="m3")
        axes[4].plot(t, m4, label="m4")
        axes[4].set_xlabel("Time (s)")
        axes[4].set_ylabel("Motor PWM")
        axes[4].legend()
        axes[4].grid(True)

        fig.tight_layout()
        fig.savefig(self.output_dir / "crazyflie_telemetry.png", dpi=150)
        plt.close(fig)

    @staticmethod
    def _num(value):
        if value == "" or value is None:
            return float("nan")
        return float(value)

    @staticmethod
    def _unwrap_deg_array(values_deg: np.ndarray) -> np.ndarray:
        return np.rad2deg(np.unwrap(np.deg2rad(values_deg)))

    @staticmethod
    def _nearest_equivalent_goal_series(goal_wrapped_deg: np.ndarray, reference_unwrapped_deg: np.ndarray) -> np.ndarray:
        out = np.full_like(goal_wrapped_deg, np.nan, dtype=float)
        valid = np.isfinite(goal_wrapped_deg)
        if not np.any(valid):
            return out

        for i, goal_deg in enumerate(goal_wrapped_deg):
            if not np.isfinite(goal_deg):
                continue
            ref = reference_unwrapped_deg[i]
            out[i] = goal_deg + 360.0 * round((ref - goal_deg) / 360.0)
        return out
