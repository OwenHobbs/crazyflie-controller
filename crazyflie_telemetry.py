from __future__ import annotations

from dataclasses import asdict, dataclass
import threading
import time
from typing import Optional

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig


@dataclass
class TelemetrySnapshot:
    host_time: float = 0.0
    cf_time_ms: int = 0
    vbat: Optional[float] = None
    battery_level: Optional[int] = None
    pm_state: Optional[int] = None
    stabilizer_thrust: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    motor_m1: Optional[int] = None
    motor_m2: Optional[int] = None
    motor_m3: Optional[int] = None
    motor_m4: Optional[int] = None

    def to_dict(self) -> dict:
        return asdict(self)


class CrazyflieTelemetry:
    """Handles Crazyflie-side telemetry logging separate from the flight client."""

    def __init__(self, cf: Crazyflie, period_in_ms: int = 100):
        self._cf = cf
        self._period_in_ms = period_in_ms
        self._telemetry_lock = threading.Lock()
        self._telemetry = TelemetrySnapshot()
        self._log_configs: list[LogConfig] = []

    def start(self) -> None:
        self.stop() # remove current log callbacks if already logging

        power_log = LogConfig(name='PowerTelemetry', period_in_ms=self._period_in_ms)
        power_log.add_variable('pm.vbat', 'float')
        power_log.add_variable('pm.batteryLevel', 'uint8_t')
        power_log.add_variable('pm.state', 'int8_t')
        power_log.add_variable('stabilizer.thrust', 'float')

        attitude_motor_log = LogConfig(name='AttitudeMotorTelemetry', period_in_ms=self._period_in_ms)
        attitude_motor_log.add_variable('stabilizer.roll', 'float')
        attitude_motor_log.add_variable('stabilizer.pitch', 'float')
        attitude_motor_log.add_variable('stabilizer.yaw', 'float')
        attitude_motor_log.add_variable('motor.m1', 'uint16_t')
        attitude_motor_log.add_variable('motor.m2', 'uint16_t')
        attitude_motor_log.add_variable('motor.m3', 'uint16_t')
        attitude_motor_log.add_variable('motor.m4', 'uint16_t')

        self._log_configs = [power_log, attitude_motor_log]

        for logconf in self._log_configs:
            try:
                self._cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._on_log_data)
                logconf.error_cb.add_callback(self._on_log_error)
                logconf.start()
            except Exception as exc:
                print(f'Could not start log config {logconf.name}: {exc}')

    def stop(self) -> None:
        for logconf in self._log_configs:
            try:
                logconf.stop()
            except Exception:
                pass
        self._log_configs = []

    def get_telemetry(self) -> TelemetrySnapshot:
        with self._telemetry_lock:
            return TelemetrySnapshot(**self._telemetry.to_dict())

    def _on_log_data(self, timestamp: int, data: dict, logconf: LogConfig) -> None:
        del logconf
        with self._telemetry_lock:
            self._telemetry.host_time = time.time()
            self._telemetry.cf_time_ms = int(timestamp)

            if 'pm.vbat' in data:
                self._telemetry.vbat = float(data['pm.vbat'])
            if 'pm.batteryLevel' in data:
                self._telemetry.battery_level = int(data['pm.batteryLevel'])
            if 'pm.state' in data:
                self._telemetry.pm_state = int(data['pm.state'])
            if 'stabilizer.thrust' in data:
                self._telemetry.stabilizer_thrust = float(data['stabilizer.thrust'])

            if 'stabilizer.roll' in data:
                self._telemetry.roll = float(data['stabilizer.roll'])
            if 'stabilizer.pitch' in data:
                self._telemetry.pitch = float(data['stabilizer.pitch'])
            if 'stabilizer.yaw' in data:
                self._telemetry.yaw = float(data['stabilizer.yaw'])

            if 'motor.m1' in data:
                self._telemetry.motor_m1 = int(data['motor.m1'])
            if 'motor.m2' in data:
                self._telemetry.motor_m2 = int(data['motor.m2'])
            if 'motor.m3' in data:
                self._telemetry.motor_m3 = int(data['motor.m3'])
            if 'motor.m4' in data:
                self._telemetry.motor_m4 = int(data['motor.m4'])

    @staticmethod
    def _on_log_error(logconf: LogConfig, msg: str) -> None:
        print(f'Log error in {logconf.name}: {msg}')
