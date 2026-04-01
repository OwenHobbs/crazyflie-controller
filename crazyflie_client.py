import threading

import cflib
from cflib.crazyflie import Crazyflie


class CrazyflieClient:
    """Small wrapper around the Bitcraze Crazyflie API."""

    def __init__(self, uri: str, cache_dir: str = './cache'):
        self.uri = uri
        self._cf = Crazyflie(rw_cache=cache_dir)
        self._connected_event = threading.Event()
        self._disconnected_event = threading.Event()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

    @property
    def cf(self) -> Crazyflie:
        return self._cf

    @staticmethod
    def init_drivers() -> None:
        cflib.crtp.init_drivers()

    def open_link(self) -> None:
        print(f'Connecting to {self.uri}')
        self._cf.open_link(self.uri)

    def wait_until_connected(self, timeout: float = 10.0) -> bool:
        return self._connected_event.wait(timeout)

    def unlock_thrust_protection(self) -> None:
        self.send_setpoint(0.0, 0.0, 0.0, 0)

    def send_setpoint(self, roll: float, pitch: float, yaw_rate: float, thrust: int) -> None:
        self._cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)

    def stop(self) -> None:
        self.send_setpoint(0.0, 0.0, 0.0, 0)

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self._cf.close_link()

    def _connected(self, link_uri: str) -> None:
        print(f'Connected to {link_uri}')
        self._connected_event.set()

    def _connection_failed(self, link_uri: str, msg: str) -> None:
        print(f'Connection to {link_uri} failed: {msg}')
        self._disconnected_event.set()

    def _connection_lost(self, link_uri: str, msg: str) -> None:
        print(f'Connection to {link_uri} lost: {msg}')
        self._disconnected_event.set()

    def _disconnected(self, link_uri: str) -> None:
        print(f'Disconnected from {link_uri}')
        self._disconnected_event.set()
