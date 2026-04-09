import threading

import cflib
from cflib.crazyflie import Crazyflie

"""
This module provides a wrapper client for communicating with Crazyflie drones.
Handles connection management, setpoint commands, and telemetry callbacks.
Used by flight_service.py to send control commands and receive drone state.
"""

# Wrapper client for Bitcraze Crazyflie drone communication
class CrazyflieClient:
    """Small wrapper around the Bitcraze Crazyflie API."""

    # Initialize the Crazyflie client with connection URI
    def __init__(self, uri: str, cache_dir: str = './cache'):
        self.uri = uri
        self._cf = Crazyflie(rw_cache=cache_dir)
        self._connected_event = threading.Event()
        self._disconnected_event = threading.Event()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

    # Get the underlying Crazyflie object
    @property
    def cf(self) -> Crazyflie:
        return self._cf

    # Initialize CRTP drivers for drone communication
    @staticmethod
    def init_drivers() -> None:
        cflib.crtp.init_drivers()

    # Open the connection to the drone
    def open_link(self) -> None:
        print(f'Connecting to {self.uri}')
        self._cf.open_link(self.uri)

    # Wait for the drone to connect with optional timeout
    def wait_until_connected(self, timeout: float = 10.0) -> bool:
        return self._connected_event.wait(timeout)

    # Unlock thrust protection by sending a zero setpoint
    def unlock_thrust_protection(self) -> None:
        self.send_setpoint(0.0, 0.0, 0.0, 0)

    # Send a control setpoint to the drone
    def send_setpoint(self, roll: float, pitch: float, yaw_rate: float, thrust: int) -> None:
        self._cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)

    # Stop the drone by sending zero commands
    def stop(self) -> None:
        self.send_setpoint(0.0, 0.0, 0.0, 0)

    # Close the connection to the drone
    def close(self) -> None:
        try:
            self.stop()
        finally:
            self._cf.close_link()

    # Handle successful connection event
    def _connected(self, link_uri: str) -> None:
        print(f'Connected to {link_uri}')
        self._connected_event.set()

    # Handle connection failure event
    def _connection_failed(self, link_uri: str, msg: str) -> None:
        print(f'Connection to {link_uri} failed: {msg}')
        self._disconnected_event.set()

    # Handle connection lost event
    def _connection_lost(self, link_uri: str, msg: str) -> None:
        print(f'Connection to {link_uri} lost: {msg}')
        self._disconnected_event.set()

    # Handle disconnection event
    def _disconnected(self, link_uri: str) -> None:
        print(f'Disconnected from {link_uri}')
        self._disconnected_event.set()
