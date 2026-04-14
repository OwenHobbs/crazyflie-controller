import logging
import threading
import time

import keyboard

from crazyflie_client import CrazyflieClient
from flight_behavior_controller import BehaviorController
from flight_service import FlightService
from vicon_motion import ViconMotionClient

"""
Simple single-drone test flight for BehaviorController.
"""

MOCAP_HOSTNAME = '128.101.167.111'
MOCAP_SYSTEM_TYPE = 'vicon'

CRAZYFLIE_URI = 'radio://0/90/2M/E7E7E7E7E8'
DRONE_OBJECT_NAME = '2026_Drone2'

TAKEOFF_HEIGHT = 1.0
MOVE_DISTANCE = 0.5
MOVE_SPEED = 0.3

logging.basicConfig(level=logging.ERROR)


def wait_for_initial_pose(
    flight_service: FlightService,
    stop_event: threading.Event,
) -> bool:
    while not stop_event.is_set():
        if flight_service.get_latest_pose(DRONE_OBJECT_NAME) is not None:
            return True
        time.sleep(0.05)
    return False


def wait_until_idle(
    behavior_controller: BehaviorController,
    stop_event: threading.Event,
) -> bool:
    while not stop_event.is_set():
        if behavior_controller.wait_until_idle(timeout=0.1):
            return True
    return False


def main() -> None:
    CrazyflieClient.init_drivers()

    mocap_client = ViconMotionClient(MOCAP_HOSTNAME, MOCAP_SYSTEM_TYPE)
    flight_service = FlightService(
        crazyflie_uri=CRAZYFLIE_URI,
        drone_object_name=DRONE_OBJECT_NAME,
        mocap_client=mocap_client,
        log_output_dir='flight_logs_1',
    )
    behavior_controller = BehaviorController(
        flight_service=flight_service,
        drone_object_name=DRONE_OBJECT_NAME,
    )

    stop_event = threading.Event()

    def on_esc() -> None:
        print('\nEsc pressed, shutting down')
        stop_event.set()

    keyboard.add_hotkey('esc', on_esc)

    try:
        mocap_client.start()
        flight_service.start()
        behavior_controller.start()

        if not wait_for_initial_pose(flight_service, stop_event):
            return

        print('Taking off')
        behavior_controller.takeoff(height=TAKEOFF_HEIGHT)
        if not wait_until_idle(behavior_controller, stop_event):
            return

        print('Moving forward')
        behavior_controller.move_by(dx=MOVE_DISTANCE, dy=0.0, dz=0.0, speed=MOVE_SPEED)
        if not wait_until_idle(behavior_controller, stop_event):
            return

        print('Hovering')
        behavior_controller.hover()
        if stop_event.wait(2.0):
            return

        print('Landing')
        behavior_controller.land(descent_speed=MOVE_SPEED)
        wait_until_idle(behavior_controller, stop_event)
    except KeyboardInterrupt:
        print('\nCtrl+C pressed, shutting down')
    finally:
        behavior_controller.stop()
        flight_service.stop()
        mocap_client.stop()
        keyboard.unhook_all_hotkeys()


if __name__ == '__main__':
    main()
