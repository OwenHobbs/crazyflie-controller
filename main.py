import logging
import time
import threading
import keyboard

from crazyflie_client import CrazyflieClient
from control import Goal
from flight_service import FlightService
from vicon_motion import ViconMotionClient

MOCAP_HOSTNAME = '128.101.167.111'
MOCAP_SYSTEM_TYPE = 'vicon'

CRAZYFLIE_URI_1 = 'radio://0/100/2M/E7E7E7E7E9'
DRONE_OBJECT_NAME_1 = '2026_Drone1'
CRAZYFLIE_URI_2 = 'radio://0/90/2M/E7E7E7E7E8'
DRONE_OBJECT_NAME_2 = '2026_Drone2'

TAKEOFF_HEIGHT = 1.5
TAKEOFF_HOLD_SECONDS = 10.0

logging.basicConfig(level=logging.ERROR)

# TODO: move this function to flight service or vicon?
def get_start_pos(
        flight_service: FlightService,
        stop_event: threading.Event,
        drone_object_name: str,
):
    start_pose = None
    while start_pose is None and not stop_event.is_set():
        start_pose = flight_service.get_latest_pose(drone_object_name)
        time.sleep(0.01)

    if start_pose is None:
        print('Could not get initial drone pose')
        flight_service.stop()
        keyboard.unhook_all_hotkeys()
        return None

    return start_pose

def main() -> None:
    CrazyflieClient.init_drivers()

    mocap_client = ViconMotionClient(MOCAP_HOSTNAME, MOCAP_SYSTEM_TYPE)
    mocap_client.start()

    flight_service_1 = FlightService(
        crazyflie_uri=CRAZYFLIE_URI_1,
        drone_object_name=DRONE_OBJECT_NAME_1,
        mocap_client=mocap_client,
        log_output_dir="flight_logs_1"
    )
    flight_service_2 = FlightService(
        crazyflie_uri=CRAZYFLIE_URI_2,
        drone_object_name=DRONE_OBJECT_NAME_2,
        mocap_client=mocap_client,
        log_output_dir="flight_logs_2"
    )

    stop_event = threading.Event()

    def on_esc():
        print('\nEsc pressed, shutting down')
        stop_event.set()

    keyboard.add_hotkey('esc', on_esc)

    flight_service_1.start()
    flight_service_2.start()

    start_pose_1 = get_start_pos(
        flight_service=flight_service_1,
        stop_event=stop_event,
        drone_object_name=DRONE_OBJECT_NAME_1
    )
    start_pose_2 = get_start_pos(
        flight_service=flight_service_2,
        stop_event=stop_event,
        drone_object_name=DRONE_OBJECT_NAME_2
    )

    print(f'start_pose_1: {start_pose_1}')
    print(f'start_pose_2: {start_pose_2}')

    try:
        while not stop_event.is_set():
            flight_service_1.set_goal(
                Goal(x=start_pose_1.x, y=start_pose_1.y, z=start_pose_1.z + TAKEOFF_HEIGHT)
            )
            flight_service_2.set_goal(
                Goal(x=start_pose_2.x, y=start_pose_2.y, z=start_pose_2.z + TAKEOFF_HEIGHT)
            )

            if stop_event.wait(TAKEOFF_HOLD_SECONDS):
                break

            flight_service_1.set_goal(
                Goal(x=start_pose_1.x + 1.0, y=start_pose_1.y, z=start_pose_1.z + TAKEOFF_HEIGHT)
            )
            flight_service_2.set_goal(
                Goal(x=start_pose_2.x + 1.0, y=start_pose_2.y, z=start_pose_2.z + TAKEOFF_HEIGHT)
            )

            if stop_event.wait(TAKEOFF_HOLD_SECONDS):
                break

    except KeyboardInterrupt:
        print('\nCtrl+C pressed, shutting down')
    finally:
        flight_service_1.stop()
        flight_service_2.stop()
        mocap_client.stop()
        keyboard.unhook_all_hotkeys()


if __name__ == '__main__':
    main()