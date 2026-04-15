import logging
import time
import threading
import keyboard

from crazyflie_client import CrazyflieClient
from flight_control import Goal, PIDGains
from flight_service import FlightService
from vicon_motion import ViconMotionClient

"""
Main entry point for the Crazyflie drone control application.
Orchestrates the entire flight system by connecting motion capture (vicon_motion),
drone communication (crazyflie_client), flight control (flight_service, flight_control),
and data logging (flight_logger). Handles user input and goal sequencing.
"""

MOCAP_HOSTNAME = '128.101.167.111'
MOCAP_SYSTEM_TYPE = 'vicon'

BLUE_CRAZYFLIE_URI = 'radio://0/90/2M/E7E7E7E7E8'
BLUE_CRAZYFLIE_OBJECT_NAME = '2026_Drone_Blue'
BLUE_CRAZYFLIE_GAINS = PIDGains(
    kpz=1.2,
    kdz=1.2,
    hover_thrust_cmd=58100,
)

RED_CRAZYFLIE_URI = 'radio://0/100/2M/E7E7E7E7E9'
RED_CRAZYFLIE_OBJECT_NAME = '2026_Drone_Red'
RED_CRAZYFLIE_GAINS = PIDGains(
    kpz=2.8,
    kdz=0.8,
    kpz_down=1.1,
    kdz_down=1.0,
    hover_thrust_cmd=54250,
)

TAKEOFF_HEIGHT = 1.5
FLIGHT_X_STEP = 0.5
FLIGHT_Y_STEP = 0.5
FLIGHT_Z_STEP = -0.5
FLIGHT_HEADING_1 = 0
FLIGHT_HEADING_2 = 90
FLIGHT_DELAY = 8
USE_TWO_DRONES = False

logging.basicConfig(level=logging.ERROR)

# Get the initial position of a drone from motion capture
def get_start_pos(
        flight_service: FlightService,
        stop_event: threading.Event
):
    start_pose = None
    while start_pose is None and not stop_event.is_set():
        start_pose = flight_service.get_latest_pose(flight_service.drone_object_name)
        time.sleep(0.01)

    if start_pose is None:
        print('Could not get initial drone pose')
        flight_service.stop()
        keyboard.unhook_all_hotkeys()
        return None

    return start_pose

# Main application entry point
def main() -> None:
    CrazyflieClient.init_drivers()

    mocap_client = ViconMotionClient(MOCAP_HOSTNAME, MOCAP_SYSTEM_TYPE)
    mocap_client.start()

    # Change between RED and BLUE as necessary
    flight_service_1 = FlightService(
        crazyflie_uri=BLUE_CRAZYFLIE_URI,
        drone_object_name=BLUE_CRAZYFLIE_OBJECT_NAME,
        mocap_client=mocap_client,
        gains=BLUE_CRAZYFLIE_GAINS
    )
    if USE_TWO_DRONES:
        flight_service_2 = FlightService(
            crazyflie_uri=RED_CRAZYFLIE_URI,
            drone_object_name=RED_CRAZYFLIE_OBJECT_NAME,
            mocap_client=mocap_client,
            gains=RED_CRAZYFLIE_GAINS
        )

    stop_event = threading.Event()

    def on_esc():
        print('\nEsc pressed, shutting down')
        stop_event.set()

    keyboard.add_hotkey('esc', on_esc)

    flight_service_1.start()
    if USE_TWO_DRONES:
        flight_service_2.start()

    start_pose_1 = get_start_pos(
        flight_service=flight_service_1,
        stop_event=stop_event,
    )
    if USE_TWO_DRONES:
        start_pose_2 = get_start_pos(
            flight_service=flight_service_2,
            stop_event=stop_event,
        )

    print(f'start_pose_1: {start_pose_1}')
    if USE_TWO_DRONES:
        print(f'start_pose_2: {start_pose_2}')

    try:
        while not stop_event.is_set():
            flight_service_1.set_goal(
                Goal(x=start_pose_1.x, y=start_pose_1.y, z=start_pose_1.z + TAKEOFF_HEIGHT, heading=FLIGHT_HEADING_1)
            )
            if USE_TWO_DRONES:
                flight_service_2.set_goal(
                    Goal(x=start_pose_2.x, y=start_pose_2.y, z=start_pose_2.z + TAKEOFF_HEIGHT, heading=FLIGHT_HEADING_1)
                )

            if stop_event.wait(FLIGHT_DELAY):
                break

            flight_service_1.set_goal(
                Goal(x=start_pose_1.x, y=start_pose_1.y, z=start_pose_1.z + TAKEOFF_HEIGHT, heading=FLIGHT_HEADING_2)
            )
            if USE_TWO_DRONES:
                flight_service_2.set_goal(
                    Goal(x=start_pose_2.x, y=start_pose_2.y, z=start_pose_2.z + TAKEOFF_HEIGHT, heading=FLIGHT_HEADING_2)
                )

            if stop_event.wait(FLIGHT_DELAY):
                break

            flight_service_1.set_goal(
                Goal(x=start_pose_1.x + FLIGHT_X_STEP, y=start_pose_1.y + FLIGHT_Y_STEP, z=start_pose_1.z + TAKEOFF_HEIGHT + FLIGHT_Z_STEP, heading=FLIGHT_HEADING_2)
            )
            if USE_TWO_DRONES:
                flight_service_2.set_goal(
                    Goal(x=start_pose_2.x + FLIGHT_X_STEP, y=start_pose_2.y + FLIGHT_Y_STEP, z=start_pose_2.z + TAKEOFF_HEIGHT + FLIGHT_Z_STEP, heading=FLIGHT_HEADING_2)
                )

            if stop_event.wait(FLIGHT_DELAY):
                break

            flight_service_1.set_goal(
                Goal(x=start_pose_1.x + FLIGHT_X_STEP, y=start_pose_1.y + FLIGHT_Y_STEP, z=start_pose_1.z + TAKEOFF_HEIGHT + FLIGHT_Z_STEP, heading=FLIGHT_HEADING_1)
            )
            if USE_TWO_DRONES:
                flight_service_2.set_goal(
                    Goal(x=start_pose_2.x + FLIGHT_X_STEP, y=start_pose_2.y + FLIGHT_Y_STEP, z=start_pose_2.z + TAKEOFF_HEIGHT + FLIGHT_Z_STEP, heading=FLIGHT_HEADING_1)
                )

            if stop_event.wait(FLIGHT_DELAY):
                break

    except KeyboardInterrupt:
        print('\nCtrl+C pressed, shutting down')
    finally:
        flight_service_1.stop()
        if USE_TWO_DRONES:
            flight_service_2.stop()
        mocap_client.stop()
        keyboard.unhook_all_hotkeys()


# Run the application
if __name__ == '__main__':
    main()
