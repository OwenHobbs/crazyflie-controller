import logging
import threading
import keyboard

from crazyflie.crazyflie_client import CrazyflieClient
from flight.flight_control import PIDGains
from flight.flight_service import FlightService
from mission.mission_follow_object import MissionFollowObject
from mission.mission_step_test import MissionStepTest
from mocap.mocap_client import MocapClient

"""
Main entry point for the Crazyflie drone control application.
Orchestrates the entire flight system by connecting the mocap, Crazyflie,
and flight packages. Handles user input and goal sequencing.
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

USE_TWO_DRONES = True

logging.basicConfig(level=logging.ERROR)

# Main application entry point
def main() -> None:
    CrazyflieClient.init_drivers()

    mocap_client = MocapClient(MOCAP_HOSTNAME, MOCAP_SYSTEM_TYPE)
    mocap_client.start()

    # TODO: Change between RED and BLUE as necessary
    flight_service_1 = FlightService(
        crazyflie_uri=BLUE_CRAZYFLIE_URI,
        drone_object_name=BLUE_CRAZYFLIE_OBJECT_NAME,
        mocap_client=mocap_client,
        gains=BLUE_CRAZYFLIE_GAINS
    )
    flight_service_2 = None
    if USE_TWO_DRONES:
        flight_service_2 = FlightService(
            crazyflie_uri=RED_CRAZYFLIE_URI,
            drone_object_name=RED_CRAZYFLIE_OBJECT_NAME,
            mocap_client=mocap_client,
            gains=RED_CRAZYFLIE_GAINS
        )

    # Create stop_event
    stop_event = threading.Event()
    def on_esc():
        print('\nEsc pressed, shutting down')
        stop_event.set()
    keyboard.add_hotkey('esc', on_esc)

    flight_service_1.start()
    if USE_TWO_DRONES:
        flight_service_2.start()

    try:
        # TODO: Initialize desired mission here

        # MissionStepTest(
        #     stop_event=stop_event,
        #     flight_service_1=flight_service_1,
        #     flight_service_2=flight_service_2
        # ).execute()

        MissionFollowObject(
            stop_event=stop_event,
            flight_service_1=flight_service_1,
            flight_service_2=flight_service_2
        ).execute()
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
