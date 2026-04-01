import logging
import time

import keyboard
from cflib.utils import uri_helper

from crazyflie_client import CrazyflieClient
from crazyflie_telemetry import CrazyflieTelemetry
from control import Goal, PIDPositionController
from vicon_motion import ViconMotionClient
from flight_logger import FlightLogger


URI = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E7E9')
TIME_SIZE = 5

HOST_NAME = '128.101.167.111'
MOCAP_SYSTEM_TYPE = 'vicon'
DRONE_OBJECT_NAME = '2026_Drone1'
GROUND_OBJECT_NAME = 'box'

TAKEOFF_HEIGHT = 1.5
TAKEOFF_HOLD_SECONDS = 5.0


logging.basicConfig(level=logging.ERROR)


def build_goal(start_pose, ground_pose, runtime: float) -> Goal:
    if runtime < TAKEOFF_HOLD_SECONDS or ground_pose is None:
        return Goal(x=start_pose.x, y=start_pose.y, z=TAKEOFF_HEIGHT)

    return Goal(x=ground_pose.x, y=ground_pose.y, z=ground_pose.z + TAKEOFF_HEIGHT)


def main() -> None:
    CrazyflieClient.init_drivers()

    cf = CrazyflieClient(URI)
    telemetry_client = CrazyflieTelemetry(cf.cf)
    mocap = ViconMotionClient(HOST_NAME, MOCAP_SYSTEM_TYPE)
    controller = PIDPositionController(window_size=TIME_SIZE)
    logger = FlightLogger()

    cf.open_link()
    if not cf.wait_until_connected(timeout=10.0):
        print('Timed out waiting for Crazyflie connection')
        cf.close()
        return

    telemetry_client.start()

    start_pose = mocap.get_pose(DRONE_OBJECT_NAME)
    start_time = time.time()

    cf.unlock_thrust_protection()

    try:
        while not keyboard.is_pressed('esc'):
            frame = mocap.wait_for_frame()

            drone_pose = frame.get(DRONE_OBJECT_NAME)
            if drone_pose is None:
                continue

            ground_pose = frame.get(GROUND_OBJECT_NAME)
            runtime = time.time() - start_time
            goal = build_goal(start_pose, ground_pose, runtime)

            controller.add_sample(
                x=drone_pose.x,
                y=drone_pose.y,
                z=drone_pose.z,
                timestamp=drone_pose.timestamp,
            )

            command = controller.compute_command(
                current_yaw=drone_pose.yaw,
                goal_x=goal.x,
                goal_y=goal.y,
                goal_z=goal.z,
            )

            telemetry = telemetry_client.get_telemetry()

            logger.log_sample(
                runtime=runtime,
                drone_pose=drone_pose,
                goal=goal,
                command=command,
                ground_pose=ground_pose,
                telemetry=telemetry,
            )

            cf.send_setpoint(command.roll, command.pitch, 0.0, command.thrust)

    except KeyboardInterrupt:
        pass
    finally:
        print('\nKill button pressed, shutting down')
        logger.save_all()
        telemetry_client.stop()
        cf.close()


if __name__ == '__main__':
    main()
