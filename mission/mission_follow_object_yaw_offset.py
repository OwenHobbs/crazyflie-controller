import threading
import math

from flight.flight_control import Goal
from flight.flight_service import FlightService
from mission.mission_base import MissionBase

OBJECT_NAME = 'box'
TAKEOFF_HEIGHT = 1.0

class MissionFollowObjectYawOffset(MissionBase):

    def __init__(
        self,
        stop_event: threading.Event,
        flight_service_1: FlightService,
        flight_service_2: FlightService
    ):
        super().__init__(stop_event)
        self._flight_service_1 = flight_service_1
        self._flight_service_2 = flight_service_2

    def execute(self):
        # Initialize start pose
        drone_start_pose_1 = self._flight_service_1.get_latest_pose(self._flight_service_1.drone_object_name)
        if drone_start_pose_1 is None:
            print(f'Unable to find drone 1: {self._flight_service_1.drone_object_name}')
            return
        drone_start_pose_2 = self._flight_service_2.get_latest_pose(self._flight_service_2.drone_object_name)
        if drone_start_pose_2 is None:
            print(f'Unable to find drone 2: {self._flight_service_2.drone_object_name}')
            return

        # Takeoff
        self._flight_service_1.set_goal(Goal(
            x = drone_start_pose_1.x,
            y = drone_start_pose_1.y,
            z = drone_start_pose_1.z + TAKEOFF_HEIGHT,
            heading = 0
        ))
        self._flight_service_2.set_goal(Goal(
            x=drone_start_pose_2.x,
            y=drone_start_pose_2.y,
            z=drone_start_pose_2.z + TAKEOFF_HEIGHT,
            heading=0
        ))
        if self._wait(5): return

        while not self._stop_event.is_set():
            current_object_pose = self._flight_service_1.get_latest_pose(OBJECT_NAME)

            if current_object_pose is None:
                print(f'Unable to find object: {OBJECT_NAME}')
                # TODO: implement safe land
                break

            self._flight_service_1.set_goal(Goal(
                x = current_object_pose.x,
                y = current_object_pose.y + 0.25,
                z = current_object_pose.z + TAKEOFF_HEIGHT,
                heading = math.degrees(current_object_pose.yaw)
            ))

            leader_pose = self._flight_service_1.get_latest_pose(self._flight_service_1.drone_object_name)

            print(f'leader_pose: {leader_pose}')
            
            offset_x = 0.5 * math.sin(leader_pose.yaw)
            offset_y = 0.5 * math.cos(leader_pose.yaw) * -1

            self._flight_service_2.set_goal(Goal(
                x=leader_pose.x + offset_x,
                y=leader_pose.y + offset_y,
                z=leader_pose.z,
                heading=math.degrees(leader_pose.yaw)
            ))

            if self._wait(0.1): break