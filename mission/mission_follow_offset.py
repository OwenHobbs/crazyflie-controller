import threading
import math

from flight.flight_action import FlightActionHover, FlightActionFollowAtOffset
from flight.flight_control import Goal
from flight.flight_service import FlightService
from mission.mission_base import MissionBase

TAKEOFF_HEIGHT = 1.5
OBJECT_NAME = 'box'
LEADER_NAME = '2026_Drone_Blue'

class MissionStepTest(MissionBase):

    def __init__(
        self,
        stop_event: threading.Event,
        flight_service_1: FlightService,
        flight_service_2: FlightService | None,
    ):
        super().__init__(stop_event)
        self._flight_service_1 = flight_service_1
        self._flight_service_2 = flight_service_2
        self._use_two_drones = flight_service_2 is not None
        self._start_pose_1 = None
        self._start_pose_2 = None

    # TODO: move to parent class since this is a common function?
    def _initialize_start_poses(self):
        self._start_pose_1 = self._wait_for_start_pose(self._flight_service_1)
        print(f'start_pose_1: {self._start_pose_1}')
        if self._use_two_drones:
            self._start_pose_2 = self._wait_for_start_pose(self._flight_service_2)
            print(f'start_pose_2: {self._start_pose_2}')

    # Helper function to easily control both drones
    # Assumes start poses is already initialized
    def _set_goal_relative_to_start_pose(
        self,
        offset_x: float,
        offset_y: float,
        offset_z: float,
        # This is the absolute heading, TODO: make offset_heading?
        heading: float | None = None
    ):
        self._flight_service_1.set_action(FlightActionHover(
            Goal(x=self._start_pose_1.x + offset_x, y=self._start_pose_1.y + offset_y, z=self._start_pose_1.z + offset_z, heading=heading)
        ))
        if self._use_two_drones:
            self._flight_service_2.set_action(FlightActionHover(
                Goal(x=self._start_pose_2.x + offset_x, y=self._start_pose_2.y + offset_y, z=self._start_pose_2.z + offset_z, heading=heading)
            ))

    def execute(self):
        self._initialize_start_poses()

        self._flight_service_1.set_action(FlightActionHover(
            Goal(x=self._start_pose_1.x, y=self._start_pose_1.y, z=self._start_pose_1.z + TAKEOFF_HEIGHT, heading=0)
        ))
        if self._use_two_drones:
            self._flight_service_2.set_action(FlightActionHover(
                Goal(x=self._start_pose_2.x, y=self._start_pose_2.y, z=self._start_pose_2.z + TAKEOFF_HEIGHT, heading=0)
            ))
        if self._wait(5): return

        while not self._stop_event.is_set():

            current_object_pose = self._flight_service_1.get_latest_pose(OBJECT_NAME)

            if current_object_pose is None:
                print(f'Unable to find object: {OBJECT_NAME}')
                # TODO: implement safe land
                break

            self._flight_service_1.set_action(FlightActionHover(
                Goal(
                    x = current_object_pose.x,
                    y = current_object_pose.y + 0.25,
                    z = current_object_pose.z + TAKEOFF_HEIGHT,
                    heading = math.degrees(current_object_pose.yaw)
                ))
            )
            
            if self._use_two_drones:
                self._flight_service_2.set_action(FlightActionFollowAtOffset(
                    drone_object_name=LEADER_NAME,
                    distance=0.5
                ))

            
            if self._wait(0.1): break