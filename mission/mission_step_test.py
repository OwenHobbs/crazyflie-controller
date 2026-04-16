import threading

from flight.flight_action import FlightActionHover
from flight.flight_control import Goal
from flight.flight_service import FlightService
from mission.mission_base import MissionBase

TAKEOFF_HEIGHT = 1.5
FLIGHT_X_STEP = 0.5
FLIGHT_Y_STEP = 0.5
FLIGHT_Z_STEP = -0.5
FLIGHT_HEADING_1 = 0
FLIGHT_HEADING_2 = 90
FLIGHT_DELAY = 8

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

        while not self._stop_event.is_set():
            self._set_goal_relative_to_start_pose(0, 0, TAKEOFF_HEIGHT, FLIGHT_HEADING_1)
            if self._wait(FLIGHT_DELAY): break

            self._set_goal_relative_to_start_pose(0, 0, TAKEOFF_HEIGHT, FLIGHT_HEADING_2)
            if self._wait(FLIGHT_DELAY): break

            self._set_goal_relative_to_start_pose(FLIGHT_X_STEP, FLIGHT_Y_STEP, TAKEOFF_HEIGHT + FLIGHT_Z_STEP, FLIGHT_HEADING_2)
            if self._wait(FLIGHT_DELAY): break

            self._set_goal_relative_to_start_pose(FLIGHT_X_STEP, FLIGHT_Y_STEP, TAKEOFF_HEIGHT + FLIGHT_Z_STEP, FLIGHT_HEADING_1)
            if self._wait(FLIGHT_DELAY): break