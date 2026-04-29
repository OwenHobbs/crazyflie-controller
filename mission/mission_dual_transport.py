import threading

from flight.flight_action import FlightActionLand, FlightActionMoveTo
from flight.flight_control import Goal
from flight.flight_service import FlightService
from mission.mission_base import MissionBase

OBJECT_NAME = 'box'
TAKEOFF_HEIGHT = 1.0

class MissionDualTransport(MissionBase):

    def __init__(
        self,
        stop_event: threading.Event,
        flight_service_1: FlightService,
        flight_service_2: FlightService,
    ):
        super().__init__(stop_event)
        self._flight_service_1 = flight_service_1
        self._flight_service_2 = flight_service_2
        self._start_pose_1 = None
        self._start_pose_2 = None
        self._object_pose = None

    def _initialize_poses(self):
        # Initialize start_pose_1
        self._start_pose_1 = self._flight_service_1.get_latest_pose(self._flight_service_1.drone_object_name)
        if self._start_pose_1 is None:
            print(f'Unable to find start_pose_1: {self._flight_service_1.drone_object_name}')
            return True # indicate error
        else:
            print(f'start_pose_1: {self._start_pose_1}')

        # Initialize start_pose_2
        self._start_pose_2 = self._flight_service_2.get_latest_pose(self._flight_service_2.drone_object_name)
        if self._start_pose_2 is None:
            print(f'Unable to find start_pose_2: {self._flight_service_2.drone_object_name}')
            return True # indicate error
        else:
            print(f'start_pose_2: {self._start_pose_2}')

        # Initialize object_pose
        self._object_pose = self._flight_service_2.get_latest_pose(OBJECT_NAME)
        if self._object_pose is None:
            print(f'Unable to find object_pose: {OBJECT_NAME}')
            return True # indicate error
        else:
            print(f'object_pose: {self._object_pose}')

        return False

    # Helper function to easily control both drones
    # Assumes start poses is already initialized
    def _move_relative_to_start_pose(
        self,
        offset_x: float,
        offset_y: float,
        offset_z: float,
        # This is the absolute heading, TODO: make offset_heading?
        heading: float | None = None,
        wait: bool = True,
    ):
        self._flight_service_1.set_action(FlightActionMoveTo(
            Goal(x=self._start_pose_1.x + offset_x, y=self._start_pose_1.y + offset_y, z=self._start_pose_1.z + offset_z, heading=heading)
        ))
        self._flight_service_2.set_action(FlightActionMoveTo(
            Goal(x=self._start_pose_2.x + offset_x, y=self._start_pose_2.y + offset_y, z=self._start_pose_2.z + offset_z, heading=heading)
        ))

        if wait:
            if self._flight_service_1.wait_for_action_handoff(self._stop_event): return
            if self._flight_service_2.wait_for_action_handoff(self._stop_event): return

    def execute(self):
        if self._initialize_poses(): return

        # Determine which flight service is further positive on x-axis
        if self._start_pose_1.x > self._start_pose_2.x:
            pos_x_flight_service = self._flight_service_1
            neg_x_flight_service = self._flight_service_2
        else:
            pos_x_flight_service = self._flight_service_2
            neg_x_flight_service = self._flight_service_1

        # Takeoff
        self._move_relative_to_start_pose(0, 0, TAKEOFF_HEIGHT)
        if self._wait(3): return

        # Move to object
        pos_x_flight_service.set_action(FlightActionMoveTo(Goal(
            x = self._object_pose.x + 0.5,
            y = self._object_pose.y,
            z = self._object_pose.z + TAKEOFF_HEIGHT,
        )))
        neg_x_flight_service.set_action(FlightActionMoveTo(Goal(
            x=self._object_pose.x - 0.5,
            y=self._object_pose.y,
            z=self._object_pose.z + TAKEOFF_HEIGHT,
        )))
        if pos_x_flight_service.wait_for_action_handoff(self._stop_event): return
        if neg_x_flight_service.wait_for_action_handoff(self._stop_event): return
        if self._wait(3): return

        # Drop package
        pos_x_flight_service.set_action(FlightActionMoveTo(Goal(
            x=self._object_pose.x + 0.5,
            y=self._object_pose.y,
            z=self._object_pose.z + 0.2,
        )))
        neg_x_flight_service.set_action(FlightActionMoveTo(Goal(
            x=self._object_pose.x - 0.5,
            y=self._object_pose.y,
            z=self._object_pose.z + 0.2,
        )))
        if pos_x_flight_service.wait_for_action_handoff(self._stop_event): return
        if neg_x_flight_service.wait_for_action_handoff(self._stop_event): return
        if self._wait(3): return

        # Return to height
        pos_x_flight_service.set_action(FlightActionMoveTo(Goal(
            x=self._object_pose.x + 0.5,
            y=self._object_pose.y,
            z=self._object_pose.z + TAKEOFF_HEIGHT,
        )))
        neg_x_flight_service.set_action(FlightActionMoveTo(Goal(
            x=self._object_pose.x - 0.5,
            y=self._object_pose.y,
            z=self._object_pose.z + TAKEOFF_HEIGHT,
        )))
        if pos_x_flight_service.wait_for_action_handoff(self._stop_event): return
        if neg_x_flight_service.wait_for_action_handoff(self._stop_event): return
        if self._wait(3): return

        # Return to base
        self._move_relative_to_start_pose(0, 0, TAKEOFF_HEIGHT)
        if self._wait(3): return

        # Land
        self._flight_service_1.set_action(FlightActionLand(
            landing_height=self._start_pose_1.z
        ))
        self._flight_service_2.set_action(FlightActionLand(
            landing_height=self._start_pose_2.z
        ))
        if self._flight_service_1.wait_for_action_handoff(self._stop_event): return
        if self._flight_service_2.wait_for_action_handoff(self._stop_event): return
