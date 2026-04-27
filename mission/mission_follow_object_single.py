import threading

from flight.flight_action import FlightActionMoveTo, FlightActionHover
from flight.flight_control import Goal
from flight.flight_service import FlightService
from mission.mission_base import MissionBase

OBJECT_NAME = 'box'
TAKEOFF_HEIGHT = 1.0

class MissionFollowObjectSingle(MissionBase):

    def __init__(
        self,
        stop_event: threading.Event,
        flight_service: FlightService
    ):
        super().__init__(stop_event)
        self._flight_service = flight_service
        self._object_pose = None

    def update_object_pose(self):
        new_object_pose = self._flight_service.get_latest_pose(OBJECT_NAME)
        if new_object_pose is None:
            print(f'Unable to find object: {OBJECT_NAME}')
            return True # indicate error
        else:
            self._object_pose = new_object_pose
            return False

    def execute(self):
        # Initialize drone_start_pose
        drone_start_pose = self._flight_service.get_latest_pose(self._flight_service.drone_object_name)
        if drone_start_pose is None:
            print(f'Unable to find drone_start_pose: {self._flight_service.drone_object_name}')
            return
        else:
            print(f'drone_start_pose: {drone_start_pose}')

        # Ensure we are able to find the object pose before taking off
        if self.update_object_pose(): return

        # Takeoff
        self._flight_service.set_action(FlightActionMoveTo(Goal(
            x = drone_start_pose.x,
            y = drone_start_pose.y,
            z = drone_start_pose.z + TAKEOFF_HEIGHT,
            heading = 0
        )))
        if self._flight_service.wait_for_action_handoff(self._stop_event): return
        if self._wait(4): return

        while not self._stop_event.is_set():
            self.update_object_pose()

            # TODO: could change to FlightActionMoveTo if distance change is large
            self._flight_service.set_action(FlightActionHover(Goal(
                x = self._object_pose.x,
                y = self._object_pose.y,
                z = self._object_pose.z + TAKEOFF_HEIGHT
            )))

            if self._wait(0.1): break
