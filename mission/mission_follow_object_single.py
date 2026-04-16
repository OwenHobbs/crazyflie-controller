import threading

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

    def execute(self):
        # Initialize start pose
        drone_start_pose = self._flight_service.get_latest_pose(self._flight_service.drone_object_name)
        if drone_start_pose is None:
            print(f'Unable to find drone: {self._flight_service.drone_object_name}')
            return

        # Takeoff
        self._flight_service.set_goal(Goal(
            x = drone_start_pose.x,
            y = drone_start_pose.y,
            z = drone_start_pose.z + TAKEOFF_HEIGHT,
            heading = 0
        ))
        if self._wait(5): return

        while not self._stop_event.is_set():
            current_object_pose = self._flight_service.get_latest_pose(OBJECT_NAME)
            if current_object_pose is None:
                print(f'Unable to find object: {OBJECT_NAME}')
                # TODO: implement safe land
                break

            self._flight_service.set_goal(Goal(
                x = current_object_pose.x,
                y = current_object_pose.y,
                z = current_object_pose.z + TAKEOFF_HEIGHT
            ))

            if self._wait(0.1): break
