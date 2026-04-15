import threading
import time
from abc import abstractmethod, ABC

from flight.flight_service import FlightService


# This class is to be inherited by individual missions
# and contains common functions that may be useful
# ABC: Abstract Base Class
class MissionBase(ABC):

    def __init__(
        self,
        stop_event: threading.Event,
    ):
        self._stop_event = stop_event

    @abstractmethod
    def execute(self):
        pass

    def _wait(self, timeout: float) -> bool:
        return self._stop_event.wait(timeout)

    def _wait_for_start_pose(
        self,
        flight_service: FlightService, # used for mocap
        object_name: str | None = None
    ):
        if object_name is None:
            object_name = flight_service.drone_object_name

        start_pose = None
        while start_pose is None and not self._stop_event.is_set():
            start_pose = flight_service.get_latest_pose(object_name)
            time.sleep(0.01)

        if start_pose is None:
            print(f'Could not get start pose for {object_name}')
            self._stop_event.set()
            return None

        return start_pose
