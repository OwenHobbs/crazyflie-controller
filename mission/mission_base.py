import threading
from abc import abstractmethod, ABC

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
