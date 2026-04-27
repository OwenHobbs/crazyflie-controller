import queue
import threading

from flight.flight_action import FlightActionMoveTo
from flight.flight_control import Goal
from flight.flight_service import FlightService
from mission.mission_base import MissionBase


class MissionUserInputSingle(MissionBase):

    def __init__(
        self,
        stop_event: threading.Event,
        flight_service: FlightService
    ):
        super().__init__(stop_event)
        self._flight_service = flight_service
        self._start_pose = None
        self._current_goal = None

    def _input_float(self, prompt: str, default: float | None = None) -> float | None:
        # Retry until the user enters a number or the mission is stopped.
        while not self._stop_event.is_set():
            result_queue = queue.Queue(maxsize=1)

            def read_input():
                try:
                    # TODO: Replace daemon-threaded input; it can crash Python
                    # during shutdown if Esc stops the mission while input() is blocked.
                    result_queue.put(input(f'{prompt} [{default}]: '))
                except EOFError:
                    result_queue.put(None)

            # Keep input() off the mission thread so Esc can stop the mission
            # while the console is waiting for the user to type.
            threading.Thread(target=read_input, daemon=True).start()

            while not self._stop_event.is_set():
                try:
                    raw = result_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if raw is None:
                    return None

                raw = raw.strip()
                if raw == "" and default is not None:
                    return default

                try:
                    return float(raw)
                except ValueError:
                    print("Please enter a number.")
                    # Restart the prompt without recursing or stacking callers.
                    break

        return None

    def relative_move(self, delta_x, delta_y, delta_z):
        new_goal = Goal(
            x = self._current_goal.x + delta_x,
            y = self._current_goal.y + delta_y,
            z = self._current_goal.z + delta_z
        )
        self._current_goal = new_goal

        print(f'Moving to {self._current_goal}')
        self._flight_service.set_action(FlightActionMoveTo(self._current_goal))
        if self._flight_service.wait_for_action_handoff(self._stop_event): return
        print(f'Movement complete\n')

    def execute(self):
        # Initialize start_pose
        self._start_pose = self._flight_service.get_latest_pose(self._flight_service.drone_object_name)
        if self._start_pose is None:
            print(f'Unable to find start_pose: {self._flight_service.drone_object_name}')
            return
        else:
            print(f'start_pose_1: {self._start_pose}')
        self._current_goal = self._start_pose

        takeoff_height = self._input_float('Enter takeoff height', 1.0)
        if takeoff_height is None: return
        self.relative_move(0, 0, takeoff_height)

        while not self._stop_event.is_set():
            delta_x = self._input_float('Enter delta x', 0.0)
            if delta_x is None: break

            delta_y = self._input_float('Enter delta y', 0.0)
            if delta_y is None: break

            delta_z = self._input_float('Enter delta z', 0.0)
            if delta_z is None: break

            self.relative_move(delta_x, delta_y, delta_z)

            if self._wait(0.1): break
