from __future__ import annotations
import time
import math
from abc import ABC, abstractmethod

from flight.flight_control import Goal
from mocap.mocap_client import Pose


class FlightAction(ABC):
    def __init__(self):
        self._action_start_time: float | None = None
        self._new_action: FlightAction | None = None
        self._drone_pose: Pose | None = None
        self._frame: dict[str, Pose] | None = None
        self._flight_runtime: float | None = None
        self._action_runtime: float | None = None

    def on_initial_execution(self):
        pass

    @abstractmethod
    def determine_goal(self) -> Goal | None:
        pass

    # Return the goal to fly and next FlightAction if to be changed
    def execute(
        self,
        drone_pose: Pose,
        frame: dict[str, Pose],
        flight_runtime: float,
    ) -> tuple[Goal | None, FlightAction | None]:
        # Update parameters that could be useful in self.determine_goal
        # Or these values could be passed to determine_goal
        self._drone_pose = drone_pose
        self._frame = frame
        self._flight_runtime = flight_runtime

        if self._action_start_time is None:
            self._action_start_time = time.time()
            self._action_runtime = 0
            # Run callback for initial execution
            self.on_initial_execution()
        else:
            self._action_runtime = time.time() - self._action_start_time

        return self.determine_goal(), self._new_action

class FlightActionIdle(FlightAction):
    def determine_goal(self) -> Goal | None:
        return None

class FlightActionHover(FlightAction):
    def __init__(self, desired_goal: Goal):
        super().__init__()
        self._desired_goal = desired_goal

    def determine_goal(self) -> Goal | None:
        return self._desired_goal

class FlightActionLand(FlightAction):
    def __init__(self, location):
        super().__init__()
        self.object_name = location

    def on_initial_execution(self):
        self._initial_position = self._drone_pose

    def set_object_goal(self):
        objPose = self.frame.get(self.object_name)
        z_real = self._initial_position.z + 1/(5*(objPose.z - self._initial_position.z))*math.pow((self._action_runtime), 2)
        if (z_real < 0.005):
            z_real = 0.005
        self._desired_goal.x = objPose.x
        self._desired_goal.y = objPose.y
        self._desired_goal.z = z_real
        self._desired_goal.heading = math.rad2deg(objPose.yaw)
    
    def determine_goal(self) -> Goal | None:
        set_object_goal(self)
        return self._desired_goal
