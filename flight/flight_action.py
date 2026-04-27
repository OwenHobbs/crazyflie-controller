from __future__ import annotations
import time
import math
from abc import ABC, abstractmethod

from flight.flight_control import Goal
from mocap.mocap_client import Pose


class FlightAction(ABC):
    def __init__(self):
        self._action_start_time: float | None = None
        self._handoff_action: FlightAction | None = None
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
            self._action_start_time = time.monotonic()
            self._action_runtime = 0
            # Run callback for initial execution
            self.on_initial_execution()
        else:
            self._action_runtime = time.monotonic() - self._action_start_time

        return self.determine_goal(), self._handoff_action

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
    def __init__(
        self,
        landing_height: float = 0.0,
        descent_rate: float = 0.05,
        touchdown_tolerance: float = 0.02,
    ):
        super().__init__()
        self._landing_height = landing_height
        self._descent_rate = descent_rate
        self._touchdown_tolerance = touchdown_tolerance
        self._start_pose: Pose | None = None

    def on_initial_execution(self):
        self._start_pose = self._drone_pose

    def determine_goal(self) -> Goal | None:
        # Check if we are at landing height
        if self._drone_pose.z <= self._landing_height + self._touchdown_tolerance:
            self._handoff_action = FlightActionIdle()
            return None

        target_z = max(
            self._landing_height,
            self._start_pose.z - self._descent_rate * self._action_runtime,
        )
        return Goal(
            x=self._start_pose.x,
            y=self._start_pose.y,
            z=target_z,
            heading=math.degrees(self._start_pose.yaw),
        )

class FlightActionMoveTo(FlightAction):
    def __init__(
            self,
            destination_goal: Goal,
            xy_rate: float = 0.6,
            xy_tolerance: float = 0.2,
            z_rate: float = 0.4,
            z_tolerance: float = 0.2
    ):
        super().__init__()
        self._destination_goal: Goal = destination_goal
        self._xy_rate = xy_rate
        self._xy_tolerance = xy_tolerance
        self._z_rate = z_rate
        self._z_tolerance = z_tolerance
        self._start_pose: Pose | None = None

    def on_initial_execution(self):
        self._start_pose = self._drone_pose

    def determine_goal(self) -> Goal | None:
        # Check if we are at destination
        xy_error = math.hypot(
            self._destination_goal.x - self._drone_pose.x,
            self._destination_goal.y - self._drone_pose.y,
        )
        z_error = self._destination_goal.z - self._drone_pose.z
        if xy_error <= self._xy_tolerance and abs(z_error) <= self._z_tolerance:
            self._handoff_action = FlightActionHover(self._destination_goal)
            return None

        destination_dx = self._destination_goal.x - self._start_pose.x
        destination_dy = self._destination_goal.y - self._start_pose.y
        destination_xy_distance = math.hypot(destination_dx, destination_dy)

        if destination_xy_distance > 0:
            xy_distance_traveled = min(
                destination_xy_distance,
                self._xy_rate * self._action_runtime,
            )
            xy_progress = xy_distance_traveled / destination_xy_distance
            target_x = self._start_pose.x + destination_dx * xy_progress
            target_y = self._start_pose.y + destination_dy * xy_progress
        else:
            target_x = self._destination_goal.x
            target_y = self._destination_goal.y

        destination_dz = self._destination_goal.z - self._start_pose.z
        z_distance_traveled = min(
            abs(destination_dz),
            self._z_rate * self._action_runtime,
        )
        target_z = self._start_pose.z + math.copysign(z_distance_traveled, destination_dz)

        return Goal(
            x=target_x,
            y=target_y,
            z=target_z,
            heading=self._destination_goal.heading,
        )

# TODO: This action has not been tested yet
class FlightActionFollowAtOffset(FlightAction):
    def __init__(self, drone_object_name: str, distance: float):
        super().__init__()
        self._drone_object_name = drone_object_name
        self._distance = distance

    def determine_goal(self) -> Goal | None:
        leader_pose = self._frame[self._drone_object_name]
        
        offset_x = self._distance * math.sin(leader_pose.yaw)
        offset_y = self._distance * math.cos(leader_pose.yaw) * -1

        return Goal(
            x=leader_pose.x + offset_x,
            y=leader_pose.y + offset_y,
            z=leader_pose.z,
            heading= math.degrees(leader_pose.yaw)
        )