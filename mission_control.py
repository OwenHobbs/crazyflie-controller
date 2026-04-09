import logging
import time
import threading
import keyboard

from crazyflie_client import CrazyflieClient
from flight_control import Goal
from flight_service import FlightService
from vicon_motion import ViconMotionClient

class MissionControl:
    def __init__(self, flight_service):
        self.flight_service = flight_service

    def time_control(self, x_goal, y_goal, z_goal, yaw_goal, x_start, y_start, z_start, start_time): 
        #TODO further expand to allow user controlled velocity, step x and y
        drone_pos = self.flight_service.get_latest_pose(self.flight_service._drone_object_name)
        z_real = z_start + 1/(5*(z_goal - z_start))*math.pow((drone_pos.timestamp-start_time), 2)

        if (z_goal - z_start < 0 and z_real < z_goal):
            z_real = z_goal
        
        if (z_goal - z_start > 0 and z_real > z_goal):
            z_real = z_goal

        if (abs(drone_pos.x - x_goal) < 0.005 & abs(drone_pos.y - y_goal) < 0.005 & abs(drone_pos.z - z_goal) < 0.005 & abs(drone_pos.yaw - yaw_goal) < 0.005):
            return x_goal, y_goal, z_goal, yaw_goal, True
        return x_goal, y_goal, z_real, yaw_goal, False

    def go_to(self, x_goal, y_goal, z_goal, yaw_goal): #right now this blocks until goal is reached
        if (z_goal < 0):
            z_goal = 0
        drone_pos = self.flight_service.get_latest_pose(self.flight_service._drone_object_name)
        goal_reached = False
        while not goal_reached
            x_pos, y_pos, z_pos, yaw_pos, goal_reached = time_control(x_goal, y_goal, z_goal, yaw_goal, drone_pose.x, drone_pose.y, drone_pose.z, drone_pose.timestamp)
            self.flight_service.set_goal(Goal(x=x_pos, y=y_pos, z=z_pos, heading=yaw_pos))

    def hover(self, height):
        drone_pos = self.flight_service.get_latest_pose(self.flight_service._drone_object_name)
        go_to(drone_pos.x, drone_pos.y, drone_pos.z + height, drone_pos.yaw)

    def follow_object(self, object_name, offset_x, offset_z, offset_yaw):
        object_pos = self.flight_service.get_latest_pose(object_name)
        go_to(object_pos.x + offset_x, object_pos.y, object_pos.z + offset_z, object_pos.yaw + offset_yaw)

    def return_to_base(self, base_name):
        object_pos = self.flight_service.get_latest_pose(base_name)
        go_to(object_pos.x, object_pos.y, object_pos.z + 1.0, object_pos.yaw)
        go_to(object_pos.x, object_pos.y, object_pos.z + 0.003, object_pos.yaw)