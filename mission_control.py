import logging
import time
import threading
import keyboard

from crazyflie_client import CrazyflieClient
from flight_control import Goal
from flight_service import FlightService
from vicon_motion import ViconMotionClient

TWO_DRONES = True
DRONE_OBJECT = '2026_Drone1'
GROUND_OBJECT = '2026_Holder_1'
BASESTATION = '2026_BaseStation_1'

def init_mission(flight_service_1, flight_service_2):
    if (flight_service_2 == None):
        TWO_DRONES = False

def time_control(x_goal, y_goal, z_goal, yaw_goal, x_start, y_start, z_start, start_time): 
    #TODO further expand to allow user controlled velocity, step x and y
    drone_pos = flight_service_1.get_latest_pose(DRONE_OBJECT)
    z_real = z_start + 1/(5*(z_goal - z_start))*math.pow((drone_pos.timestamp-start_time), 2)
    
    if (abs(drone_pos.x - x_goal) < 0.005 & abs(drone_pos.y - y_goal) < 0.005 & abs(drone_pos.z - z_goal) < 0.005 & abs(drone_pos.yaw - yaw_goal) < 0.005):
        return x_goal, y_goal, z_goal, yaw_goal, True
    return x_goal, y_goal, z_real, yaw_goal, False

def go_to(x_goal, y_goal, z_goal, yaw_goal): #right now this blocks until goal is reached
    drone_pos = flight_service_1.get_latest_pose(DRONE_OBJECT)
    goal_reached = False
    while not goal_reached
        x_pos, y_pos, z_pos, yaw_pos, goal_reached = time_control(x_goal, y_goal, z_goal, yaw_goal, drone_pose.x, drone_pose.y, drone_pose.z, drone_pose.timestamp)
        flight_service_1.set_goal(Goal(x=x_pos, y=y_pos, z=z_pos, heading=yaw_pos))

def hover():

def find_box():

def return_to_base():