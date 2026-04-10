# import logging
# import time
# import threading
# import keyboard
# import math

# from crazyflie_client import CrazyflieClient
# from flight_control import Goal
# from flight_service import FlightService
# from vicon_motion import ViconMotionClient

# TWO_DRONES = True
# DRONE_OBJECT = '2026_Drone1'
# GROUND_OBJECT = '2026_Holder_1'
# BASESTATION = '2026_BaseStation_1'

# def init_mission(flight_service_1, flight_service_2):
#     if (flight_service_2 == None):
#         TWO_DRONES = False

# def time_control(x_goal, y_goal, z_goal): 

# def hover():

# def find_box():

# def return_to_base():

# def land():

# def goto():

    
# #in my mind this function takes in which drone is the follower, which is the leader, and the offset from the leader to maintain.
# # It then continuously updates the follower's goal position based on the leader's current position plus the offset.
# def offset(follower, leader, offset):
#     while True:
#         leader_position = leader.get_latest_position()  

#         offset_x = offset * math.sin(math.radians(leader_position.heading)) * -1
#         offset_y = offset * math.cos(math.radians(leader_position.heading)) * -1

#         Goal = Goal(x=leader_position.x + offset_x, y=leader_position.y + offset_y, z=leader_position.z, heading=leader_position.heading)
#         follower.set_goal(Goal)