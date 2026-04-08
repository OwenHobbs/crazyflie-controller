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

def time_control(x_goal, y_goal, z_goal): 

def hover():

def find_box():

def return_to_base():