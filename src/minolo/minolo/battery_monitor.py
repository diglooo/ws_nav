import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32,String
from geometry_msgs.msg import Twist
import serial
from collections import namedtuple
import pickle
import numpy as np
WHEEL_DIAMETER_M = 0.17
WHEELBASE_M =0.37
M_PER_REV = (WHEEL_DIAMETER_M*np.pi)
START_SYMBOL = 0xABCD
import struct
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from .hoverboard_interface import hoverboard_node
class battery_monitor(Node):

    def __init__(self,hoverboard):
        super().__init__('battery_monitor')
        self.get_logger().info('Main node started')
        self.hoverboard = hoverboard
        self.battery_status_publisher = self.create_publisher(BatteryState,'/battery_state',10)
        self.battery_timer = self.create_timer(10,self.read_battery)


    def read_battery(self):
        volt_int = self.hoverboard.get_battery_volt()
        
        msg = BatteryState()
        
        msg.voltage =float(volt_int)
        
        self.battery_status_publisher.publish(msg)

        
