
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
WHEEL_DIAMETER_M = 0.17
WHEELBASE_M =0.37
M_PER_REV = (WHEEL_DIAMETER_M*np.pi)
START_SYMBOL = 0xABCD
import math
from diff_motor_msgs.msg import MotorState
from .hoverboard_interface import hoverboard_node
class diff_motor_controller(Node):

    def __init__(self,hoverboard):
        super().__init__('minimal_subscriber')
        self.get_logger().info('Main node started')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.relay_vel,
            10)
        self.hoverboard = hoverboard
        
    

    def get_diff_vel(self,t,r):  
        rSpd = (np.int16)(((t.x + (r.z * WHEELBASE_M * 0.5))) / M_PER_REV *60)
        lSpd = (np.int16)(((t.x - (r.z * WHEELBASE_M * 0.5))) / M_PER_REV *60)
        rSpd = np.clip(rSpd,-60,60)
        lSpd = np.clip(lSpd,-60,60)        
        return rSpd,lSpd
    

    def relay_vel(self, msg):

        vel_right,vel_left = self.get_diff_vel(msg.linear,msg.angular)
        self.hoverboard.update_vels(vel_right,vel_left)

        
