
from rclpy.node import Node
from collections import namedtuple
import numpy as np
WHEEL_DIAMETER_M = 0.17
WHEELBASE_M =0.37
M_PER_REV = (WHEEL_DIAMETER_M*np.pi)
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from diff_motor_msgs.msg import MotorState
from .hoverboard_interface import hoverboard_node
class odometry_node(Node):

    def __init__(self,hoverboard):
        super().__init__('odometry_node')
        self.get_logger().info('Main node started')
        self.hoverboard = hoverboard

        self.last_time = self.get_clock().now()

        self.theta = 0
        self.x = 0
        self.y = 0

        self.odometry_timer = self.create_timer(0.01,self.update_odometry)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

    def update_odometry(self):

        #motor speed in RPM
        rspd_rpm,lspd_rpm = self.hoverboard.get_curr_speed_r_l()

        #we need velocities in m/s
        rspd = (rspd_rpm * M_PER_REV) / 60
        lspd = (lspd_rpm * M_PER_REV) / 60

        #we need velocities in m/s and rad/s
        actual_vel_t = (rspd + lspd) / 2
        actual_vel_r = (rspd - lspd) / WHEELBASE_M

        current_time = self.get_clock().now()
        
        delta_time = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        delta_theta = actual_vel_r * delta_time
        self.theta += delta_theta
    
        delta_x = actual_vel_t * delta_time * math.cos(self.theta)
        delta_y = actual_vel_t * delta_time * math.sin(self.theta)
  
        
        # Update pose
        self.x += delta_x
        self.y += delta_y

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Publish the transform
        self.publish_transform()

    def publish_transform(self):
        """Broadcast the transform from odom to base_link."""
        t = TransformStamped()

        # Fill in the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set rotation (convert theta to quaternion)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

