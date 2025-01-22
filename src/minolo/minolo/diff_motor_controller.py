import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from nav_msgs.msg import Odometry
from diff_motor_msgs.msg import MotorFeedback,MotorCommand

class diff_motor_controller(Node):

    def __init__(self):
        super().__init__('minolo_motor_controller')
        self.get_logger().info('Motor Controller started')
        self.declare_parameter('wheel_diameter',0.17)
        self.declare_parameter('wheels_base',0.037)
        self.declare_parameter('round_ticks',90)  
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')      
        self.declare_parameter('publish_tf',True)
        
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value 
             
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.get_logger().info('.%4f'%self.wheel_diameter)
        self.wheels_base = self.get_parameter('wheels_base').get_parameter_value().double_value        
        self.round_ticks = self.get_parameter('round_ticks').get_parameter_value().integer_value   
        self.get_logger().info('.%4f'%self.wheels_base)
        self.wheel_circumference = np.pi*self.wheel_diameter
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        self.command_publisher = self.create_publisher(MotorCommand,'/motor_command',5)
        self.odom_publisher = self.create_publisher(Odometry,'/odom',5)

        self.transform_subscriber = self.create_subscription(Twist,'cmd_vel', self.relay_vel, 5)
        self.odometry_subscriber = self.create_subscription(MotorFeedback,'/motor_feedback',self.update_odometry,5)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.theta = 0
        self.x = 0
        self.y = 0
        self.cur_ticks_right= -1
        self.cur_ticks_left = -1
        #self.odometry_timer = self.create_timer(0.01,self.update_odometry)     


    def get_diff_vel(self,t,r):  
        rSpd = (np.int16)(((t.x + (r.z * self.wheels_base * 0.5))) / self.wheel_circumference *60)
        lSpd = (np.int16)(((t.x - (r.z * self.wheels_base * 0.5))) / self.wheel_circumference *60)
        rSpd = np.clip(rSpd,-30,30)
        lSpd = np.clip(lSpd,-30,30)        
        return rSpd,lSpd 

    def relay_vel(self, msg):
        vel_right,vel_left = self.get_diff_vel(msg.linear,msg.angular)
        msg_cmd = MotorCommand()
        msg_cmd.left_rpm = (int)(vel_left)
        msg_cmd.right_rpm = (int)(vel_right)
        #print(msg_cmd)
        self.command_publisher.publish(msg_cmd)
        #self.hoverboard.update_vels(vel_right,vel_left)

    def update_odometry(self,feedback_msg):
        r_ticks,l_ticks = feedback_msg.right_ticks_delta,feedback_msg.left_ticks_delta #self.hoverboard.get_curr_speed_r_l()
        r_dist = (self.wheel_circumference * r_ticks)/self.round_ticks
        l_dist = (self.wheel_circumference * l_ticks)/self.round_ticks
        #print(r_dist,l_dist)
        dist_linear = (r_dist+l_dist)/2
        self.delta_rotational = (r_dist-l_dist)/self.wheels_base
        #we need velocities in m/s and rad/s
          
        # Update pose
        self.delta_x = dist_linear * math.cos((self.delta_rotational/2) + self.theta)
        self.x += self.delta_x
        self.delta_y = dist_linear * math.sin((self.delta_rotational/2) + self.theta)
        self.y += self.delta_y
        self.theta += self.delta_rotational

        # Normalize theta to [-pi, pi]
        # Publish the transform
        self.publish_transform()

    def publish_transform(self):
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        q_delta = tf_transformations.quaternion_from_euler(0, 0, self.delta_rotational)

        """Broadcast the transform from odom to base_link."""
        t = TransformStamped()
        # Fill in the header
        
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set rotation (convert theta to quaternion)    
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        """Broadcast Odometry message to /odom topic"""
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        odom_msg = Odometry()
        
        odom_msg.header.frame_id = self.frame_id   
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quat

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = self.get_clock().now().to_msg()
        self.odom_publisher.publish(odom_msg)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    motor = diff_motor_controller()

    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()