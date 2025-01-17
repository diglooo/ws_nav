import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from nav_msgs.msg import Odometry
from diff_motor_msgs.msg import MotorState

class diff_motor_controller(Node):

    def __init__(self):
        super().__init__('minolo_motor_controller')
        self.get_logger().info('Motor Controller started')
        self.declare_parameter('wheel_diameter',0.0)
        self.declare_parameter('wheels_base',0.0)
        self.declare_parameter('publish_tf',False)
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.get_logger().info('.%4f'%self.wheel_diameter)
        self.wheels_base = self.get_parameter('wheels_base').get_parameter_value().double_value        
        self.get_logger().info('.%4f'%self.wheels_base)
        self.wheel_circumference = np.pi*self.wheel_diameter
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.transform_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.relay_vel,
            10)
        
        self.command_publisher = self.create_publisher(MotorState,'/motor_command',10)

        self.last_time = self.get_clock().now()

        self.theta = 0
        self.x = 0
        self.y = 0

        #self.odometry_timer = self.create_timer(0.01,self.update_odometry)
        self.odometry_subscriber = self.create_subscription(MotorState,'/motor_feedback',self.update_odometry,10)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.odom_publisher = self.create_publisher(Odometry,'/wheel_odom',10)

    def get_diff_vel(self,t,r):  
        rSpd = (np.int16)(((t.x + (r.z * self.wheels_base * 0.5))) / self.wheel_circumference *60)
        lSpd = (np.int16)(((t.x - (r.z * self.wheels_base * 0.5))) / self.wheel_circumference *60)
        rSpd = np.clip(rSpd,-60,60)
        lSpd = np.clip(lSpd,-60,60)        
        return rSpd,lSpd
    

    def relay_vel(self, msg):

        vel_right,vel_left = self.get_diff_vel(msg.linear,msg.angular)
        msg_cmd = MotorState()
        msg_cmd.left_rpm = (int)(vel_left)
        msg_cmd.right_rpm = (int)(vel_right)
        self.command_publisher.publish(msg_cmd)
        #self.hoverboard.update_vels(vel_right,vel_left)

    def update_odometry(self,feedback_msg):

        #motor speed in RPM
        
        rspd_rpm,lspd_rpm = feedback_msg.right_rpm,feedback_msg.left_rpm #self.hoverboard.get_curr_speed_r_l()

        #we need velocities in m/s
        rspd = (rspd_rpm * self.wheel_circumference) / 60
        lspd = (lspd_rpm * self.wheel_circumference) / 60

        #we need velocities in m/s and rad/s
        self.actual_vel_t = (rspd + lspd) / 2
        self.actual_vel_r = (rspd - lspd) / self.wheels_base

        current_time = self.get_clock().now()
        
        delta_time = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        delta_theta = self.actual_vel_r * delta_time
        self.theta += delta_theta
    
        delta_x = self.actual_vel_t * delta_time * math.cos(self.theta)
        delta_y = self.actual_vel_t * delta_time * math.sin(self.theta)
  
        
        # Update pose
        self.x += delta_x
        self.y += delta_y

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Publish the transform
        self.publish_transform()

    def publish_transform(self):
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        if(self.publish_tf):
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
            
            

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quat
        #set the velocity
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.twist.twist.linear.x = self.actual_vel_t  
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.actual_vel_r
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)

    motor = diff_motor_controller()

    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()