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

class VelocityWriter(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.set_vels(0,0)
        timer_period = 0.01
        self.last_time = self.get_clock().now()
        self.timeout=None
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser_port  =serial.Serial('/dev/ttyUSB1',115200) 
        self.theta = 0
        self.x = 0
        self.y = 0
        self.battery_status_publisher = self.create_publisher(BatteryState,'/battery_volt',10)
        self.battery_timer = self.create_timer(5,self.read_battery)
        self.odometry_timer = self.create_timer(0.05,self.update_odometry)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

    def read_battery(self):
        volt_int = struct.unpack('<Hhhhhhhhh',self.ser_port.read(18))[5]
        
        msg = BatteryState()
        
        msg.voltage =float(volt_int)
        self.battery_status_publisher.publish(msg)

    def update_odometry(self):
        feedback_array = struct.unpack('<Hhhhhhhhh',self.ser_port.read(18))
        #print(feedback_array)
        rspd = feedback_array[3]
        lspd = feedback_array[4]
        #print(rspd,lspd)
        actual_vel_t = (rspd+lspd)/2
        actual_vel_r = (M_PER_REV *60*(rspd-lspd)) /(2*WHEELBASE_M*0.5)

        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        #delta_x = actual_vel_t * delta_time * math.cos(self.theta)
        #delta_y = actual_vel_t * delta_time * math.sin(self.theta)
        delta_theta = self.vel_rz * delta_time
        self.theta += delta_theta
        
        delta_x = self.vel_tx * delta_time * math.cos(self.theta)
        delta_y = self.vel_tx * delta_time * math.sin(self.theta)  

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

    def get_string_message(self):  
        rSpd = (np.int16)(((self.vel_tx + (self.vel_rz * WHEELBASE_M * 0.5))) / M_PER_REV *60)
        lSpd = (np.int16)(((self.vel_tx - (self.vel_rz * WHEELBASE_M * 0.5))) / M_PER_REV *60)
        rSpd = np.clip(rSpd,-60,60)
        lSpd = np.clip(lSpd,-60,60)        
        checksum = (np.int16)(rSpd ^ lSpd ^ START_SYMBOL)
        return struct.pack('<Hhhh', START_SYMBOL, lSpd, rSpd, checksum)
    
    def set_vels(self,vel_t,vel_r):
        self.vel_tx = vel_t
        self.vel_rz = vel_r
        #print('New speeds: Tx: %.2f Rz: %.2f'%(vel_t,vel_r))
    def listener_callback(self, msg):
        #print('msg received')
        if(self.timeout!=None):
            self.timeout.cancel()
        self.set_vels(msg.linear.x,msg.angular.z)
        self.timeout = self.create_timer(0.02, self.clear_vel)
    def clear_vel(self):
        self.set_vels(0,0)
        self.timeout.cancel()
    def timer_callback(self):
        msg_byte = self.get_string_message()
        
        self.ser_port.write(msg_byte)
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = VelocityWriter()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.ser_port.close()
    minimal_subscriber.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()