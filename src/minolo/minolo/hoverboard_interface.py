
import rclpy
from rclpy.node import Node
import serial
import numpy as np
from sensor_msgs.msg import BatteryState
START_SYMBOL = 0xABCD
import struct
import math
from diff_motor_msgs.msg import MotorState
class hoverboard_node(Node):

    def __init__(self):
        super().__init__('minolo_motor_interface')
        self.get_logger().info('Main node started')
        self.cur_speed_r = 0
        self.cur_speed_l = 0
        self.cmd_speed_r = 0
        self.cmd_speed_l =0
        
        self.declare_parameter('read_frequency',50)
        self.declare_parameter('write_frequency',200)
        self.declare_parameter('serial_port','/dev/ttyUSB1')
        self.declare_parameter('baude_rate',115200)
        self.declare_parameter('timeout_seconds',0.1)
        self.read_period = float(1/self.get_parameter("read_frequency").get_parameter_value().integer_value)
        self.write_period = float(1/self.get_parameter("write_frequency").get_parameter_value().integer_value)
        self.timeout_seconds = self.get_parameter("timeout_seconds").get_parameter_value().double_value
        self.get_logger().info('read period%.3f'%self.read_period)
        self.get_logger().info('write period %.3f'%self.write_period)
        self.get_logger().info('timeout %.4f'%self.timeout_seconds)
        self.get_logger().info('baude %d'%self.get_parameter("baude_rate").get_parameter_value().integer_value)
        self._set_vels(0,0)
        self.timer_read = self.create_timer(self.read_period, self._read_data)
        self.timer_write = self.create_timer(self.write_period, self._write_data)
        #self.ser_port  =serial.Serial(self.get_parameter("serial_port").get_parameter_value().string_value,self.get_parameter("baude_rate").get_parameter_value().integer_value) 
        self.ser_port = serial.Serial('/dev/ttyUSB1',115200)
        self.get_logger().info('port opened successfully')
        self.battery_volt = 0
        self.timeout_vel = self.create_timer(self.timeout_seconds, self._clear_vel)
        ##ROS STYLE
        self.feedback_publisher = self.create_publisher(MotorState,'/motor_feedback',10)
        self.command_subscribtion = self.create_subscription(
            MotorState,
            '/motor_command',
            self.update_rpms,
            10)
        self.battery_status_publisher = self.create_publisher(BatteryState,'/battery_state',10)
        self.battery_timer = self.create_timer(10,self.publish_battery_status)
        self.get_logger().info('Initialized Successfully')
    def publish_battery_status(self):
        
        
        msg = BatteryState()
        
        msg.voltage =float(self.battery_volt)
        
        self.battery_status_publisher.publish(msg)
        
    def _read_data(self):
        
        dev_data = struct.unpack('<Hhhhhhhhh',self.ser_port.read(18))
        msg_state = MotorState()
        msg_state.left_rpm = self.cmd_speed_l
        msg_state.right_rpm = self.cmd_speed_r
        self.feedback_publisher.publish(msg_state)
        self.battery_volt = dev_data[5]
        #self.cur_speed_r = dev_data[3]
        #self.cur_speed_l = dev_data[4]
    
    def get_battery_volt(self):
        return self.battery_volt
    def get_curr_speed_r_l(self):
        return self.cmd_speed_r,self.cmd_speed_l
    
   
    def _construct_string_message(self):  
          
        checksum = (np.int16)(self.cmd_speed_r ^ self.cmd_speed_l ^ START_SYMBOL)
        return struct.pack('<Hhhh', START_SYMBOL, self.cmd_speed_l, self.cmd_speed_r, checksum)
    
    def _set_vels(self,vel_r,vel_l):
        self.cmd_speed_r = vel_r
        self.cmd_speed_l = vel_l
    def update_rpms(self,msg):
        if(self.timeout_vel!=None):
            self.timeout_vel.cancel()
        self._set_vels(msg.right_rpm,msg.left_rpm)
        self.timeout_vel = self.create_timer(self.timeout_seconds, self._clear_vel)
    def update_vels(self, vel_r,vel_l):
        
        if(self.timeout_vel!=None):
            self.timeout_vel.cancel()
        self._set_vels(vel_r,vel_l)
        self.timeout_vel = self.create_timer(self.timeout_seconds, self._clear_vel)
    def _clear_vel(self):
        self._set_vels(0,0)
        self.timeout_vel.cancel()
    def _write_data(self):
        
        msg_byte = self._construct_string_message()
        
        self.ser_port.write(msg_byte)
    def close_port(self):
        self.ser_port.close()
def main(args=None):
    rclpy.init(args=args)

    hover = hoverboard_node()

    rclpy.spin(hover)
    hover.get_logger().info('closing port')
    hover.close_port()
    hover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()