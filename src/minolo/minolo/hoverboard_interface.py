
import rclpy
from rclpy.node import Node
import serial
import numpy as np
from sensor_msgs.msg import BatteryState
START_SYMBOL = 0xABCD
START_BYTE=0xAB
MESSAGE_LENGTH=22
import struct
import math
from diff_motor_msgs.msg import MotorFeedback,MotorCommand
import threading
import time
import asyncio

class hoverboard_node(Node):
    def __init__(self):
        super().__init__('minolo_motor_interface')
        self.cur_speed_r = 0
        self.cur_speed_l = 0
        self.cmd_speed_r = 0
        self.cmd_speed_l =0

        self.declare_parameter('serial_port','')
        self.declare_parameter('baudrate',115200)
        self.declare_parameter('autozero_vel_timeout_seconds',0.2)
        self.declare_parameter('ticks_ceiling',9000)
        self.declare_parameter('ticks_increment_max',20)

        self.timeout_seconds = self.get_parameter("autozero_vel_timeout_seconds").get_parameter_value().double_value
        self.ticks_ceiling = self.get_parameter("ticks_ceiling").get_parameter_value().integer_value
        self.ticks_increment_max = self.get_parameter("ticks_increment_max").get_parameter_value().integer_value
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value      
        
        self._set_vels(0,0)
        self.battery_volt = 0
        self.timeout_vel = self.create_timer(self.timeout_seconds, self._clear_vel)
        ##ROS STYLE
        self.feedback_publisher = self.create_publisher(MotorFeedback,'/motor_feedback',20)
        self.command_subscribtion = self.create_subscription(MotorCommand,'/motor_command', self.update_rpms, 1)
        self.battery_status_publisher = self.create_publisher(BatteryState,'/battery_state',20)
        self.battery_timer = self.create_timer(10,self.publish_battery_status)
        self.prev_ticks_r=-1
        self.prev_ticks_l=-1   

        serial_recv_thread = threading.Thread(target=self.serial_comm_loop, daemon=True)
        serial_recv_thread.start()
        
    def serial_comm_loop(self):
        with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate) as ser:   
            motor_feedback_msg = MotorFeedback()
            
            #Loop rate is dictated by the hoverboard firmware which
            #runs at 20Hz
            while rclpy.ok():                            
                #Read 22 bytes from serial, read() is BLOCKING
                serial_packet = ser.read(22)

                if ser.in_waiting>0:
                    self.get_logger().warn(f'Serial pkt buffer with {ser.in_waiting} bytes')
            
                #Unpack data
                recv_unpacked_data = struct.unpack('<Hhhhhhhhhhh',serial_packet)            
                rx_checksum = (np.int16)(recv_unpacked_data[0]^recv_unpacked_data[1]^recv_unpacked_data[2]^recv_unpacked_data[3]^recv_unpacked_data[4]^recv_unpacked_data[5]^recv_unpacked_data[6]^recv_unpacked_data[7]^recv_unpacked_data[8]^recv_unpacked_data[9])
            
                if (rx_checksum == recv_unpacked_data[10]):        
                    motor_feedback_msg.right_ticks_delta, motor_feedback_msg.left_ticks_delta = self.get_ticks_difference_r_l(recv_unpacked_data[5],recv_unpacked_data[6])
                    motor_feedback_msg.right_ticks_act, motor_feedback_msg.left_ticks_act = recv_unpacked_data[5], recv_unpacked_data[6]
                    self.battery_volt = recv_unpacked_data[7]
                    self.feedback_publisher.publish(motor_feedback_msg)
                else:
                    ser.reset_input_buffer()    
                    self.get_logger().warn('Serial pkt sync loss. Buffer flushed.')
    
                #Write data to serial   
                cmd_speed_l=self.cmd_speed_l*2
                cmd_speed_r=self.cmd_speed_r*2
                tx_checksum = (np.int16)((cmd_speed_r) ^ (cmd_speed_l) ^ START_SYMBOL)
                serial_command = struct.pack('<Hhhh', START_SYMBOL, cmd_speed_l, cmd_speed_r, tx_checksum)
                ser.write(serial_command)

    def publish_battery_status(self):
        msg = BatteryState()      
        msg.voltage = float(self.battery_volt)
        self.battery_status_publisher.publish(msg)

    def get_ticks_difference_r_l(self,ticks_r,ticks_l):  
        if(self.prev_ticks_r==-1):
            #initializazion, run once
            diff_r = 0      
        else:
            diff_r = ticks_r - self.prev_ticks_r
            #detect large positive increment
            if diff_r > self.ticks_increment_max:
                diff_r -= (self.ticks_ceiling + 1)
            #detect large negative increment
            elif diff_r < -self.ticks_increment_max:
                diff_r += (self.ticks_ceiling + 1)
        self.prev_ticks_r = ticks_r
            
        if(self.prev_ticks_l==-1):
            #initializazion, run once
            diff_l = 0
        else:   
            diff_l = ticks_l - self.prev_ticks_l
            #detect large positive increment
            if diff_l > self.ticks_increment_max:
                diff_l -= (self.ticks_ceiling + 1)
            #detect large negative increment
            elif diff_l < -self.ticks_increment_max:
                diff_l += (self.ticks_ceiling + 1)
        self.prev_ticks_l = ticks_l       
        return diff_r,diff_l
    
    def get_battery_volt(self):
        return self.battery_volt
    
    def get_curr_speed_r_l(self):
        return self.cmd_speed_r,self.cmd_speed_l
   
    def _construct_string_message(self):     
        checksum = (np.int16)((self.cmd_speed_r*2) ^ (self.cmd_speed_l*2) ^ START_SYMBOL)
        return struct.pack('<Hhhh', START_SYMBOL, self.cmd_speed_l*2, self.cmd_speed_r*2, checksum)
    
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


def main(args=None):
    rclpy.init(args=args)
    hover = hoverboard_node()
    try:
        rclpy.spin(hover)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()