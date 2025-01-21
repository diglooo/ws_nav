
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

class hoverboard_node(Node):

    def __init__(self):
        super().__init__('minolo_motor_interface')
        self.get_logger().info('Main node started')
        self.cur_speed_r = 0
        self.cur_speed_l = 0
        self.cmd_speed_r = 0
        self.cmd_speed_l =0
        
        self.declare_parameter('read_frequency',20)
        self.declare_parameter('write_frequency',20)
        self.declare_parameter('serial_port','')
        self.declare_parameter('baudrate',115200)
        self.declare_parameter('timeout_seconds',0.2)
        self.declare_parameter('ticks_ceiling',9000)
        self.declare_parameter('ticks_increment_max',20)

        self.read_period = float(1/self.get_parameter("read_frequency").get_parameter_value().integer_value)
        self.write_period = float(1/self.get_parameter("write_frequency").get_parameter_value().integer_value)
        self.timeout_seconds = self.get_parameter("timeout_seconds").get_parameter_value().double_value
        self.ticks_ceiling = self.get_parameter("ticks_ceiling").get_parameter_value().integer_value
        self.ticks_increment_max = self.get_parameter("ticks_increment_max").get_parameter_value().integer_value
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.get_logger().info('read period%.3f'%self.read_period)
        self.get_logger().info('write period %.3f'%self.write_period)
        self.get_logger().info('timeout %.4f'%self.timeout_seconds)
        self.get_logger().info('baude %d'%self.get_parameter("baudrate").get_parameter_value().integer_value)
        
        self._set_vels(0,0)
    
        #self.timer_read = self.create_timer(self.read_period, self._read_data)
        #self.timer_write = self.create_timer(self.write_period, self._write_data)
        #self.timer_write = self.create_timer(0.05, self._serial_comm)
        #self.ser_port = serial.Serial(self.serial_port_name,115200)
        #self.get_logger().info('port opened successfully')
        self.battery_volt = 0
        self.timeout_vel = self.create_timer(self.timeout_seconds, self._clear_vel)
        ##ROS STYLE
        self.feedback_publisher = self.create_publisher(MotorFeedback,'/motor_feedback',10)
        self.command_subscribtion = self.create_subscription(
            MotorCommand,
            '/motor_command',
            self.update_rpms,
            10)
        self.battery_status_publisher = self.create_publisher(BatteryState,'/battery_state',10)
        self.battery_timer = self.create_timer(10,self.publish_battery_status)
        self.get_logger().info('Initialized Successfully')
        self.prev_ticks_r=-1
        self.prev_ticks_l=-1

        serial_recv_thread = threading.Thread(target=self.serial_recv, daemon=True)
        serial_recv_thread.start()
        
    def serial_recv(self):
        try:
            # Open the serial port
            with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate, timeout=0.5) as ser:   
                self.get_logger().info(f'{self.serial_port_name} port opened successfully')         
                while True:
                    cmd_speed_l=self.cmd_speed_l*2
                    cmd_speed_r=self.cmd_speed_r*2
                    recv_unpacked_data = struct.unpack('<Hhhhhhhhhhh',ser.read(22))                  
                    rx_checksum = (np.int16)(recv_unpacked_data[0]^recv_unpacked_data[1]^recv_unpacked_data[2]^recv_unpacked_data[3]^recv_unpacked_data[4]^recv_unpacked_data[5]^recv_unpacked_data[6]^recv_unpacked_data[7]^recv_unpacked_data[8]^recv_unpacked_data[9])
                    msg_state = MotorFeedback()

                    if (rx_checksum == recv_unpacked_data[10]):       
                        msg_state = MotorFeedback()
                        msg_state.right_ticks_delta, msg_state.left_ticks_delta = self.get_ticks_difference_r_l(recv_unpacked_data[5],recv_unpacked_data[6])
                        msg_state.right_ticks_act, msg_state.left_ticks_act = recv_unpacked_data[5], recv_unpacked_data[6]
                        #self.get_logger().info('%d, %d'%(dev_data[6],dev_data[5]))
                        #print(msg_state)
                        self.battery_volt = recv_unpacked_data[7]
                        self.feedback_publisher.publish(msg_state)
                    else:
                        ser.reset_input_buffer()    
                    
                    tx_checksum = (np.int16)((cmd_speed_r) ^ (cmd_speed_l) ^ START_SYMBOL)
                    serial_command = struct.pack('<Hhhh', START_SYMBOL, cmd_speed_l, cmd_speed_r, tx_checksum)
                    ser.write(serial_command)
                    time.sleep(0.05)  

        except serial.SerialException as e:
            return None

    def publish_battery_status(self):
        msg = BatteryState()      
        msg.voltage =float(self.battery_volt)
        self.battery_status_publisher.publish(msg)

    def get_ticks_difference_r_l(self,ticks_r,ticks_l):
        if(self.prev_ticks_r==-1):
            diff_r = 0
            
        else:
            diff_r = ticks_r - self.prev_ticks_r
            if diff_r > self.ticks_ceiling:
                diff_r -= (self.ticks_ceiling + 1)

            # Handle negative rollover (from 0 to MAX_TICKS)
            elif diff_r < -self.ticks_increment_max:
                diff_r += (self.ticks_ceiling + 1)

        self.prev_ticks_r = ticks_r
        if(self.prev_ticks_l==-1):
            diff_l = 0
            
        diff_l = ticks_l - self.prev_ticks_l
        if diff_l > self.ticks_ceiling:
            diff_l -= (self.ticks_ceiling + 1)

        # Handle negative rollover (from 0 to MAX_TICKS)
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