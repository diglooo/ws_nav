import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time
import threading
import math
import csv

class ImuReceiver(Node):
    def __init__(self):
        super().__init__('imu_receiver')
        self.declare_parameter('serial_port','/dev/ttyACM0')
        self.declare_parameter('baudrate',115200)
        self.declare_parameter('output_topic','/imu_data')
        self.declare_parameter('accel_sensitivity',2.0)
        self.declare_parameter('gyro_sensitivity',500.0)
        self.declare_parameter('frame_id','imu_link')
        
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.accel_sensitivity = self.get_parameter('accel_sensitivity').get_parameter_value().double_value
        self.gyro_sensitivity = self.get_parameter('gyro_sensitivity').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.cmd_vel_publisher = self.create_publisher(Imu,self.output_topic,10)       
        
    def _parse_serial_data(self, serial_data):
        try:
            parts = serial_data.strip().split(',')   
            if len(parts) == 6:
                return [int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])]
            else:   
                raise ValueError("Invalid data format")
        except Exception as e:
            return None
        
    def _scale_ac(self, value):
        return ((value * self.accel_sensitivity) / 32768.0) * 9.81
    
    def _scale_gy(self, value):
        return ((value * self.gyro_sensitivity) / 32768.0) * (math.pi / 180.0)

    def serial_comm_loop(self):
        message = Imu()
        #self.file = open("imu_data.csv", mode="w", newline="")
        #csvwriter = csv.writer(self.file) 
        try:
            # Open the serial port
            with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate, timeout=0.5) as ser:   
                self.get_logger().info(f'{self.serial_port_name} port opened successfully')         
                while rclpy.ok():
                    # Read a line from the serial port
                    serial_data = ser.readline().decode('utf-8')
                    parsed_data = self._parse_serial_data(serial_data)
                                            
                    if parsed_data:           
                        message.header.frame_id = self.frame_id
                        message.header.stamp = self.get_clock().now().to_msg()               
                        [ax, ay, az, gx, gy, gz] = parsed_data
                        
                        #csvwriter.writerow([self._scale_ac(ax), self._scale_ac(ay), self._scale_ac(az), self._scale_gy(gx), self._scale_gy(gy), self._scale_gy(gz)])
                        message.linear_acceleration.x=-self._scale_ac(ax)
                        message.linear_acceleration.y= - self._scale_ac(ay)
                        message.linear_acceleration.z=  self._scale_ac(az)
                        message.angular_velocity.x=self._scale_gy(gx)
                        message.angular_velocity.y= - self._scale_gy(gy)
                        message.angular_velocity.z=self._scale_gy(gz)                 
                        self.cmd_vel_publisher.publish(message)                         
        except serial.SerialException as e:
            self.get_logger().error(e)
            return None

def main(args=None):
    rclpy.init(args=args)

    imu_recv = ImuReceiver()
    try:
        imu_recv.serial_comm_loop()
    except KeyboardInterrupt:
        pass  


if __name__ == '__main__':
    main()