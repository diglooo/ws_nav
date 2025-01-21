import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

class TeleopReceiver(Node):
    def __init__(self):
        super().__init__('teleop_receiver')
        self.declare_parameter('serial_port','')
        self.declare_parameter('baudrate',115200)
        self.declare_parameter('velocity_topic','/cmd_vel')
        
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
    
        self.cmd_vel_msg = Twist()
        self.cmd_vel_publisher = self.create_publisher(Twist,self.velocity_topic,10)
        
        serial_recv_thread = threading.Thread(target=self.serial_recv, daemon=True)
        serial_recv_thread.start()
        
        
    def _parse_serial_data(self, serial_data):
        try:
            parts = serial_data.strip().split(',')   
            if len(parts) == 3 and parts[0] == "START":
                command = parts[0]
                value1 = int(parts[1])
                value2 = int(parts[2])
                return [command, value1, value2]
            else:
                raise ValueError("Invalid data format")
        except Exception as e:
            return None
    
    def serial_recv(self):
        try:
            # Open the serial port
            with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate, timeout=0.5) as ser:   
                self.get_logger().info(f'{self.serial_port_name} port opened successfully')         
                while True:
                    # Read a line from the serial port
                    if ser.in_waiting > 0:
                        serial_data = ser.readline().decode('utf-8')
                        parsed_data = self._parse_serial_data(serial_data)
                       
                        if parsed_data:
                            x_vel = -(parsed_data[2] / 2500)*0.5;
                            rz_vel = -(parsed_data[1] / 2500)*1.0;
                            self.cmd_vel_msg.linear.x = x_vel  
                            self.cmd_vel_msg.angular.z = rz_vel 
                            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                    time.sleep(0.001)                     
        except serial.SerialException as e:
            return None

def main(args=None):
    rclpy.init(args=args)

    teleop_receiver = TeleopReceiver()
    rclpy.spin(teleop_receiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_receiver.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()