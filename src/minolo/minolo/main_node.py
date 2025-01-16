import rclpy

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from .hoverboard_interface import hoverboard_node
from .diff_motor_controller import diff_motor_controller
from .diff_odometry import odometry_node
from .battery_monitor import battery_monitor

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.get_logger().info('Main node started')



        # Instantiate hoverboard interface (shared by all nodes)
        self.hoverboard_interface = hoverboard_node()

        # Instantiate other nodes
        self.diff_motor_node = diff_motor_controller(self.hoverboard_interface)
        self.wheel_odometry = odometry_node(self.hoverboard_interface)
        self.battery_monitor_node = battery_monitor(self.hoverboard_interface)

        # Executor to manage callbacks
        self.exec = SingleThreadedExecutor()
        print(self.exec)
        self.exec.add_node(self.hoverboard_interface)
        self.exec.add_node(self.diff_motor_node)
        self.exec.add_node(self.wheel_odometry)
        self.exec.add_node(self.battery_monitor_node)
        
    def run(self):
        try:
            self.exec.spin()
            rclpy.spin(self)
            #rclpy.spin(self.hoverboard_interface)
            #rclpy.spin(self.diff_motor_node)
            #rclpy.spin(self.wheel_odometry)
            #rclpy.spin(self.battery_monitor_node)
        finally:
            self.exec.shutdown()
            self.hoverboard_interface.close_port()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Create the main node instance
    main_node = MainNode()

    # Run the main node's executor
    main_node.run()

if __name__ == '__main__':
    main()