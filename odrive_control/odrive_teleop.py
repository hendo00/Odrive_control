import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import AxisState, ControlMode
import signal
import sys

class ODriveTeleopNode(Node):
    def __init__(self):
        super().__init__('odrive_teleop')
        self.declare_parameter('use_joy', False)

        # Initialize ODrive
        # Odrivetool --serial-number 347E36583330  << Later for multiple odrvs

        self.odrv = odrive.find_any()
        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL

        # Set torque limits
        self.max_torque = 2.0
        self.min_torque = -2.0

        # Subscribers
        self.use_joy = self.get_parameter('use_joy').get_parameter_value().bool_value
        if self.use_joy:
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        else:
            self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg: Twist):
        self.get_logger().info(f"Received Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        torque = msg.linear.x
        torque = max(min(torque, self.max_torque), self.min_torque)
        self.odrv.axis0.controller.input_torque = torque
        self.get_logger().info(f"Set torque to: {torque}")


    def joy_callback(self, msg: Joy):
        # Use joystick axes for torque
        torque = msg.axes[1] * self.max_torque  # Assuming joystick Y-axis controls torque
        torque = max(min(torque, self.max_torque), self.min_torque)
        self.odrv.axis0.controller.input_torque = torque
        self.get_logger().info(f'Set torque to: {torque}')
    
    def cleanup_odrive(self):
        self.odrv.axis0.requested_state = AxisState.IDLE
        self.get_logger().info("ODrive set to IDLE. Exiting...")

def main(args=None):
    rclpy.init(args=args)
    node = ODriveTeleopNode()

    # Define the signal handler
    def signal_handler(sig, frame):
        if rclpy.ok():
            node.get_logger().info("Received shutdown signal.")
            node.cleanup_odrive()
            node.destroy_node()
            rclpy.shutdown()
        sys.exit(0)

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Create an executor with a timeout
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # Periodically check for signals
    except KeyboardInterrupt:
        pass  # Let the signal handler manage cleanup
    finally:
        if rclpy.ok():
            node.get_logger().info("Shutting down ROS...")
            node.cleanup_odrive()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
