import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import AxisState, ControlMode
import signal
import sys
import time

class ODriveTeleopNode(Node):
    def __init__(self):
        super().__init__('switching_control_teleop')
        self.declare_parameter('use_joy', False)

        # Initialize ODrive
        self.odrv = odrive.find_any()
        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL

        # Set torque limits
        self.max_torque = 2.0
        self.min_torque = -2.0

        # Control mode variables
        self.control_modes = [ControlMode.TORQUE_CONTROL, ControlMode.VELOCITY_CONTROL, ControlMode.POSITION_CONTROL]
        self.current_mode_index = 0  # Default to Torque Control
        self.mode_confirmed = True  # Initially confirmed

        # Input hold flag
        self.hold_input = False
        self.held_value = 0

        # Debounce state
        self.last_button_time = {}  # Store last press time for buttons
        self.debounce_duration = 0.2  # Debounce duration in seconds

        # Subscribers
        self.use_joy = self.get_parameter('use_joy').get_parameter_value().bool_value
        if self.use_joy:
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        else:
            self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg: Twist):
        if self.mode_confirmed:
            if self.control_modes[self.current_mode_index] == ControlMode.TORQUE_CONTROL:
                torque = self.held_value if self.hold_input else msg.linear.x
                torque = max(min(torque, self.max_torque), self.min_torque)
                self.odrv.axis0.controller.input_torque = torque
                self.get_logger().info(f"Set torque to: {torque}")

    def joy_callback(self, msg: Joy):
        current_time = time.time()

        # Button 0: Hold input
        if self.is_button_pressed(msg.buttons[0], 0, current_time):
            self.hold_input = True
            self.held_value = msg.axes[1] * (self.max_torque if self.control_modes[self.current_mode_index] == ControlMode.TORQUE_CONTROL else 10)
            self.get_logger().info("Holding current input.")

        # Button 1: Release hold
        if self.is_button_pressed(msg.buttons[1], 1, current_time):
            self.hold_input = False
            self.get_logger().info("Released input hold.")

        # Button 3: Switch control modes (cycle freely)
        if self.is_button_pressed(msg.buttons[3], 3, current_time):
            self.current_mode_index = (self.current_mode_index + 1) % len(self.control_modes)
            mode_name = self.get_mode_name(self.control_modes[self.current_mode_index])
            self.get_logger().info(f"Switched to mode: {mode_name}")

        # Button 4: Confirm mode and lock control
        if self.is_button_pressed(msg.buttons[4], 4, current_time):
            self.mode_confirmed = True
            self.odrv.axis0.controller.config.control_mode = self.control_modes[self.current_mode_index]
            mode_name = self.get_mode_name(self.control_modes[self.current_mode_index])
            self.get_logger().info(f"Confirmed mode: {mode_name}. Ready for input.")

        # Handle joystick axis input based on the confirmed mode
        if self.mode_confirmed and not self.hold_input:
            if self.control_modes[self.current_mode_index] == ControlMode.TORQUE_CONTROL:
                torque = msg.axes[1] * self.max_torque
                torque = max(min(torque, self.max_torque), self.min_torque)
                self.odrv.axis0.controller.input_torque = torque
                self.get_logger().info(f'Set torque to: {torque}')

            elif self.control_modes[self.current_mode_index] == ControlMode.VELOCITY_CONTROL:
                velocity = msg.axes[1] * 10  # Scale velocity as needed
                self.odrv.axis0.controller.input_vel = velocity
                self.get_logger().info(f'Set velocity to: {velocity}')

            elif self.control_modes[self.current_mode_index] == ControlMode.POSITION_CONTROL:
                position = msg.axes[1] * 10  # Scale position as needed
                self.odrv.axis0.controller.input_pos = position
                self.get_logger().info(f'Set position to: {position}')

    def is_button_pressed(self, button_state, button_index, current_time):
        if button_state == 1:
            last_time = self.last_button_time.get(button_index, 0)
            if current_time - last_time >= self.debounce_duration:
                self.last_button_time[button_index] = current_time
                return True
        return False

    def get_mode_name(self, mode):
        if mode == ControlMode.TORQUE_CONTROL:
            return "Torque Control"
        elif mode == ControlMode.VELOCITY_CONTROL:
            return "Velocity Control"
        elif mode == ControlMode.POSITION_CONTROL:
            return "Position Control"
        return "Unknown"

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
