import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

import sys
import select
import tty
import termios
import threading
import time

# 按键映射
key_map = {
    'w': "FORWARD", 's': "BACKWARD",
    'a': "LEFT", 'd': "RIGHT",
    'i': "UP", 'k': "DOWN",
    'j': "YAW_LEFT", 'l': "YAW_RIGHT",
    't': "ARM", 'o': "DISARM",
    'y': "OFFBOARD",
    ' ': "STOP"
}

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_status = VehicleStatus()
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_speed = 0.0
        
        # --- 新增：用于超时机制的时间戳 ---
        self.last_key_press_time = self.get_clock().now()

        self.get_logger().info("Keyboard Teleop Node started.")
        self.print_instructions()

        tty.setcbreak(sys.stdin.fileno())
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def destroy_node(self):
        self.get_logger().info("Restoring terminal settings...")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

    def keyboard_listener(self):
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                self.process_key(key.lower())

    def process_key(self, key):
        # --- 改进：每次按键都更新时间戳 ---
        self.last_key_press_time = self.get_clock().now()
        
        command = key_map.get(key)
        linear_speed = 2.0
        angular_speed = 0.5

        if command == "FORWARD": self.vx = linear_speed
        elif command == "BACKWARD": self.vx = -linear_speed
        elif command == "LEFT": self.vy = -linear_speed
        elif command == "RIGHT": self.vy = linear_speed
        elif command == "UP": self.vz = -linear_speed
        elif command == "DOWN": self.vz = linear_speed
        elif command == "YAW_LEFT": self.yaw_speed = -angular_speed
        elif command == "YAW_RIGHT": self.yaw_speed = angular_speed
        elif command == "ARM": self.arm()
        elif command == "DISARM": self.disarm()
        elif command == "OFFBOARD": self.engage_offboard_mode()
        elif command == "STOP":
            self.vx = self.vy = self.vz = self.yaw_speed = 0.0

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info("Requesting Offboard mode...")
        else:
            self.get_logger().warn("Cannot engage Offboard mode: Vehicle is not armed!")
            
    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        # --- 改进：在发送指令前，检查是否超时 ---
        now = self.get_clock().now()
        # 如果距离上次按键超过 0.2 秒，则自动悬停
        if (now - self.last_key_press_time).nanoseconds > 200_000_000: # 0.2 seconds
            self.vx = self.vy = self.vz = self.yaw_speed = 0.0

        # 后续的发布逻辑保持不变
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        # ... (rest of offboard_msg) ...
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_msg)

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [float('nan'), float('nan'), float('nan')]
        trajectory_msg.velocity = [self.vx, self.vy, self.vz]
        trajectory_msg.yawspeed = self.yaw_speed
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(trajectory_msg)

    def print_instructions(self):
        print("-----------------------------------")
        print("      Keyboard Control for PX4 (V6: Responsive Feel)     ")
        print("-----------------------------------")
        print("  HOLD w/s: Forward/Backward")
        print("  HOLD a/d: Left/Right")
        print("  HOLD i/k: UP/DOWN")
        print("  HOLD j/l: Yaw Left/Right")
        print("  space: Emergency STOP (Hover)")
        print("  RELEASE any movement key to HOVER")
        print("-----------------------------------")
        print("  FLIGHT PROCEDURE:")
        print("  1. Press 't' to ARM")
        print("  2. Press 'y' to ENGAGE OFFBOARD MODE")
        print("  3. Use movement keys to fly")
        print("  4. Press 'o' to DISARM (will land first)")
        print("-----------------------------------")
        print("  CLICK ON THIS TERMINAL TO ACTIVATE KEYS")
        print("-----------------------------------")


def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop_node = KeyboardTeleopNode()
    try:
        rclpy.spin(keyboard_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()