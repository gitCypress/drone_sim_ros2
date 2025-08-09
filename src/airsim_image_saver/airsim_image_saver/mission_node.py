import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

import math
from enum import Enum

class MissionState(Enum):
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER = 3
    CIRCLING = 4
    RETURNING = 5
    LANDING = 6
    DISARMING = 7

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # Mission Parameters
        self.takeoff_height = -5.0  # meters (NED frame, so negative is up)
        self.circle_radius = 10.0   # meters
        self.circle_speed = 2.0     # m/s
        self.hover_duration = 3.0   # seconds

        # State machine variables
        self.state = MissionState.IDLE
        self.hover_start_time = 0
        self.circle_start_time = 0

        # Vehicle state variables
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)

        # Main timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Mission node started. Waiting for first position data...")

    def local_position_callback(self, msg):
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        if self.state == MissionState.IDLE and self.local_position.z != 0.0:
            self.get_logger().info("Position data received. Starting mission.")
            self.state = MissionState.ARMING


    def timer_callback(self):
        """Main state machine logic."""
        self.publish_offboard_control_mode()

        if self.state == MissionState.ARMING:
            self.arm()
            self.engage_offboard_mode()
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Vehicle armed. Transitioning to TAKEOFF.")
                self.state = MissionState.TAKEOFF

        elif self.state == MissionState.TAKEOFF:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            if abs(self.local_position.z - self.takeoff_height) < 0.5:
                self.get_logger().info("Takeoff complete. Transitioning to HOVER.")
                self.state = MissionState.HOVER
                self.hover_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        elif self.state == MissionState.HOVER:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            if self.get_clock().now().seconds_nanoseconds()[0] - self.hover_start_time > self.hover_duration:
                self.get_logger().info("Hover complete. Transitioning to CIRCLING.")
                self.state = MissionState.CIRCLING
                self.circle_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        elif self.state == MissionState.CIRCLING:
            elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.circle_start_time
            angular_velocity = self.circle_speed / self.circle_radius
            angle = angular_velocity * elapsed_time
            
            x = self.circle_radius * (math.cos(angle) - 1.0)
            y = self.circle_radius * math.sin(angle)
            
            self.publish_position_setpoint(x, y, self.takeoff_height)

            if angle >= 2 * math.pi:
                self.get_logger().info("Circle complete. Transitioning to RETURNING.")
                self.state = MissionState.RETURNING

        elif self.state == MissionState.RETURNING:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            distance_to_center = math.sqrt(self.local_position.x**2 + self.local_position.y**2)
            if distance_to_center < 0.5:
                self.get_logger().info("Returned to center. Transitioning to LANDING.")
                self.state = MissionState.LANDING
        
        elif self.state == MissionState.LANDING:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                 if self.vehicle_status.landed_state == VehicleStatus.LANDED_STATE_ON_GROUND:
                    self.get_logger().info("Landed. Transitioning to DISARMING.")
                    self.state = MissionState.DISARMING

        elif self.state == MissionState.DISARMING:
            self.disarm()
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_STANDBY:
                self.get_logger().info("Mission complete. Vehicle disarmed.")
                self.timer.cancel()
                self.state = MissionState.IDLE

    # --- Publisher Functions ---
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = float('nan') # Let PX4 control yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

def main(args=None):
    rclpy.init(args=args)
    mission_node = MissionNode()
    try:
        rclpy.spin(mission_node)
    except KeyboardInterrupt:
        pass
    finally:
        mission_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()