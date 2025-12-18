import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


class MissionNode(Node):
    STATE_WAITING_FOR_OFFBOARD = 0
    STATE_TAKEOFF = 1
    STATE_GO_TO = 2
    STATE_HOLD = 3

    def __init__(self):
        super().__init__('mission_node')

        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.sub_local_pos = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_local_position,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,  # PX4 publishes best-effort sensor QoS
        )

        self.takeoff_height = self._declare_float('takeoff_height', 5.0)
        self.goal_x = self._declare_float('goal_x', 5.0)
        self.goal_y = self._declare_float('goal_y', 0.0)
        self.goal_z = self._declare_float('goal_z', -5.0)  # NED frame: positive is down
        self.goal_yaw = self._declare_float('goal_yaw', 0.0)
        self.pos_tol_xy = self._declare_float('position_tolerance_xy', 0.5)
        self.pos_tol_z = self._declare_float('position_tolerance_z', 0.5)
        self.hold_time_sec = self._declare_float('hold_time', 3.0)
        self.takeoff_timeout_sec = self._declare_float('takeoff_timeout', 8.0)

        self.target_takeoff_z = -abs(self.takeoff_height)
        self.goal = (self.goal_x, self.goal_y, self.goal_z)

        self.state = self.STATE_WAITING_FOR_OFFBOARD
        self.offboard_armed = False
        self.last_state_change = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.goal_reached_since = None
        self.last_position = None

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.get_logger().info(
            f'Starting mission node. Using NED frame (z down). '
            f'Takeoff to {-self.target_takeoff_z:.2f} m above home, then go to '
            f'(x={self.goal_x:.2f}, y={self.goal_y:.2f}, z={self.goal_z:.2f}) with yaw={self.goal_yaw:.2f} rad.'
        )

    def _declare_float(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        return float(self.get_parameter(name).value)

    def on_local_position(self, msg: VehicleLocalPosition) -> None:
        self.last_position = msg

    def loop(self) -> None:
        now = self.get_clock().now()
        now_us = int(now.nanoseconds / 1000)

        self._publish_offboard_control_mode(now_us)

        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.yaw = self.goal_yaw

        if self.state == self.STATE_WAITING_FOR_OFFBOARD:
            sp.position = [0.0, 0.0, self.target_takeoff_z]
            if not self.offboard_armed and (now - self.start_time).nanoseconds > 1e9:
                self.set_offboard()
                self.arm()
                self.offboard_armed = True
                self.state = self.STATE_TAKEOFF
                self.last_state_change = now
                self.get_logger().info('Offboard set and armed; climbing to takeoff height.')

        elif self.state == self.STATE_TAKEOFF:
            sp.position = [0.0, 0.0, self.target_takeoff_z]
            if self._has_reached_altitude(self.target_takeoff_z):
                self.state = self.STATE_GO_TO
                self.last_state_change = now
                self.goal_reached_since = None
                self.get_logger().info('Takeoff complete; proceeding to goal.')
            elif (now - self.last_state_change).nanoseconds >= int(self.takeoff_timeout_sec * 1e9):
                self.state = self.STATE_GO_TO
                self.last_state_change = now
                self.goal_reached_since = None
                self.get_logger().warn(
                    'Takeoff timeout reached without altitude confirmation; proceeding to goal based on timer.'
                )

        elif self.state == self.STATE_GO_TO:
            sp.position = list(self.goal)
            if self._at_goal():
                if self.goal_reached_since is None:
                    self.goal_reached_since = now
                elif (now - self.goal_reached_since).nanoseconds >= int(self.hold_time_sec * 1e9):
                    self.state = self.STATE_HOLD
                    self.last_state_change = now
                    self.get_logger().info('Goal reached; holding position.')
            else:
                self.goal_reached_since = None

        elif self.state == self.STATE_HOLD:
            sp.position = list(self.goal)

        else:
            sp.position = [0.0, 0.0, self.target_takeoff_z]

        self.pub_traj.publish(sp)

    def _publish_offboard_control_mode(self, timestamp_us: int) -> None:
        msg = OffboardControlMode()
        msg.timestamp = timestamp_us
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.pub_offboard.publish(msg)

    def _has_reached_altitude(self, target_z: float) -> bool:
        if self.last_position is None or not self.last_position.z_valid:
            return False
        return abs(self.last_position.z - target_z) <= self.pos_tol_z

    def _at_goal(self) -> bool:
        if self.last_position is None or not (self.last_position.xy_valid and self.last_position.z_valid):
            return False
        dx = self.last_position.x - self.goal[0]
        dy = self.last_position.y - self.goal[1]
        dz = self.last_position.z - self.goal[2]
        xy_dist = math.hypot(dx, dy)
        return xy_dist <= self.pos_tol_xy and abs(dz) <= self.pos_tol_z

    def arm(self) -> None:
        self._cmd(400, param1=1.0)  # MAV_CMD_COMPONENT_ARM_DISARM
        self.get_logger().info('Arming...')

    def set_offboard(self) -> None:
        self._cmd(176, param1=1.0, param2=6.0)  # MAV_CMD_DO_SET_MODE
        self.get_logger().info('Switching to OFFBOARD...')

    def _cmd(self, command: int, **kw) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = kw.get('param1', 0.0)
        msg.param2 = kw.get('param2', 0.0)
        msg.param3 = kw.get('param3', 0.0)
        msg.param4 = kw.get('param4', 0.0)
        msg.param5 = kw.get('param5', 0.0)
        msg.param6 = kw.get('param6', 0.0)
        msg.param7 = kw.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
