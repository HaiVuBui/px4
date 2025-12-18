import math

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint, VehicleCommand


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-6:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / norm for v in q)


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return normalize_quaternion((w, x, y, z))


def quaternion_to_euler(q):
    w, x, y, z = normalize_quaternion(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def ros_to_px4_quaternion(q):
    """
    PX4 expects attitude setpoint quaternions in FRD/NED with inverted Y/Z
    compared to ROS ENU. Flip those axes before publishing.
    """
    w, x, y, z = q
    return (w, x, -y, -z)


class AttitudeControlNode(Node):
    """
    Simple offboard attitude controller.

    Publishes an attitude + thrust setpoint so the drone climbs while spinning
    about the yaw axis. Orientation can be provided via RPY or quaternion
    parameters (ROS ENU convention); the node handles conversion to PX4 NED.
    """

    def __init__(self):
        super().__init__("attitude_control_node")

        # Parameters
        self.declare_parameter("timer_period", 0.05)  # seconds
        self.declare_parameter("orientation_mode", "rpy")  # "rpy" or "quaternion"
        self.declare_parameter("roll", 0.0)   # rad
        self.declare_parameter("pitch", 0.0)  # rad
        self.declare_parameter("yaw", 0.0)    # rad
        self.declare_parameter("qx", 0.0)
        self.declare_parameter("qy", 0.0)
        self.declare_parameter("qz", 0.0)
        self.declare_parameter("qw", 1.0)
        self.declare_parameter("yaw_rate", 0.4)  # rad/s
        self.declare_parameter("thrust", 0.6)    # normalized [0,1], published as -thrust in NED

        self.timer_period = float(self.get_parameter("timer_period").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.thrust = float(self.get_parameter("thrust").value)

        orientation_mode = str(self.get_parameter("orientation_mode").value).lower()
        if orientation_mode not in ("rpy", "quaternion"):
            self.get_logger().warn('orientation_mode must be "rpy" or "quaternion"; defaulting to "rpy"')
            orientation_mode = "rpy"
        self.orientation_mode = orientation_mode

        if self.orientation_mode == "rpy":
            self.base_roll = float(self.get_parameter("roll").value)
            self.base_pitch = float(self.get_parameter("pitch").value)
            self.base_yaw = float(self.get_parameter("yaw").value)
        else:
            qx = float(self.get_parameter("qx").value)
            qy = float(self.get_parameter("qy").value)
            qz = float(self.get_parameter("qz").value)
            qw = float(self.get_parameter("qw").value)
            self.base_roll, self.base_pitch, self.base_yaw = quaternion_to_euler(
                (qw, qx, qy, qz)
            )

        # Publishers
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.pub_attitude = self.create_publisher(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", 10
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.t0 = self.get_clock().now()
        self.did_arm = False

        self.create_timer(self.timer_period, self.loop)

    def loop(self):
        now = self.get_clock().now()
        elapsed_s = (now - self.t0).nanoseconds * 1e-9
        now_us = now.nanoseconds // 1000

        # Stream offboard mode (attitude only)
        offboard = OffboardControlMode()
        offboard.timestamp = now_us
        offboard.position = False
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = True
        offboard.body_rate = False
        offboard.thrust_and_torque = False
        offboard.direct_actuator = False
        self.pub_offboard.publish(offboard)

        # Desired attitude: keep roll/pitch, spin yaw at yaw_rate
        yaw = self.base_yaw + self.yaw_rate * elapsed_s
        q = euler_to_quaternion(self.base_roll, self.base_pitch, yaw)
        q_px4 = ros_to_px4_quaternion(q)

        att_sp = VehicleAttitudeSetpoint()
        att_sp.timestamp = now_us
        att_sp.roll_body = math.nan
        att_sp.pitch_body = math.nan
        att_sp.yaw_body = math.nan
        att_sp.q_d = q_px4
        att_sp.yaw_sp_move_rate = float(self.yaw_rate)
        att_sp.thrust_body[0] = 0.0
        att_sp.thrust_body[1] = 0.0
        att_sp.thrust_body[2] = -self.thrust  # NED frame: negative = upward thrust
        self.pub_attitude.publish(att_sp)

        # Arm + switch to offboard after streaming setpoints for ~1s
        if not self.did_arm and (now - self.t0).nanoseconds > 1e9:
            self.set_offboard()
            self.arm()
            self.did_arm = True

    def arm(self):
        self._cmd(400, param1=1.0)  # MAV_CMD_COMPONENT_ARM_DISARM
        self.get_logger().info("Arming...")

    def set_offboard(self):
        self._cmd(176, param1=1.0, param2=6.0)  # MAV_CMD_DO_SET_MODE
        self.get_logger().info("Switching to OFFBOARD...")

    def _cmd(self, command, **kw):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = kw.get("param1", 0.0)
        msg.param2 = kw.get("param2", 0.0)
        msg.param3 = kw.get("param3", 0.0)
        msg.param4 = kw.get("param4", 0.0)
        msg.param5 = kw.get("param5", 0.0)
        msg.param6 = kw.get("param6", 0.0)
        msg.param7 = kw.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = AttitudeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
