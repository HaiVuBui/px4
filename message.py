#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleStatus,
    VehicleAttitude,
    VehicleLocalPosition,
)
from manipulation_msgs.srv import SetPose


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion [w, x, y, z]."""
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return q
    return q / norm


def quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q
    # Standard Hamilton -> rotation matrix
    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )
    return R


class Px4Manipulation(Node):
    """
    ROS 2 Python node for omnidirectional PX4 control (position + attitude).

    Publishes:
      - /fmu/in/offboard_control_mode        (px4_msgs/OffboardControlMode)
      - /fmu/in/vehicle_attitude_setpoint   (px4_msgs/VehicleAttitudeSetpoint)

    Subscribes:
      - /fmu/out/vehicle_status             (px4_msgs/VehicleStatus)
      - /fmu/out/vehicle_attitude           (px4_msgs/VehicleAttitude)
      - /fmu/out/vehicle_local_position     (px4_msgs/VehicleLocalPosition)

    Service:
      - /set_pose (manipulation_msgs/SetPose)
    """

    def __init__(self):
        super().__init__("px4_manipulation")

        # --- QoS: KeepLast(1), best effort (like C++ code) ---
        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # --- Publishers ---
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.vehicle_attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            "/fmu/in/vehicle_attitude_setpoint",
            qos_profile,
        )

        # --- Subscribers ---
        self.vehicle_nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile,
        )

        # --- Service ---
        self.create_service(
            SetPose,
            "/set_pose",
            self.target_pose_callback,
        )

        # --- Internal state (Eigen -> NumPy) ---
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.vehicle_position = np.zeros(3)
        self.vehicle_velocity = np.zeros(3)

        self.reference_position = np.zeros(3)
        self.reference_attitude = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

        # --- Control loop timer (20 ms = 50 Hz) ---
        self.statusloop_timer = self.create_timer(0.02, self.statusloop_callback)

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def statusloop_callback(self):
        """
        Simple PID position controller + attitude / thrust setpoint publisher.
        """
        # Position error
        error_position = self.vehicle_position - self.reference_position

        kp = 0.05
        kd = 0.05
        hover_thrust_inertial = np.array([0.0, 0.0, 0.2])

        acceleration_feedback = -kp * error_position - kd * self.vehicle_velocity

        # Compute thrust vector in inertial frame
        thrust_inertial = acceleration_feedback + hover_thrust_inertial

        # Saturate thrust between -1 and 1
        thrust_inertial = np.clip(thrust_inertial, -1.0, 1.0)

        # Convert vehicle attitude quaternion to rotation matrix
        q = normalize_quaternion(self.vehicle_attitude)
        R = quat_to_rot_matrix(q)

        # Transform thrust to body frame: R^T * thrust_inertial
        thrust_body = R.T @ thrust_inertial

        # --- Offboard control mode ---
        now_us = self.get_clock().now().nanoseconds // 1000

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = now_us
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.attitude = True
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)

        # --- Attitude setpoint (only in OFFBOARD) ---
        if self.vehicle_nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            att_sp_msg = VehicleAttitudeSetpoint()
            att_sp_msg.timestamp = now_us

            # PX4 quaternion + frame conventions (same mapping as C++)
            att_q = normalize_quaternion(self.reference_attitude)
            att_sp_msg.q_d[0] = att_q[0]  # w
            att_sp_msg.q_d[1] = att_q[1]  # x
            att_sp_msg.q_d[2] = -att_q[2]  # -y
            att_sp_msg.q_d[3] = -att_q[3]  # -z

            # Thrust in PX4 body coordinates (signs like C++ code)
            att_sp_msg.thrust_body[0] = float(thrust_body[0])
            att_sp_msg.thrust_body[1] = float(-thrust_body[1])
            att_sp_msg.thrust_body[2] = float(-thrust_body[2])

            self.vehicle_attitude_pub.publish(att_sp_msg)

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_nav_state = msg.nav_state

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        """
        Update vehicle attitude quaternion.
        Mapping as in C++:
          q.w = msg.q[0]
          q.x = msg.q[1]
          q.y = -msg.q[2]
          q.z = -msg.q[3]
        """
        q = np.array(
            [
                msg.q[0],
                msg.q[1],
                -msg.q[2],
                -msg.q[3],
            ],
            dtype=float,
        )
        self.vehicle_attitude = normalize_quaternion(q)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """
        Update vehicle position and velocity.
        Mapping as in C++:
          position.x = msg.x
          position.y = -msg.y
          position.z = -msg.z
          velocity.x = msg.vx
          velocity.y = -msg.vy
          velocity.z = -msg.vz
        """
        self.vehicle_position = np.array(
            [msg.x, -msg.y, -msg.z],
            dtype=float,
        )
        self.vehicle_velocity = np.array(
            [msg.vx, -msg.vy, -msg.vz],
            dtype=float,
        )

    def target_pose_callback(self, request: SetPose.Request, response: SetPose.Response):
        """
        Service callback for /set_pose.
        Sets reference position and attitude from request.pose.
        """
        # Position
        self.reference_position[0] = request.pose.position.x
        self.reference_position[1] = request.pose.position.y
        self.reference_position[2] = request.pose.position.z

        # Orientation (quaternion [w, x, y, z])
        self.reference_attitude = normalize_quaternion(
            np.array(
                [
                    request.pose.orientation.w,
                    request.pose.orientation.x,
                    request.pose.orientation.y,
                    request.pose.orientation.z,
                ],
                dtype=float,
            )
        )

        response.result = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Px4Manipulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()