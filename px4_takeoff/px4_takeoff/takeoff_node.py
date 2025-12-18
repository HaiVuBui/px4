import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj     = self.create_publisher(TrajectorySetpoint,     '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd      = self.create_publisher(VehicleCommand,         '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.t0 = self.get_clock().now()
        self.did_arm = False

    def loop(self):
        # declare we’re using position setpoints
        o = OffboardControlMode()
        o.position = True
        self.pub_offboard.publish(o)

        # Hover at z = -5.0 (NED), i.e., 5.0 m above takeoff point
        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -5.0]
        sp.yaw = 0.0
        self.pub_traj.publish(sp)

        # after ~1s of streaming setpoints, switch to OFFBOARD and arm
        if not self.did_arm and (self.get_clock().now() - self.t0).nanoseconds > 1e9:
            self.set_offboard()
            self.arm()
            self.did_arm = True

    def arm(self):
        self._cmd(400, param1=1.0)   # MAV_CMD_COMPONENT_ARM_DISARM
        self.get_logger().info('Arming...')

    def set_offboard(self):
        self._cmd(176, param1=1.0, param2=6.0)  # MAV_CMD_DO_SET_MODE
        self.get_logger().info('Switching to OFFBOARD...')

    def _cmd(self, command, **kw):
        msg = VehicleCommand()
        msg.command = command
        msg.param1  = kw.get('param1', 0.0)
        msg.param2  = kw.get('param2', 0.0)
        msg.param3  = kw.get('param3', 0.0)
        msg.param4  = kw.get('param4', 0.0)
        msg.param5  = kw.get('param5', 0.0)
        msg.param6  = kw.get('param6', 0.0)
        msg.param7  = kw.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()