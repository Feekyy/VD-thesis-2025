#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist as TwistMsg

HELP = """
=== Tello Controller ===

Movement:
  w/s : forward / backward
  a/d : left / right
  q/e : rotate left / rotate right

Altitude:
  space : up
  c     : down

x : exit
=======================================
"""

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class TeleopWasdTello(Node):
    def __init__(self):
        super().__init__('teleop_wasd_tello')

        self.declare_parameter('simulation', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('model_name', 'tello_drone')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('min_z', -10.0)
        self.declare_parameter('max_z', 100.0)
        self.declare_parameter('lin_speed', 0.1)
        self.declare_parameter('yaw_speed', 1.0)
        self.declare_parameter('dz_step', 0.1)

        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value
        self.lin_speed = self.get_parameter('lin_speed').get_parameter_value().double_value
        self.yaw_speed = self.get_parameter('yaw_speed').get_parameter_value().double_value
        self.dz_step = self.get_parameter('dz_step').get_parameter_value().double_value

        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        if self.simulation:
            self.cli_get = self.create_client(GetEntityState, '/gazebo/get_entity_state')
            self.cli_set = self.create_client(SetEntityState, '/gazebo/set_entity_state')
            for name, cli in [
                ('/gazebo/get_entity_state', self.cli_get),
                ('/gazebo/set_entity_state', self.cli_set)
            ]:
                if not cli.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f"Cannot connect to {name}")
                    cli.wait_for_service()

        mode = "SIMULATION (Gazebo)" if self.simulation else "REAL TELLO"
        self.get_logger().info(f"Started Teleop in {mode} mode")
        self.get_logger().info(HELP)
        self._print_status()

    def _print_status(self):
        self.get_logger().info(
            f"Speeds: lin={self.lin_speed:.2f} m/s, yaw={self.yaw_speed:.2f} rad/s, Δz={self.dz_step:.2f} m | "
            f"model='{self.model_name}', frame='{self.reference_frame}'"
        )

    def send_cmd(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def step_x(self, dx: float):
        if self.simulation:
            req_g = GetEntityState.Request()
            req_g.name = self.model_name
            req_g.reference_frame = self.reference_frame
            fut_g = self.cli_get.call_async(req_g)
            rclpy.spin_until_future_complete(self, fut_g)

            if not fut_g.result() or not fut_g.result().success:
                self.get_logger().error("GetEntityState unsuccessful.")
                return

            pose: Pose = fut_g.result().state.pose
            twist: TwistMsg = fut_g.result().state.twist

            new_x = pose.position.x + dx
            pose.position.x = new_x

            req_s = SetEntityState.Request()
            req_s.state = EntityState()
            req_s.state.name = self.model_name
            req_s.state.reference_frame = self.reference_frame
            req_s.state.pose = pose
            req_s.state.twist = twist

            fut_s = self.cli_set.call_async(req_s)
            rclpy.spin_until_future_complete(self, fut_s)

            if not fut_s.result() or not fut_s.result().success:
                self.get_logger().error("SetEntityState unsuccessful.")
            else:
                self.get_logger().info(f"X → {new_x:.3f} m")

        else:
            vx = dx / abs(dx) * self.lin_speed if dx != 0 else 0.0
            self.send_cmd(vx, 0.0, 0.0, 0.0)
            self.get_logger().info(f"Sent forward velocity: {vx:.2f} m/s")

    def step_y(self, dy: float):
        if self.simulation:
            req_g = GetEntityState.Request()
            req_g.name = self.model_name
            req_g.reference_frame = self.reference_frame
            fut_g = self.cli_get.call_async(req_g)
            rclpy.spin_until_future_complete(self, fut_g)

            if not fut_g.result() or not fut_g.result().success:
                self.get_logger().error("GetEntityState unsuccessful.")
                return

            pose: Pose = fut_g.result().state.pose
            twist: TwistMsg = fut_g.result().state.twist

            new_y = pose.position.y + dy
            pose.position.y = new_y

            req_s = SetEntityState.Request()
            req_s.state = EntityState()
            req_s.state.name = self.model_name
            req_s.state.reference_frame = self.reference_frame
            req_s.state.pose = pose
            req_s.state.twist = twist

            fut_s = self.cli_set.call_async(req_s)
            rclpy.spin_until_future_complete(self, fut_s)

            if not fut_s.result() or not fut_s.result().success:
                self.get_logger().error("SetEntityState unsuccessful.")
            else:
                self.get_logger().info(f"Y → {new_y:.3f} m")

        else:
            vy = dy / abs(dy) * self.lin_speed if dy != 0 else 0.0
            self.send_cmd(0.0, vy, 0.0, 0.0)
            self.get_logger().info(f"Sent lateral velocity: {vy:.2f} m/s")


    def step_z(self, dz: float):
        if self.simulation:
            req_g = GetEntityState.Request()
            req_g.name = self.model_name
            req_g.reference_frame = self.reference_frame
            fut_g = self.cli_get.call_async(req_g)
            rclpy.spin_until_future_complete(self, fut_g)
            if not fut_g.result() or not fut_g.result().success:
                self.get_logger().error("GetEntityState unsuccessful.")
                return
            pose: Pose = fut_g.result().state.pose
            twist: TwistMsg = fut_g.result().state.twist
            new_z = max(self.min_z, min(self.max_z, pose.position.z + dz))
            pose.position.z = new_z
            req_s = SetEntityState.Request()
            req_s.state = EntityState()
            req_s.state.name = self.model_name
            req_s.state.reference_frame = self.reference_frame
            req_s.state.pose = pose
            req_s.state.twist = twist
            fut_s = self.cli_set.call_async(req_s)
            rclpy.spin_until_future_complete(self, fut_s)
            if not fut_s.result() or not fut_s.result().success:
                self.get_logger().error("SetEntityState unsuccessful.")
            else:
                self.get_logger().info(f"Z → {new_z:.3f} m")
        else:
            vz = dz / abs(dz) * self.lin_speed if dz != 0 else 0.0
            self.send_cmd(0.0, 0.0, vz, 0.0)
            self.get_logger().info(f"Sent vertical velocity: {vz:.2f} m/s")

    def step_yaw(self, dtheta: float):
        if self.simulation:
            req_g = GetEntityState.Request()
            req_g.name = self.model_name
            req_g.reference_frame = self.reference_frame
            fut_g = self.cli_get.call_async(req_g)
            rclpy.spin_until_future_complete(self, fut_g)

            if not fut_g.result() or not fut_g.result().success:
                self.get_logger().error("GetEntityState unsuccessful.")
                return

            pose: Pose = fut_g.result().state.pose
            twist: TwistMsg = fut_g.result().state.twist

            import tf_transformations
            quat = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)

            new_yaw = yaw + dtheta

            new_quat = tf_transformations.quaternion_from_euler(roll, pitch, new_yaw)
            pose.orientation.x = new_quat[0]
            pose.orientation.y = new_quat[1]
            pose.orientation.z = new_quat[2]
            pose.orientation.w = new_quat[3]

            req_s = SetEntityState.Request()
            req_s.state = EntityState()
            req_s.state.name = self.model_name
            req_s.state.reference_frame = self.reference_frame
            req_s.state.pose = pose
            req_s.state.twist = twist

            fut_s = self.cli_set.call_async(req_s)
            rclpy.spin_until_future_complete(self, fut_s)

            if not fut_s.result() or not fut_s.result().success:
                self.get_logger().error("SetEntityState unsuccessful.")
            else:
                self.get_logger().info(f"Yaw → {math.degrees(new_yaw):.2f}°")

        else:
            wz = dtheta / abs(dtheta) * self.ang_speed if dtheta != 0 else 0.0
            self.send_cmd(0.0, 0.0, 0.0, wz)
            self.get_logger().info(f"Sent angular velocity: {wz:.2f} rad/s")

    def run(self):
        vx = vy = wz = 0.0
        while rclpy.ok():
            c = getch()
            if c == 'x':
                self.get_logger().info("Exiting...")
                self.send_cmd(0.0, 0.0, 0.0, 0.0)
                break
            elif c == 'w':
                self.step_x(+self.lin_speed)
                continue
            elif c == 's':
                self.step_x(-self.lin_speed)
                continue
            elif c == 'a':
                self.step_y(+self.lin_speed)
                continue
            elif c == 'd':
                self.step_y(-self.lin_speed)
                continue
            elif c == 'q':
                self.step_yaw(+self.yaw_speed)
                continue
            elif c == 'e':
                self.step_yaw(+self.yaw_speed)
                continue
            elif c == ' ':
                self.step_z(+self.dz_step)
                continue
            elif c == 'c':
                self.step_z(-self.dz_step)
                continue
            else:
                vx = vy = wz = 0.0
            self.send_cmd(vx, vy, 0.0, wz)


def main():
    rclpy.init()
    node = TeleopWasdTello()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
