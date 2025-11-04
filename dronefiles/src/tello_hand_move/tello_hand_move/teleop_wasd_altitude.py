#!/usr/bin/env python3
import sys
import termios
import tty
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist as TwistMsg

HELP = """
Movement:
  w/s : forward / backward
  a/d : left  / right
  q/e : rotate left / rotate right

Magasság:
  space  : up
    c    : down

x : exit
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

class TeleopWasdAltitude(Node):
    def __init__(self):
        super().__init__('teleop_wasd_altitude')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('model_name', 'tello')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('min_z', -10.0)
        self.declare_parameter('max_z', 100.0)
        self.declare_parameter('lin_speed', 0.8)
        self.declare_parameter('yaw_speed', 1.2)
        self.declare_parameter('dz_step', 0.1)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value
        self.lin_speed = self.get_parameter('lin_speed').get_parameter_value().double_value
        self.yaw_speed = self.get_parameter('yaw_speed').get_parameter_value().double_value
        self.dz_step = self.get_parameter('dz_step').get_parameter_value().double_value
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cli_get = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.cli_set = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        for name, cli in [
                          ('/gazebo/get_entity_state', self.cli_get),
                          ('/gazebo/set_entity_state', self.cli_set)
                         ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"Cannot connect to {name}")
                cli.wait_for_service()

        self.get_logger().info(HELP)
        self._print_status()
    def _print_status(self):
        self.get_logger().info(
            f"Speeds: lin={self.lin_speed:.2f} m/s, yaw={self.yaw_speed:.2f} rad/s, Δz={self.dz_step:.2f} m | "
            f"model='{self.model_name}', frame='{self.reference_frame}'"
        )
    def send_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)
    def step_z(self, dz: float):
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
    def run(self):
        vx = vy = wz = 0.0
        while rclpy.ok():
            c = getch()
            if c == 'x':
                self.get_logger().info("Exiting...")
                self.send_cmd(0.0, 0.0, 0.0)
                break
            if c == 'w':
                vx = +self.lin_speed
            elif c == 's':
                vx = -self.lin_speed
            elif c == 'a':
                vy = +self.lin_speed
            elif c == 'd':
                vy = -self.lin_speed
            elif c == 'q':
                wz = +self.yaw_speed
            elif c == 'e':
                wz = -self.yaw_speed
            elif c == ' ':
                self.step_z(+self.dz_step)
            elif c == 'c':
                self.step_z(-self.dz_step)
            else:
                vx = vy = wz = 0.0
            self.send_cmd(vx, vy, wz)

def main():
    rclpy.init()
    node = TeleopWasdAltitude()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
