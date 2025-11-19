#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
import transforms3d.euler as tfe
import tello_controller

from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist as TwistMsg

from tello_controller.tello_controller import TelloController

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


class TelloHandMove(Node):
    def __init__(self):
        super().__init__("tele_hand_move")

        self.declare_parameter("simulation", True)
        self.sim = self.get_parameter("simulation").value

        self.controller = TelloController(
            simulation=self.sim,
            model_name="tello_drone",
            reference_frame="world",
            cmd_vel_topic="/cmd_vel"
        )

    def run(self):
        while rclpy.ok():
            c = getch()
            if c == 'w': self.controller.step_x(+0.1)
            elif c == 's': self.controller.step_x(-0.1)
            elif c == 'a': self.controller.step_y(+0.1)
            elif c == 'd': self.controller.step_y(-0.1)
            elif c == ' ': self.controller.step_z(+0.1)
            elif c == 'c': self.controller.step_z(-0.1)
            elif c == 'q': self.controller.step_yaw(+0.05)
            elif c == 'e': self.controller.step_yaw(-0.05)
            elif c == 'x': break


def main():
    rclpy.init()
    node = TelloHandMove()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
