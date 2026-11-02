#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
import transforms3d.euler as tfe
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist as TwistMsg

#Initializes the TelloController with simulation mode and movement parameters
class TelloController(Node):
    def __init__(self,
                 simulation=True,
                 cmd_vel_topic='/cmd_vel',
                 model_name='tello_drone',
                 reference_frame='world'):
        super().__init__('tello_controller')
        self.simulation = simulation
        self.cmd_vel_topic = cmd_vel_topic
        self.model_name = model_name
        self.reference_frame = reference_frame
        self.declare_parameter('min_z', -10.0)
        self.declare_parameter('max_z', 100.0)
        self.declare_parameter('lin_speed', 0.1)
        self.declare_parameter('yaw_speed', 0.05)
        self.declare_parameter('dz_step', 0.1)
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.lin_speed = self.get_parameter('lin_speed').value
        self.yaw_speed = self.get_parameter('yaw_speed').value
        self.dz_step = self.get_parameter('dz_step').value
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        if self.simulation:
            self.cli_get = self.create_client(GetEntityState, '/gazebo/get_entity_state')
            self.cli_set = self.create_client(SetEntityState, '/gazebo/set_entity_state')
            self.cli_get.wait_for_service()
            self.cli_set.wait_for_service()
        self.get_logger().info(
            f"Controller started (simulation={self.simulation}, model={self.model_name})"
        )

    #Logs current controller settings and speed parameters
    def _print_status(self):
        self.get_logger().info(
            f"Speeds: lin={self.lin_speed:.2f} m/s, yaw={self.yaw_speed:.2f} rad/s, Δz={self.dz_step:.2f} m | "
            f"model='{self.model_name}', frame='{self.reference_frame}'"
        )

    #Publishes a velocity command to the drone via Twist message
    def send_cmd(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    #Moves the drone on every axis at once (body frame: x forward, y left, z up, yaw rotation)
    def step(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
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
            q = pose.orientation
            current_roll, current_pitch, current_yaw = tfe.quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')

            #Body frame -> world frame, so the drone always moves relative to its own heading
            dx_world = math.cos(current_yaw) * dx - math.sin(current_yaw) * dy
            dy_world = math.sin(current_yaw) * dx + math.cos(current_yaw) * dy

            pose.position.x += dx_world
            pose.position.y += dy_world
            pose.position.z = max(self.min_z, min(self.max_z, pose.position.z + dz))

            new_quat = tfe.euler2quat(current_roll, current_pitch, current_yaw + dyaw, axes='sxyz')
            pose.orientation.w = new_quat[0]
            pose.orientation.x = new_quat[1]
            pose.orientation.y = new_quat[2]
            pose.orientation.z = new_quat[3]

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

            #One Twist with every axis, so the commands don't overwrite each other
            vx = math.copysign(self.lin_speed, dx) if dx != 0 else 0.0
            vy = math.copysign(self.lin_speed, dy) if dy != 0 else 0.0
            vz = math.copysign(self.lin_speed, dz) if dz != 0 else 0.0
            self.send_cmd(vx, vy, vz, dyaw)
            self.get_logger().info
            (
                f"Sent velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s, yaw={dyaw:.2f} rad/s"
            )

    #Moves the drone forward/backward
    def step_x(self, dx: float):
        self.step(dx=dx)

    #Moves the drone left/right
    def step_y(self, dy: float):
        self.step(dy=dy)

    #Moves the drone up/down
    def step_z(self, dz: float):
        self.step(dz=dz)

    #Rotates the drone
    def step_yaw(self, dyaw: float):
        self.step(dyaw=dyaw)
