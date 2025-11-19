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
            f"TelloController started (simulation={self.simulation}, model={self.model_name})"
        )

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
            q = pose.orientation
            q_w, q_x, q_y, q_z = q.w, q.x, q.y, q.z
            current_roll, current_pitch, current_yaw = tfe.quat2euler([q_w, q_x, q_y, q_z], axes='sxyz')
            yaw = current_yaw
            
            dx_body = dx
            dy_body = 0.0

            dx_world = math.cos(yaw) * dx_body - math.sin(yaw) * dy_body
            dy_world = math.sin(yaw) * dx_body + math.cos(yaw) * dy_body
            
            pose.position.x += dx_world
            pose.position.y += dy_world

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

            q = pose.orientation
            q_w, q_x, q_y, q_z = q.w, q.x, q.y, q.z
            current_roll, current_pitch, current_yaw = tfe.quat2euler([q_w, q_x, q_y, q_z], axes='sxyz')
            yaw = current_yaw

            dx_body = 0.0
            dy_body = dy

            dx_world = math.cos(yaw) * dx_body - math.sin(yaw) * dy_body
            dy_world = math.sin(yaw) * dx_body + math.cos(yaw) * dy_body

            pose.position.x += dx_world
            pose.position.y += dy_world
            
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
            vz = dz / abs(dz) * self.lin_speed if dz != 0 else 0.0
            self.send_cmd(0.0, 0.0, vz, 0.0)
            self.get_logger().info(f"Sent vertical velocity: {vz:.2f} m/s")

    def step_yaw(self, dyaw: float):
        if self.simulation:
            req_g = GetEntityState.Request()
            req_g.name = self.model_name
            req_g.reference_frame = self.reference_frame
            fut_g = self.cli_get.call_async(req_g)
            rclpy.spin_until_future_complete(self, fut_g)
            if not fut_g.result() or not fut_g.result().success:
                self.get_logger().error("GetEntityState unsuccessful.")
                return

            pose = fut_g.result().state.pose
            twist = fut_g.result().state.twist

            q = pose.orientation
            current_euler = tfe.quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')
            new_yaw = current_euler[2] + dyaw

            new_quat = tfe.euler2quat(current_euler[0], current_euler[1], new_yaw, axes='sxyz')
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
            self.send_cmd(0.0, 0.0, 0.0, dyaw)
            self.get_logger().info(f"Sent yaw velocity: {dyaw:.2f} rad/s")

    def get_current_z(self):
        if not self.simulation:
            self.get_logger().warn("get_current_z not implemented for real Tello.")
            return 1.0
            
        req_g = GetEntityState.Request()
        req_g.name = self.model_name
        req_g.reference_frame = self.reference_frame
        
        fut_g = self.cli_get.call_async(req_g)
        rclpy.spin_until_future_complete(self, fut_g)
        
        if fut_g.result() and fut_g.result().success:
            return fut_g.result().state.pose.position.z
        
        self.get_logger().error("Failed to get Z position from Gazebo.")
        return 1.0