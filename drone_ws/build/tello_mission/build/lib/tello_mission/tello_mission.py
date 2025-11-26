#!/usr/bin/env python3
import rclpy
import json
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from tello_controller.tello_controller import TelloController

class MissionNode(Node):
    def __init__(self):
        super().__init__("tello_mission")

        self.declare_parameter("simulation", True)
        self.sim = self.get_parameter("simulation").value

        self.controller = TelloController(
            simulation=self.sim,
            model_name="tello_drone",
            reference_frame="world",
            cmd_vel_topic="/cmd_vel"
        )

        self.create_subscription(Bool, "donut_detector/activation", self.cb_activation, 10)
        self.create_subscription(Float32, "donut_detector/activation_degree", self.cb_degree, 10)
        self.create_subscription(String, "donut_detector/circles", self.cb_circles, 10)

        self.active = False
        self.degree = 0.0
        self.circles = []
        self.pass_counter = 0

        self.color_order = ["blue", "red", "green"]
        self.current_index = 0
        self.img_center_x = 450
        self.img_center_y = 290

        self.k_xy = 0.0001
        self.k_z = 0.0015
        self.k_yaw = 0.0005

        self.forward_step = 0.12
        self.radius_threshold_min = 100
        self.radius_threshold_max = 300
        self.last_known_radius = 0.0

        self.missing_counter = 0
        self.missing_threshold = 5
        self.pass_steps = 75
        
        self.max_step_x = 0.20
        self.max_step_y = 0.20
        self.max_step_z = 0.50
        
        self.search_state = 'rotate'
        self.search_step_duration = 0
        self.search_rotation_duration = 40
        self.search_forward_duration = 30
        self.search_yaw_rate = 0.5
        self.search_forward_speed = 0.1

        self.search_height_step = 0.2
        self.search_height_dir = 1.0
        
        self.rotation_steps = 0
        self.rotation_duration = 20

    def cb_activation(self, msg):
        self.active = msg.data

    def cb_degree(self, msg):
        self.degree = msg.data

    def cb_circles(self, msg):
        try:
            text = msg.data.strip()
            loaded = json.loads(text)
            
            self.circles = [
                c for c in loaded 
                if 20 < c.get("r", 0) < 500
            ]
            
        except Exception as e:
            self.get_logger().warn(f"JSON parse error: {e}")
            self.circles = []

        self.control()

    def center_distance_sq(self, c):
        dx = c.get("x", self.img_center_x) - self.img_center_x
        dy = c.get("y", self.img_center_y) - self.img_center_y
        return dx*dx + dy*dy

    def control(self):
        if self.current_index >= len(self.color_order):
            self.get_logger().info("All toruses completed! Hovering.")
            self.controller.send_cmd(0.0, 0.0, 0.0, 0.0)
            return

        target_color = self.color_order[self.current_index]

        if self.rotation_steps > 0:
            self.controller.step_yaw(0.5)
            self.rotation_steps -= 1
            self.get_logger().info(f"Rotating 180 degrees... remaining: {self.rotation_steps}")
            if self.rotation_steps == 0:
                self.get_logger().info("180 degree rotation complete.")
            return

        if self.pass_counter > 0:
            self.controller.step_x(0.18)
            self.controller.step_y(0.0)
            self.controller.step_z(0.0)
            self.controller.step_yaw(0.0)
            
            self.pass_counter -= 1
            if self.pass_counter == 0:
                self.get_logger().info(f"Completed {target_color} torus, finished pass-through.")
                self.current_index += 1
                if self.current_index < len(self.color_order) and self.current_index % 2 == 1:
                    self.rotation_steps = self.rotation_duration
            return

        target_circles = [c for c in self.circles if c.get("color") == target_color]
        
        if len(target_circles) == 0:
            if self.last_known_radius > 200:
                self.get_logger().info(f"Target lost but close (r={self.last_known_radius:.1f}). Forcing pass-through.")
                self.controller.send_cmd(0.0, 0.0, 0.0, 0.0)
                self.pass_counter = self.pass_steps
                self.last_known_radius = 0.0
                return
            if self.search_step_duration <= 0:
                if self.search_state == 'rotate':
                    self.search_state = 'forward'
                    self.search_step_duration = self.search_forward_duration
                    self.get_logger().info("Search: Target lost. Moving forward.")
                else:
                    self.search_state = 'rotate'
                    self.search_step_duration = self.search_rotation_duration
                    self.get_logger().info("Search: Target lost. Rotating.")
                    self.search_height_dir *= -1.0
                    dz = self.search_height_step * self.search_height_dir
                    self.controller.step_z(dz)
                    self.get_logger().info(f"Search: Adjusting height by {dz:.2f} m.")

            self.search_step_duration -= 1

            step_x = 0.0
            step_yaw = 0.0
            
            if self.search_state == 'rotate':
                step_yaw = self.search_yaw_rate 
            elif self.search_state == 'forward':
                step_x = self.search_forward_speed 
            
            self.controller.step_x(step_x)
            self.controller.step_y(0.0)
            self.controller.step_z(0.0) 
            self.controller.step_yaw(step_yaw)
            self.last_known_radius = 0.0
            return

        self.missing_counter = 0
        self.search_step_duration = 0
        self.search_state = 'rotate'
        
        torus = max(target_circles, key=lambda c: c.get("r", 0))

        cx = float(torus.get("x", self.img_center_x))
        cy = float(torus.get("y", self.img_center_y))
        r  = float(torus.get("r", 0))

        self.last_known_radius = r

        if r >= self.radius_threshold_max:
            self.controller.send_cmd(0.0, 0.0, 0.0, 0.0)
            self.pass_counter = self.pass_steps
            self.last_known_radius = 0.0
            self.get_logger().info(f"{target_color} torus: close enough (r={r:.1f}), starting pass-through.")
            return
        
        error_x = cx - self.img_center_x
        error_y = cy - self.img_center_y

        step_yaw = -error_x * self.k_yaw 
        step_y = 0.0 
        step_z = -error_y * self.k_z
        
        step_x_min = 0.08
        if r > 180:
            step_x_min = self.forward_step
            
        ALIGN_TOL = 30
        if abs(error_x) < ALIGN_TOL and abs(error_y) < ALIGN_TOL:
            step_x = self.max_step_x
            step_yaw = 0.0
            self.get_logger().info(f"Aligned. Moving towards {target_color} torus. Speed: {step_x:.2f} m/s (MAX SPEED)")
        else:
            step_x = step_x_min
            self.get_logger().info(f"Alignment required: dx={error_x:.0f}, dy={error_y:.0f}. Correcting.")

        step_x = float(np.clip(step_x, -self.max_step_x, self.max_step_x))
        step_y = float(np.clip(step_y, -self.max_step_y, self.max_step_y)) 
        step_z = float(np.clip(step_z, -self.max_step_z, self.max_step_z))
        step_yaw = float(np.clip(step_yaw, -0.15, 0.15)) 

        self.controller.step_x(step_x)
        self.controller.step_y(step_y)
        self.controller.step_z(step_z)
        self.controller.step_yaw(step_yaw)

        self.get_logger().info(
            f"Tracking {target_color}: dx={step_x:.2f}, yaw={step_yaw:.3f}, r={r:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()