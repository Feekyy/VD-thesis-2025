#!/usr/bin/env python3
import rclpy
import csv
import json
import math
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from tello_controller.tello_controller import TelloController
from tello_mission.pid import PID

#Mission node that sequences through colored donuts and commands the drone
class TelloMission(Node):
    def __init__(self):
        super().__init__("tello_mission")

        #Parameters (must be read before the controller is created)
        self.declare_parameter("simulation", True)
        self.sim = self.get_parameter("simulation").value

        self.controller = TelloController(
            simulation=self.sim,
            model_name="tello_drone",
            reference_frame="world",
            cmd_vel_topic="/cmd_vel"
        )

        #Subscriptions
        self.create_subscription(String, "donut_detector/circles", self.cb_circles, 10)
        self.circles = []
        self.pass_counter = 0
        self.color_order = ["blue", "red", "green"]
        self.current_index = 0
        self.img_center_x = 450
        self.img_center_y = 290

        #PID gains for turning (yaw) and height (z), one sample = one camera frame
        #Defaults come from the measurements in results/pid: P control was the best, I and D are off (0)
        self.declare_parameter("kp_yaw", 0.0005)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.0)
        self.declare_parameter("kp_z", 0.0024)
        self.declare_parameter("ki_z", 0.0)
        self.declare_parameter("kd_z", 0.0)
        self.declare_parameter("i_limit", 400.0)
        i_limit = self.get_parameter("i_limit").value
        self.pid_yaw = PID(
            self.get_parameter("kp_yaw").value,
            self.get_parameter("ki_yaw").value,
            self.get_parameter("kd_yaw").value,
            i_limit
        )
        self.pid_z = PID(
            self.get_parameter("kp_z").value,
            self.get_parameter("ki_z").value,
            self.get_parameter("kd_z").value,
            i_limit
        )
        self.get_logger().info(
            f"PID yaw: kp={self.pid_yaw.kp}, ki={self.pid_yaw.ki}, kd={self.pid_yaw.kd} | "
            f"PID z: kp={self.pid_z.kp}, ki={self.pid_z.ki}, kd={self.pid_z.kd}"
        )

        #Stale images: the detector can send the same donut again before the camera shows the last move
        self.declare_parameter("skip_stale_frames", True)
        self.skip_stale_frames = self.get_parameter("skip_stale_frames").value
        self.last_measurement = None

        #Optional per-frame CSV log for analysing the controller (empty = off)
        self.declare_parameter("log_file", "")
        log_file = self.get_parameter("log_file").value
        self.log_writer = None
        self.frame = 0
        if log_file:
            self.log_handle = open(log_file, "w", newline="")
            self.log_writer = csv.writer(self.log_handle)
            self.log_writer.writerow(
                ["frame", "time", "color", "phase", "error_x", "error_y", "r", "step_x", "step_z", "step_yaw"]
            )

        #Movement and thresholds
        self.forward_step = 0.12
        self.radius_threshold_max = 300
        self.last_known_radius = 0.0

        #Search / tracking helpers
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
        self.rotation_yaw_step = math.pi / self.rotation_duration

        #Alignment before approach: hover and turn onto the donut first, then fly towards it
        self.align_tol = 30
        self.realign_tol = 80
        self.align_frames_required = 10
        self.align_counter = 0
        self.approaching = False

    #Forget the alignment state (target lost or pass-through started)
    def reset_alignment(self):
        self.align_counter = 0
        self.approaching = False

    #Forget everything about the current target, including the PID history
    def reset_tracking(self):
        self.reset_alignment()
        self.pid_yaw.reset()
        self.pid_z.reset()
        self.last_measurement = None

    #Writes one row into the CSV log (if it is turned on)
    def log_frame(self, color, phase, error_x="", error_y="", r="", step_x=0.0, step_z=0.0, step_yaw=0.0):
        if self.log_writer is None:
            return
        self.log_writer.writerow(
            [self.frame, f"{time.monotonic():.3f}", color, phase, error_x, error_y, r, step_x, step_z, step_yaw]
        )
        self.log_handle.flush()

    #Parse detected circles JSON and run control loop
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

    #Main control logic
    def control(self):
        self.frame += 1

        #If finished the sequence, hover
        if self.current_index >= len(self.color_order):
            self.get_logger().info("All toruses completed! Hovering.")
            self.controller.send_cmd(0.0, 0.0, 0.0, 0.0)
            return

        target_color = self.color_order[self.current_index]

        #Handle in-progress rotation step (180-degree rotation)
        if self.rotation_steps > 0:
            self.controller.step(dyaw=self.rotation_yaw_step)
            self.log_frame(target_color, "rotate", step_yaw=self.rotation_yaw_step)
            self.rotation_steps -= 1
            self.get_logger().info(f"Rotating 180 degrees... remaining: {self.rotation_steps}")
            if self.rotation_steps == 0:
                self.get_logger().info("180 degree rotation complete.")
            return

        #Handle pass-through phase where drone just moves forward
        if self.pass_counter > 0:
            self.controller.step(dx=0.18)
            self.log_frame(target_color, "pass", step_x=0.18)

            self.pass_counter -= 1
            if self.pass_counter == 0:
                self.get_logger().info(f"Completed {target_color} torus, finished pass-through.")
                self.current_index += 1
                if self.current_index < len(self.color_order) and self.current_index % 2 == 1:
                    self.rotation_steps = self.rotation_duration
            return

        target_circles = [c for c in self.circles if c.get("color") == target_color]
        
        #If no detections, perform search behaviour
        if len(target_circles) == 0:
            self.reset_tracking()

            #If we recently saw a very close target, force pass-through
            if self.last_known_radius > 200:
                self.get_logger().info(
                    f"Target lost but close (r={self.last_known_radius:.1f}). Forcing pass-through."
                )
                self.controller.send_cmd(0.0, 0.0, 0.0, 0.0)
                self.pass_counter = self.pass_steps
                self.last_known_radius = 0.0
                return

            #Switch between rotate and forward search states
            step_z = 0.0
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
                    step_z = self.search_height_step * self.search_height_dir
                    self.get_logger().info(f"Search: Adjusting height by {step_z:.2f} m.")

            self.search_step_duration -= 1
            step_x = 0.0
            step_yaw = 0.0
            
            if self.search_state == 'rotate':
                step_yaw = self.search_yaw_rate 
            elif self.search_state == 'forward':
                step_x = self.search_forward_speed 
            self.controller.step(dx=step_x, dz=step_z, dyaw=step_yaw)
            self.log_frame(target_color, "search", step_x=step_x, step_z=step_z, step_yaw=step_yaw)
            self.last_known_radius = 0.0
            return

        #Reset search bookkeeping and pick the best donut
        self.search_step_duration = 0
        self.search_state = 'rotate'
        
        torus = max(target_circles, key=lambda c: c.get("r", 0))

        cx = float(torus.get("x", self.img_center_x))
        cy = float(torus.get("y", self.img_center_y))
        r  = float(torus.get("r", 0))

        #Same donut as last time: the camera does not show the last command yet, so wait for a fresh image
        measurement = (cx, cy, r)
        if self.skip_stale_frames and measurement == self.last_measurement:
            self.log_frame(target_color, "stale", cx - self.img_center_x, cy - self.img_center_y, r)
            return
        self.last_measurement = measurement

        self.last_known_radius = r

        #If close enough, initiate pass-through
        if r >= self.radius_threshold_max:
            self.controller.send_cmd(0.0, 0.0, 0.0, 0.0)
            self.reset_tracking()
            self.pass_counter = self.pass_steps
            self.last_known_radius = 0.0
            self.get_logger().info(
                f"{target_color} donut: close enough (r={r:.1f}), starting pass-through."
            )
            return
        
        #Compute image-plane errors and desired corrections
        error_x = cx - self.img_center_x
        error_y = cy - self.img_center_y
        step_yaw = -self.pid_yaw.update(error_x)
        step_y = 0.0
        step_z = -self.pid_z.update(error_y)
        step_x_min = 0.08
        if r > 180:
            step_x_min = self.forward_step
            
        aligned = abs(error_x) < self.align_tol and abs(error_y) < self.align_tol

        #Alignment phase: hover in place, only turn and change height until the donut stays centered
        if not self.approaching:
            phase = "align"
            step_x = 0.0
            self.align_counter = self.align_counter + 1 if aligned else 0
            if self.align_counter >= self.align_frames_required:
                self.approaching = True
                self.get_logger().info(
                    f"Aligned with {target_color} torus, starting approach."
                )
            else:
                self.get_logger().info(
                    f"Aligning to {target_color} torus: dx={error_x:.0f}, dy={error_y:.0f} "
                    f"({self.align_counter}/{self.align_frames_required})"
                )

        #Approach phase: if the donut drifts too far from the center, stop and realign
        elif abs(error_x) > self.realign_tol or abs(error_y) > self.realign_tol:
            phase = "realign"
            self.reset_alignment()
            step_x = 0.0
            self.get_logger().info(f"Lost alignment (dx={error_x:.0f}, dy={error_y:.0f}). Stopping to realign.")

        #If well-aligned, go at max forward speed; otherwise take smaller corrective steps
        elif aligned:
            phase = "approach"
            step_x = self.max_step_x
            step_yaw = 0.0
            self.get_logger().info(f"Aligned. Moving towards {target_color} torus. Speed: {step_x:.2f} m/s (MAX SPEED)")
        else:
            phase = "approach"
            step_x = step_x_min
            self.get_logger().info(f"Alignment required: dx={error_x:.0f}, dy={error_y:.0f}. Correcting.")

        #Clamp command magnitudes to configured maxima
        step_x = float(np.clip(step_x, -self.max_step_x, self.max_step_x))
        step_y = float(np.clip(step_y, -self.max_step_y, self.max_step_y)) 
        step_z = float(np.clip(step_z, -self.max_step_z, self.max_step_z))
        step_yaw = float(np.clip(step_yaw, -0.15, 0.15)) 

        #Send the motion command to the controller (all axes in one step)
        self.controller.step(step_x, step_y, step_z, step_yaw)
        self.log_frame(target_color, phase, error_x, error_y, r, step_x, step_z, step_yaw)
        self.get_logger().info(
            f"Tracking {target_color}: dx={step_x:.2f}, yaw={step_yaw:.3f}, r={r:.1f}"
        )

#Entry point: initializes ROS 2, creates the node and starts the main loop
def main(args=None):
    rclpy.init(args=args)
    node = TelloMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()