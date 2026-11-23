#!/usr/bin/env python3
#Puts the drone into random poses and saves one fresh camera image per pose together with the exact pose
import json, math, os, random, sys, time
import cv2
import rclpy
import transforms3d.euler as tfe
from rclpy.node import Node
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from cv_bridge import CvBridge

RINGS = {"blue": (8.77423, -0.109163, 1.88071), "green": (6.79468, 3.09263, 1.36946), "red": (7.79312, -3.54357, 3.52753)}

class Capture(Node):
    def __init__(self):
        super().__init__("capture")
        self.bridge = CvBridge()
        self.sim_time = 0.0
        self.frames = 0
        self.last = None
        self.create_subscription(Clock, "/clock", self.cb_clock, 10)
        self.create_subscription(Image, "/topic_ns/image_raw", self.cb_image, 10)
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.cli.wait_for_service()

    def cb_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def cb_image(self, msg):
        self.frames += 1
        self.last = msg

    def set_pose(self, x, y, z, yaw):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "tello_drone"
        req.state.reference_frame = "world"
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        q = tfe.euler2quat(0.0, 0.0, yaw, axes="sxyz")
        req.state.pose.orientation.w, req.state.pose.orientation.x = q[0], q[1]
        req.state.pose.orientation.y, req.state.pose.orientation.z = q[2], q[3]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result() is not None and fut.result().success

    #Waits for an image rendered clearly after the teleport (sim time stamp + several new frames)
    def fresh_image(self, min_frames=4, settle=0.25, timeout=15.0):
        t_set = self.sim_time
        start_frames = self.frames
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last is None:
                continue
            stamp = self.last.header.stamp.sec + self.last.header.stamp.nanosec * 1e-9
            if self.frames - start_frames >= min_frames and stamp >= t_set + settle:
                return self.last
        return None

def sample_pose(rng):
    if rng.random() < 0.7:
        #Aimed at a ring: from either side, off-axis, with some heading error
        name = rng.choice(list(RINGS))
        rx, ry, rz = RINGS[name]
        side = -1.0 if rng.random() < 0.75 else 1.0
        d = rng.uniform(1.5, 12.0)
        phi = math.radians(rng.uniform(-55, 55))
        x = rx + side * d * math.cos(phi)
        y = ry + d * math.sin(phi)
        z = min(5.0, max(0.3, rz + rng.uniform(-1.5, 1.5)))
        yaw = math.atan2(ry - y, rx - x) + rng.gauss(0.0, 0.3)
        kind = "aimed_" + name
    else:
        x, y = rng.uniform(-3, 22), rng.uniform(-9, 9)
        z = rng.uniform(0.3, 5.0)
        yaw = rng.uniform(-math.pi, math.pi)
        kind = "random"
    return x, y, z, yaw, kind

def main():
    out, n, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    os.makedirs(out, exist_ok=True)
    rng = random.Random(seed)
    rclpy.init()
    node = Capture()
    node.fresh_image(min_frames=2, settle=0.0)
    meta = []
    i = 0
    while i < n:
        x, y, z, yaw, kind = sample_pose(rng)
        if any(math.dist((x, y, z), p) < 1.2 for p in RINGS.values()):
            continue
        if not node.set_pose(x, y, z, yaw):
            print("set_pose failed", flush=True)
            continue
        msg = node.fresh_image()
        if msg is None:
            print("no fresh image", flush=True)
            continue
        img = node.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        name = f"img_{i:04d}.png"
        cv2.imwrite(os.path.join(out, name), img)
        meta.append({"file": name, "x": x, "y": y, "z": z, "yaw": yaw, "kind": kind})
        i += 1
        if i % 25 == 0:
            print(f"{i}/{n}", flush=True)
    json.dump(meta, open(os.path.join(out, "poses.json"), "w"), indent=1)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
