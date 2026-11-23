#!/usr/bin/env python3
#Ground truth: projects the ring centers into the camera image from the known drone pose
import math

W, H = 960, 720
HFOV = 0.96
F = (W / 2) / math.tan(HFOV / 2)
CAM_OFFSET = 0.035  #camera_joint origin in the URDF (forward from base_link)
#Ring center and outer radius (mesh outer radius 0.74 m times the visual scale in the world file)
RINGS = {
    "blue": ((8.77423, -0.109163, 1.88071), 0.74 * 1.63629),
    "green": ((6.79468, 3.09263, 1.36946), 0.74 * 1.24926),
    "red": ((7.79312, -3.54357, 3.52753), 0.74 * 1.15117),
}

def project(pose):
    cy, sy = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    cx_w = pose["x"] + CAM_OFFSET * cy
    cy_w = pose["y"] + CAM_OFFSET * sy
    cz_w = pose["z"]
    result = {}
    for name, ((rx, ry, rz), radius) in RINGS.items():
        dx, dy, dz = rx - cx_w, ry - cy_w, rz - cz_w
        #World -> camera frame (x forward, y left, z up)
        xc = cy * dx + sy * dy
        yc = -sy * dx + cy * dy
        zc = dz
        if xc < 0.3:
            result[name] = {"status": "behind"}
            continue
        u = W / 2 - F * yc / xc
        v = H / 2 - F * zc / xc
        r = F * radius / xc
        inside = 0 <= u < W and 0 <= v < H
        fully = r <= u <= W - r and r <= v <= H - r
        touches = -r < u < W + r and -r < v < H + r
        status = "full" if fully else ("partial" if inside else ("edge" if touches else "out"))
        result[name] = {"status": status, "u": u, "v": v, "r": r, "depth": xc}
    return result
