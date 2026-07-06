#!/usr/bin/env python3
"""
Genereaza ground truth TUM pentru Scenariul 2 (loop rectangular 4m x 3m).
Distribuie pozitiile pe perimetrul dreptunghiului proportional cu distanta parcursa.

Utilizare:
  python3 gen_gt_s2.py --bag ../bags/s2/run1.bag --width 4.0 --height 3.0 --output s2_gt.tum
"""

import argparse
import math

import rosbag
import numpy as np


CORNERS_CCW = [
    (0.0, 0.0),
    (4.0, 0.0),
    (4.0, 3.0),
    (0.0, 3.0),
    (0.0, 0.0),
]


def interp_rectangle(t_norm, width, height):
    """t_norm in [0,1] -> (x, y) pe perimetrul dreptunghiului."""
    corners = [
        (0.0,    0.0),
        (width,  0.0),
        (width,  height),
        (0.0,    height),
        (0.0,    0.0),
    ]
    perimeter = 2 * (width + height)
    segments = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[i + 1]
        seg_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        segments.append((seg_len / perimeter, x1, y1, x2, y2))

    cumsum = 0.0
    for frac, x1, y1, x2, y2 in segments:
        if t_norm <= cumsum + frac or frac < 1e-9:
            alpha = (t_norm - cumsum) / frac if frac > 0 else 0.0
            alpha = max(0.0, min(1.0, alpha))
            return x1 + alpha * (x2 - x1), y1 + alpha * (y2 - y1)
        cumsum += frac
    return corners[-1]


def yaw_at_t(t_norm, width, height):
    """Unghi de inaintare pe dreptunghi (quaternion z, w)."""
    perimeter = 2 * (width + height)
    dist = t_norm * perimeter
    sides = [width, height, width, height]
    angles = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
    cumsum = 0.0
    for side, angle in zip(sides, angles):
        if dist <= cumsum + side:
            yaw = angle
            return math.sin(yaw / 2), math.cos(yaw / 2)
        cumsum += side
    return 0.0, 1.0


def gen_gt_s2(bag_path, width, height, output_path):
    ts_list = []
    d_list  = []

    prev_x, prev_y = None, None
    total_dist = 0.0

    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=["/odom"]):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if prev_x is not None:
                total_dist += math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
            ts_list.append(t.to_sec())
            d_list.append(total_dist)
            prev_x, prev_y = x, y

    if not ts_list:
        raise RuntimeError("Nu s-au gasit mesaje /odom in bag")

    perimeter = 2 * (width + height)
    print(f"Distanta parcursa in bag: {total_dist:.2f}m | Perimetru GT: {perimeter:.1f}m")

    with open(output_path, "w") as f:
        f.write(f"# Ground truth S2: dreptunghi {width}m x {height}m\n")
        f.write("# format: timestamp tx ty tz qx qy qz qw\n")
        for ts, d in zip(ts_list, d_list):
            t_norm = min(d / perimeter, 1.0) if total_dist > 0 else 0.0
            gx, gy = interp_rectangle(t_norm, width, height)
            qz, qw = yaw_at_t(t_norm, width, height)
            f.write(f"{ts:.6f} {gx:.6f} {gy:.6f} 0.000000 "
                    f"0.000000 0.000000 {qz:.6f} {qw:.6f}\n")

    print(f"Salvat {len(ts_list)} pose in {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag",    required=True)
    parser.add_argument("--width",  type=float, default=4.0)
    parser.add_argument("--height", type=float, default=3.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    gen_gt_s2(args.bag, args.width, args.height, args.output)


if __name__ == "__main__":
    main()
