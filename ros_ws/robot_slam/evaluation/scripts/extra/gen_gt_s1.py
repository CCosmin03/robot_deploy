#!/usr/bin/env python3
"""
Genereaza ground truth TUM pentru Scenariul 1 (coridor drept 5m dus-intors).
Extrage timingul din /odom din bag si construieste traiectoria linie dreapta.

Utilizare:
  python3 gen_gt_s1.py --bag ../bags/s1/run1.bag --length 5.0 --output s1_gt.tum
"""

import argparse
import math

import rosbag
import yaml


def gen_gt_s1(bag_path, length_m, output_path):
    ts_list = []
    x_odom  = []

    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=["/odom"]):
            ts_list.append(t.to_sec())
            x_odom.append(msg.pose.pose.position.x)

    if not ts_list:
        raise RuntimeError("Nu s-au gasit mesaje /odom in bag")

    t0 = ts_list[0]
    x0 = x_odom[0]
    x_max = max(abs(x - x0) for x in x_odom)

    if x_max < 0.1:
        raise RuntimeError("Robotul nu pare sa se fi miscat (x_max < 0.1m)")

    scale = length_m / x_max

    with open(output_path, "w") as f:
        f.write("# Ground truth S1: coridor drept {:.1f}m\n".format(length_m))
        f.write("# format: timestamp tx ty tz qx qy qz qw\n")
        for ts, x in zip(ts_list, x_odom):
            gx = (x - x0) * scale
            f.write(f"{ts:.6f} {gx:.6f} 0.000000 0.000000 "
                    f"0.000000 0.000000 0.000000 1.000000\n")

    print(f"Salvat {len(ts_list)} pose in {output_path}")
    print(f"  Interval timp: {ts_list[0]:.1f}s - {ts_list[-1]:.1f}s")
    print(f"  Scala aplicata: {scale:.3f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag",    required=True)
    parser.add_argument("--length", type=float, default=5.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    gen_gt_s1(args.bag, args.length, args.output)


if __name__ == "__main__":
    main()
