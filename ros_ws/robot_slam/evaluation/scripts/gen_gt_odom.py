#!/usr/bin/env python3
"""
Extrage traiectoria /odom din rosbag ca pseudo-GT in format TUM.
Folosita pentru ATE/RPE pe trasee neregulate (S2 cu mobilier, S3).

ATE fata de aceasta referinta masoara cat corecteaza/deviaza
fiecare algoritm SLAM fata de odometria bruta a encoderelor.
Comparatia intre cele 6 configuratii ramane valida chiar daca
odometria nu e perfecta — toti sunt comparati cu acelasi baseline.

Utilizare:
  python3 gen_gt_odom.py --bag bags/s2/run1.bag --output ground_truth/s2_gt.tum
"""

import argparse
import math

import rosbag


def extract_odom_tum(bag_path, output_path, topic="/odom"):
    poses = []

    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            ts = t.to_sec()
            p  = msg.pose.pose.position
            q  = msg.pose.pose.orientation
            poses.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    if not poses:
        raise RuntimeError(f"Niciun mesaj pe {topic} in {bag_path}")

    x0, y0 = poses[0][1], poses[0][2]

    with open(output_path, "w") as f:
        f.write(f"# Pseudo-GT din {topic} — bag: {bag_path}\n")
        f.write("# format: timestamp tx ty tz qx qy qz qw\n")
        for ts, x, y, z, qx, qy, qz, qw in poses:
            f.write(f"{ts:.6f} {x-x0:.6f} {y-y0:.6f} {z:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

    dx = poses[-1][1] - poses[0][1]
    dy = poses[-1][2] - poses[0][2]
    loop_err = math.sqrt(dx*dx + dy*dy)
    duration = poses[-1][0] - poses[0][0]

    print(f"Salvat {len(poses)} pose -> {output_path}")
    print(f"  Durata:             {duration:.1f}s")
    print(f"  Loop closure /odom: {loop_err:.4f}m  (eroarea bazei de referinta)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag",    required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic",  default="/odom")
    args = parser.parse_args()
    extract_odom_tum(args.bag, args.output, args.topic)


if __name__ == "__main__":
    main()
