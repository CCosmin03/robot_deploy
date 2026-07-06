#!/usr/bin/env python3
"""
Genereaza harta ground truth geometrica pentru Scenariul 1 (coridor drept).
Produce fisierele <output>.pgm si <output>.yaml.

Dependinte: pip3 install numpy pyyaml

Utilizare:
  python3 generate_corridor_map.py \
      --length 2.82 --width 0.90 \
      --output evaluation/ground_truth/real_map_s1

Formatul PGM generat este compatibil ROS map_server:
  254 = liber, 0 = ocupat (perete), 205 = necunoscut
"""

import argparse
import os

import numpy as np
import yaml


def generate_corridor_map(length_m, width_m,
                          resolution=0.05,
                          wall_m=0.10,
                          margin_m=0.30,
                          output_base="real_map_s1"):

    wall_px   = max(1, round(wall_m   / resolution))
    margin_px = max(1, round(margin_m / resolution))

    corr_len_px = round(length_m / resolution)
    corr_wid_px = round(width_m  / resolution)

    total_w = corr_len_px + 2 * margin_px
    total_h = corr_wid_px + 2 * (wall_px + margin_px)

    # 205 = necunoscut (gri ROS)
    grid = np.full((total_h, total_w), 205, dtype=np.uint8)

    # Zona libera
    x0 = margin_px
    x1 = x0 + corr_len_px
    y0 = margin_px + wall_px
    y1 = y0 + corr_wid_px
    grid[y0:y1, x0:x1] = 254

    # Perete sus si jos
    grid[y0 - wall_px : y0, x0 : x1] = 0
    grid[y1 : y1 + wall_px, x0 : x1] = 0

    # Perete stanga si dreapta (inchide coridorul)
    grid[y0 - wall_px : y1 + wall_px, x0 - wall_px : x0] = 0
    grid[y0 - wall_px : y1 + wall_px, x1 : x1 + wall_px] = 0

    pgm_path  = output_base + ".pgm"
    yaml_path = output_base + ".yaml"

    with open(pgm_path, "wb") as f:
        header = (
            f"P5\n"
            f"# Corridor GT: L={length_m}m W={width_m}m res={resolution}m/px\n"
            f"{total_w} {total_h}\n"
            f"255\n"
        )
        f.write(header.encode())
        f.write(grid.tobytes())

    # Originea: coltul stanga-jos al imaginii in coordonate world ROS
    # Robotul porneste de la (0,0), in centrul intrarii coridorului
    origin_x = -margin_m
    origin_y = -(width_m / 2.0 + wall_m + margin_m)

    meta = {
        "image":           os.path.basename(pgm_path),
        "resolution":      resolution,
        "origin":          [round(origin_x, 4), round(origin_y, 4), 0.0],
        "negate":          0,
        "occupied_thresh": 0.65,
        "free_thresh":     0.196,
    }
    with open(yaml_path, "w") as f:
        yaml.dump(meta, f, default_flow_style=False)

    print(f"Harta GT salvata:")
    print(f"  PGM  : {pgm_path}  ({total_w}x{total_h} px)")
    print(f"  YAML : {yaml_path}")
    print(f"  Coridor liber : {corr_len_px} x {corr_wid_px} px = {length_m}m x {width_m}m")
    print(f"  Rezolutie     : {resolution} m/px")
    print(f"  Origine world : ({origin_x:.4f}, {origin_y:.4f})")

    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(
        description="Genereaza harta PGM ground truth pentru coridor drept (S1)."
    )
    parser.add_argument("--length",     type=float, required=True,
                        help="Lungime coridor masurata fizic (m)")
    parser.add_argument("--width",      type=float, required=True,
                        help="Latime coridor masurata fizic (m)")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Rezolutie harta (m/px), default 0.05")
    parser.add_argument("--wall",       type=float, default=0.10,
                        help="Grosime perete in harta (m), default 0.10")
    parser.add_argument("--margin",     type=float, default=0.30,
                        help="Margine libera in jurul coridorului (m), default 0.30")
    parser.add_argument("--output",     required=True,
                        help="Baza fisier output (fara extensie), ex: ground_truth/real_map_s1")
    args = parser.parse_args()

    generate_corridor_map(
        length_m   = args.length,
        width_m    = args.width,
        resolution = args.resolution,
        wall_m     = args.wall,
        margin_m   = args.margin,
        output_base= args.output,
    )


if __name__ == "__main__":
    main()
