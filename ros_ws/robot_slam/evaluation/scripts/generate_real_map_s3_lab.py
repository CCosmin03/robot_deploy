#!/usr/bin/env python3
"""
Genereaza harta Ground Truth pentru Scenariul 3 (Lab static / dinamic).

Peretele sud este inlocuit de conturul unificat:
  [dulap stang] -- bridge -- [canapea] -- bridge -- [dulap drept]
Conturul = 5 segmente negre; tot ce e la SUD de ele = gri uniform (necunoscut).

Valori PGM:
  254 = liber  |  0 = ocupat  |  205 = necunoscut

Utilizare:
  python3 generate_real_map_s3_lab.py --config scenario_config.yaml \
      --output ground_truth/real_map_s3_lab
"""

import argparse
import os
import yaml
import numpy as np


RESOLUTION = 0.05
WALL_M     = 0.10
MARGIN_M   = 0.30


def build_map(cfg, resolution=RESOLUTION, wall_m=WALL_M, margin_m=MARGIN_M):
    room_l    = float(cfg["room_length_m"])
    room_w    = float(cfg["room_width_m"])
    sofa_l    = float(cfg["sofa_length_m"])
    d_n       = float(cfg["dist_north_m"])   # y fata nord sofa de la perete N
    d_e       = float(cfg["dist_east_m"])    # x stang sofa de la marginea stanga imaginii
    wardrobes = cfg.get("wardrobes", []) or []
    radiator  = cfg.get("radiator")  or {}
    obstacles = cfg.get("obstacles", []) or []

    wall_px   = max(1, round(wall_m   / resolution))
    margin_px = max(1, round(margin_m / resolution))
    room_l_px = round(room_l / resolution)
    room_w_px = round(room_w / resolution)

    total_w = margin_px + wall_px + room_l_px + wall_px + margin_px
    total_h = margin_px + wall_px + room_w_px + wall_px + margin_px

    grid = np.full((total_h, total_w), 205, dtype=np.uint8)

    ox = margin_px + wall_px   # coloana stanga a interiorului
    oy = margin_px + wall_px   # randul de sus al interiorului

    # ── Pereti N, V, E — peretele S e inlocuit de mobilier ────────────────────
    grid[margin_px : margin_px + wall_px,
         margin_px : total_w - margin_px] = 0              # N
    grid[margin_px : total_h - margin_px,
         margin_px : margin_px + wall_px] = 0              # V
    grid[margin_px : total_h - margin_px,
         total_w - margin_px - wall_px : total_w - margin_px] = 0  # E

    # ── Interior alb ───────────────────────────────────────────────────────────
    grid[oy : oy + room_w_px, ox : ox + room_l_px] = 254

    # ── Calorifer — solid negru lipit de perete V ─────────────────────────────
    cal_l_m = float(radiator.get("length_m", 0))
    cal_w_m = float(radiator.get("width_m",  0))
    if cal_l_m > 0 and cal_w_m > 0:
        cal_wall = radiator.get("wall", "west")
        cal_d_n  = float(radiator.get("dist_from_north_m", 0))
        cal_l_px = round(cal_l_m / resolution)
        cal_w_px = max(1, round(cal_w_m / resolution))
        cal_r0   = oy + round(cal_d_n / resolution)
        cal_r1   = min(cal_r0 + cal_l_px, oy + room_w_px)
        if cal_wall == "west":
            grid[cal_r0:cal_r1, ox : min(ox + cal_w_px, ox + room_l_px)] = 0
        elif cal_wall == "east":
            grid[cal_r0:cal_r1,
                 max(ox, ox + room_l_px - cal_w_px) : ox + room_l_px] = 0

    # ── Conturul sudic unificat ────────────────────────────────────────────────
    #
    #  x:  ox ─────────── x_s ─────────────────── x_e ──────────── ox+room_l_px
    #       [  sect stanga  |       sofa            |   sect dreapta  ]
    #  y:       y_l ────────|                        |──────── y_r
    #                       |── y_c ─────────────────|
    #
    # Sect stanga = dulap stang + bridge stang (aceeasi inaltime: y_l)
    # Sect dreapta = bridge drept + dulap drept (aceeasi inaltime: y_r)

    x_s = ox + round(d_e / resolution)
    x_e = min(x_s + round(sofa_l / resolution), ox + room_l_px)
    y_c = oy + round(d_n / resolution)

    if len(wardrobes) >= 2:
        y_l = oy + round(float(wardrobes[0]["y_m"]) / resolution)
        y_r = oy + round(float(wardrobes[1]["y_m"]) / resolution)
    else:
        y_l = y_r = y_c

    # Fill: tot la SUD de granita = gri (overwrite white)
    grid[y_l : oy + room_w_px, ox : x_s]             = 205
    grid[y_c : oy + room_w_px, x_s       : x_e]      = 205
    grid[y_r : oy + room_w_px, x_e       : ox + room_l_px] = 205

    # Ascunde peretele V/E in spatele dulapurilor (inclusiv coltul sud)
    grid[y_l : total_h - margin_px, margin_px : margin_px + wall_px]                     = 205
    grid[y_r : total_h - margin_px, total_w - margin_px - wall_px : total_w - margin_px] = 205

    # Contur negru — 5 segmente (fara nicio linie la SUD de ele)
    grid[y_l,        ox        : x_s + 1]           = 0   # fata N sect stanga
    grid[y_c : y_l + 1, x_s]                        = 0   # fata V sofa (vertical)
    grid[y_c,        x_s       : x_e]               = 0   # fata N sofa
    grid[y_c : y_r + 1, x_e - 1]                    = 0   # fata E sofa (vertical)
    grid[y_r,        x_e - 1   : ox + room_l_px]    = 0   # fata N sect dreapta

    # ── Obstacole libere (cutii) — gri interior + contur negru ────────────────
    for i, obs in enumerate(obstacles):
        ol  = max(1, round(float(obs.get("length_m", 0)) / resolution))
        ow  = max(1, round(float(obs.get("width_m",  0)) / resolution))
        ox_ = ox + round(float(obs.get("x_m", 0)) / resolution)
        oy_ = oy + round(float(obs.get("y_m", 0)) / resolution)
        ox_end = min(ox_ + ol, ox + room_l_px)
        oy_end = min(oy_ + ow, oy + room_w_px)
        if ox_end <= ox_ or oy_end <= oy_:
            print(f"[WARN] Obstacol '{obs.get('name', i+1)}' in afara camerei")
            continue
        grid[oy_        : oy_end, ox_        : ox_end] = 205
        grid[oy_,         ox_     : ox_end]             = 0
        grid[oy_end - 1,  ox_     : ox_end]             = 0
        grid[oy_        : oy_end, ox_]                  = 0
        grid[oy_        : oy_end, ox_end - 1]           = 0
        print(f"  Obstacol '{obs.get('name', i+1)}': {ol}x{ow} px la ({ox_-ox},{oy_-oy}) px")

    origin_x = -(ox * resolution)
    origin_y = -(oy * resolution)
    return grid, resolution, origin_x, origin_y


def save_pgm(grid, path):
    h, w = grid.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(grid.tobytes())
    print(f"  PGM salvat: {path}  ({w}x{h} px)")


def save_yaml(path, pgm_name, resolution, origin_x, origin_y):
    data = {
        "image":           pgm_name,
        "resolution":      float(resolution),
        "origin":          [float(origin_x), float(origin_y), 0.0],
        "negate":          0,
        "occupied_thresh": 0.65,
        "free_thresh":     0.196,
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"  YAML salvat: {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config",     required=True)
    parser.add_argument("--output",     required=True)
    parser.add_argument("--resolution", type=float, default=RESOLUTION)
    parser.add_argument("--wall",       type=float, default=WALL_M)
    parser.add_argument("--margin",     type=float, default=MARGIN_M)
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    cfg_s3 = cfg.get("s3a")
    if cfg_s3 is None:
        raise ValueError("Sectiunea 's3a' lipseste din scenario_config.yaml")

    grid, res, ox, oy = build_map(cfg_s3, args.resolution, args.wall, args.margin)

    pgm_path  = args.output + ".pgm"
    yaml_path = args.output + ".yaml"
    os.makedirs(os.path.dirname(pgm_path) or ".", exist_ok=True)
    save_pgm(grid, pgm_path)
    save_yaml(yaml_path, os.path.basename(pgm_path), res, ox, oy)
    print(f"[GT S3] Harta generata: {pgm_path}")


if __name__ == "__main__":
    main()
