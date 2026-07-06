#!/usr/bin/env python3
"""
Genereaza harta Ground Truth pentru Scenariul 2 (Loop Closure - Canapea).

Harta contine:
  - peretii camerei (dreptunghiulara)
  - canapeaua (obstacol rectangular interior)
  - dulapuri optionale (obstacole rectangulare pe pereti)

Valori PGM:
  254 = liber (spatiu navigabil)
    0 = ocupat (perete / canapea / dulap)
  205 = necunoscut (border/margine)

Utilizare:
  python3 generate_real_map_s2_sofa.py --config scenario_config.yaml \
      --output ground_truth/real_map_s2_sofa
"""

import argparse
import os
import struct
import yaml
import numpy as np


RESOLUTION  = 0.05   # m/pixel
WALL_M      = 0.10   # grosimea peretilor in metri
MARGIN_M    = 0.30   # margine gri (necunoscut) in jurul camerei


def build_map(cfg_s2, resolution=RESOLUTION, wall_m=WALL_M, margin_m=MARGIN_M):
    room_l = float(cfg_s2["room_length_m"])
    room_w = float(cfg_s2["room_width_m"])
    sofa_l = float(cfg_s2["sofa_length_m"])
    sofa_w = float(cfg_s2["sofa_width_m"])
    d_n    = float(cfg_s2["dist_north_m"])
    d_s    = float(cfg_s2["dist_south_m"])
    d_e    = float(cfg_s2["dist_east_m"])
    d_w    = float(cfg_s2["dist_west_m"])
    wardrobes = cfg_s2.get("wardrobes", []) or []
    radiator  = cfg_s2.get("radiator") or {}

    if room_l <= 0 or room_w <= 0:
        raise ValueError("Completeaza room_length_m si room_width_m in scenario_config.yaml")

    wall_px   = max(1, round(wall_m   / resolution))
    margin_px = max(1, round(margin_m / resolution))

    # Dimensiuni totale in pixeli (margine + perete + spatiu liber + perete + margine)
    room_l_px = round(room_l / resolution)
    room_w_px = round(room_w / resolution)

    total_w = margin_px + wall_px + room_l_px + wall_px + margin_px  # coloane (X)
    total_h = margin_px + wall_px + room_w_px + wall_px + margin_px  # randuri  (Y)

    grid = np.full((total_h, total_w), 205, dtype=np.uint8)  # tot gri (necunoscut)

    # Offset la coltul interior NV al camerei (primul pixel liber)
    ox = margin_px + wall_px   # coloana stanga a spatiului liber
    oy = margin_px + wall_px   # randul de sus al spatiului liber

    # Pereti
    grid[margin_px : margin_px + wall_px,           margin_px : total_w - margin_px] = 0   # N
    grid[total_h - margin_px - wall_px : total_h - margin_px,
         margin_px : total_w - margin_px] = 0                                               # S
    grid[margin_px : total_h - margin_px, margin_px : margin_px + wall_px]           = 0   # V
    grid[margin_px : total_h - margin_px, total_w - margin_px - wall_px : total_w - margin_px] = 0  # E

    # Spatiu liber interior
    grid[oy : oy + room_w_px, ox : ox + room_l_px] = 254

    # Canapea — pozitionata dupa distantele D1-D4
    if sofa_l > 0 and sofa_w > 0:
        sofa_l_px = round(sofa_l / resolution)
        sofa_w_px = round(sofa_w / resolution)
        d_w_px    = round(d_w / resolution)
        d_n_px    = round(d_n / resolution)

        # Originea canapelei in grid (colt NV al canapelei)
        # Folosim dist_east pentru stanga imaginii (orientare corecta)
        d_e_px  = round(d_e / resolution)
        sofa_ox = ox + d_e_px
        sofa_oy = oy + d_n_px

        if (sofa_ox + sofa_l_px <= ox + room_l_px and
                sofa_oy + sofa_w_px <= oy + room_w_px):
            # Interior gri (necunoscut) + contur negru — ca in hartile SLAM reale
            grid[sofa_oy:sofa_oy+sofa_w_px, sofa_ox:sofa_ox+sofa_l_px] = 205
            grid[sofa_oy,                   sofa_ox:sofa_ox+sofa_l_px]  = 0  # nord
            grid[sofa_oy+sofa_w_px-1,       sofa_ox:sofa_ox+sofa_l_px]  = 0  # sud
            grid[sofa_oy:sofa_oy+sofa_w_px, sofa_ox]                    = 0  # vest
            grid[sofa_oy:sofa_oy+sofa_w_px, sofa_ox+sofa_l_px-1]       = 0  # est
        else:
            print("[WARN] Canapeaua depaseste limitele camerei — verifica D1-D4 si S1-S2")

    # Dulapuri — colturi interioare: interior gri, doar fetele vizibile din camera = negre
    for i, dub in enumerate(wardrobes):
        dl = round(float(dub.get("length_m", 0)) / resolution)
        dw = round(float(dub.get("width_m",  0)) / resolution)
        dx = round(float(dub.get("x_m",      0)) / resolution)
        dy = round(float(dub.get("y_m",      0)) / resolution)
        if dl <= 0 or dw <= 0:
            continue
        d_ox = ox + dx
        d_oy = oy + dy
        d_ox_end = min(d_ox + dl, ox + room_l_px)
        d_oy_end = min(d_oy + dw, oy + room_w_px)
        if d_ox_end <= d_ox or d_oy_end <= d_oy or d_ox >= ox + room_l_px:
            print(f"[WARN] Dulap {i+1} complet in afara camerei — verificati x_m/y_m")
            continue
        # Interior necunoscut
        grid[d_oy:d_oy_end, d_ox:d_ox_end] = 205
        # Detectie pozitie colt
        at_west  = d_ox     <= ox
        at_east  = d_ox_end >= ox + room_l_px
        at_north = d_oy     <= oy
        at_south = d_oy_end >= oy + room_w_px
        # Sterge peretii ascunsi in spatele dulsapului
        c_lo_w = (d_ox - wall_px)     if at_west  else d_ox
        c_hi_w = (d_ox_end + wall_px) if at_east  else d_ox_end
        if at_south: grid[d_oy_end:d_oy_end+wall_px, c_lo_w:c_hi_w]          = 205
        if at_north: grid[d_oy-wall_px:d_oy,         c_lo_w:c_hi_w]          = 205
        if at_west:  grid[d_oy:d_oy_end, d_ox-wall_px:d_ox]                  = 205
        if at_east:  grid[d_oy:d_oy_end, d_ox_end:d_ox_end+wall_px]          = 205
        # Fetele vizibile — 1px
        if not at_north: grid[d_oy,       d_ox:d_ox_end] = 0
        if not at_south: grid[d_oy_end-1, d_ox:d_ox_end] = 0
        if not at_west:  grid[d_oy:d_oy_end, d_ox]       = 0
        if not at_east:  grid[d_oy:d_oy_end, d_ox_end-1] = 0

    # Calorifere — obiect solid lipit de perete
    cal_l_m = float(radiator.get("length_m", 0))
    cal_w_m = float(radiator.get("width_m",  0))
    if cal_l_m > 0 and cal_w_m > 0:
        cal_wall  = radiator.get("wall", "west")
        cal_d_n   = float(radiator.get("dist_from_north_m", 0))
        cal_l_px  = round(cal_l_m / resolution)
        cal_w_px  = max(1, round(cal_w_m / resolution))
        cal_r0    = oy + round(cal_d_n / resolution)
        cal_r1    = min(cal_r0 + cal_l_px, oy + room_w_px)
        if cal_wall == "west":
            grid[cal_r0:cal_r1, ox : min(ox + cal_w_px, ox + room_l_px)] = 0
        elif cal_wall == "east":
            grid[cal_r0:cal_r1, max(ox, ox + room_l_px - cal_w_px) : ox + room_l_px] = 0
        elif cal_wall == "north":
            cal_c0 = oy + round(cal_d_n / resolution)   # refolosim d_n ca dist_from_west
            grid[oy : min(oy + cal_w_px, oy + room_w_px), cal_c0 : min(cal_c0 + cal_l_px, ox + room_l_px)] = 0

    # Origine reala in metri (coltul NV al peretilor, in coord ROS)
    origin_x = -(ox * resolution)
    origin_y = -(oy * resolution)   # Y inversat (ROS: Y sus = pozitiv)

    return grid, resolution, origin_x, origin_y


def save_pgm(grid, path):
    h, w = grid.shape
    header = f"P5\n{w} {h}\n255\n".encode()
    with open(path, "wb") as f:
        f.write(header)
        f.write(grid.tobytes())
    print(f"  PGM salvat: {path}  ({w}x{h} px)")


def save_yaml(path, pgm_name, resolution, origin_x, origin_y):
    data = {
        "image":      pgm_name,
        "resolution": float(resolution),
        "origin":     [float(origin_x), float(origin_y), 0.0],
        "negate":     0,
        "occupied_thresh": 0.65,
        "free_thresh":     0.196,
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"  YAML salvat: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Genereaza harta GT pentru S2 (camera + canapea + dulapuri)."
    )
    parser.add_argument("--config", required=True,
                        help="scenario_config.yaml cu dimensiunile fizice S2")
    parser.add_argument("--output", required=True,
                        help="Calea de baza output (fara extensie), ex: ground_truth/real_map_s2_sofa")
    parser.add_argument("--resolution", type=float, default=RESOLUTION,
                        help=f"Rezolutie m/px (default {RESOLUTION})")
    parser.add_argument("--wall",       type=float, default=WALL_M,
                        help=f"Grosimea peretilor m (default {WALL_M})")
    parser.add_argument("--margin",     type=float, default=MARGIN_M,
                        help=f"Margine necunoscuta m (default {MARGIN_M})")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    cfg_s2 = cfg.get("s2")
    if cfg_s2 is None:
        raise ValueError("Sectiunea 's2' lipseste din scenario_config.yaml")

    grid, res, ox, oy = build_map(cfg_s2, args.resolution, args.wall, args.margin)

    pgm_path  = args.output + ".pgm"
    yaml_path = args.output + ".yaml"
    pgm_name  = os.path.basename(pgm_path)

    os.makedirs(os.path.dirname(pgm_path) or ".", exist_ok=True)
    save_pgm(grid, pgm_path)
    save_yaml(yaml_path, pgm_name, res, ox, oy)
    print(f"[GT S2] Harta generata: {pgm_path}")


if __name__ == "__main__":
    main()
