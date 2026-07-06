#!/usr/bin/env python3
"""
Masoara dimensiunile canapelei si camerei pe hartile SLAM generate.
Compara cu masuratorile reale si calculeaza eroarea procentuala.

Utilizare:
  python3 measure_map.py --results-dir evaluation/results --scenario s2
"""

import argparse
import os
import cv2
import numpy as np
import yaml

REAL_SOFA_L   = 1.50   # m — lungimea canapelei
REAL_SOFA_W   = 1.00   # m — latimea canapelei
REAL_ROOM_W   = 2.85   # m — latimea camerei
REAL_ROOM_L   = 3.40   # m — lungimea camerei

CONFIGS = ["c1", "c2", "c3", "c4", "c5", "c6"]
LABELS  = {
    "c1": "GMap enc",
    "c2": "GMap+EKF",
    "c3": "Hector",
    "c4": "Carto enc",
    "c5": "Carto+EKF",
    "c6": "Carto IMU",
}


def load_map(pgm_path, yaml_path):
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    return img, float(meta["resolution"])


def find_sofa(img, resolution):
    """
    Detecteaza canapeaua ca cel mai mare obiect izolat (nu atinge bordura hartii).
    Returneaza dimensiunile (L, W) in metri sau (None, None) daca nu gaseste.
    """
    occupied = (img < 50).astype(np.uint8) * 255

    kernel = np.ones((3, 3), np.uint8)
    occupied = cv2.morphologyEx(occupied, cv2.MORPH_CLOSE, kernel, iterations=2)
    occupied = cv2.morphologyEx(occupied, cv2.MORPH_OPEN,  kernel, iterations=1)

    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)

    h, w = img.shape
    sofa_area_px = (REAL_SOFA_L * REAL_SOFA_W) / (resolution ** 2)

    best = None
    best_score = float("inf")

    for i in range(1, n_labels):
        x, y, bw, bh, area = stats[i]

        # Exclude componente care ating marginea hartii (sunt pereti)
        if x <= 2 or y <= 2 or (x + bw) >= (w - 2) or (y + bh) >= (h - 2):
            continue

        # Filtru dupa arie — accept intre 25% si 400% din aria asteptata
        if area < sofa_area_px * 0.25 or area > sofa_area_px * 4.0:
            continue

        mask = (labels == i).astype(np.uint8)
        contour, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contour:
            continue

        rect = cv2.minAreaRect(contour[0])
        pw, ph = rect[1]
        if pw == 0 or ph == 0:
            continue

        dim_l = max(pw, ph) * resolution
        dim_w = min(pw, ph) * resolution

        score = abs(dim_l - REAL_SOFA_L) + abs(dim_w - REAL_SOFA_W)
        if score < best_score:
            best_score = score
            best = (dim_l, dim_w)

    return best


def measure_room(img, resolution):
    """
    Masoara dimensiunile spatiului liber din harta (aproximare camera).
    """
    free = (img > 200).astype(np.uint8) * 255
    coords = np.argwhere(free > 0)
    if len(coords) == 0:
        return None, None
    h_px = coords[:, 0].max() - coords[:, 0].min()
    w_px = coords[:, 1].max() - coords[:, 1].min()
    return w_px * resolution, h_px * resolution


def pct_err(real, measured):
    return abs(real - measured) / real * 100.0


def find_map(results_dir, config, scenario):
    """Gaseste fisierul de harta dintr-un run (primul disponibil)."""
    for run in range(1, 6):
        folder = os.path.join(results_dir, f"{config}_{scenario}_r{run}")
        pgm = os.path.join(folder, "map.pgm")
        yml = os.path.join(folder, "map.yaml")
        if os.path.exists(pgm) and os.path.exists(yml):
            return pgm, yml
    return None, None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default="evaluation/results")
    parser.add_argument("--scenario",    default="s2")
    args = parser.parse_args()

    print(f"\n{'='*70}")
    print(f"  MASURATORI HARTA — Scenariu {args.scenario.upper()}")
    print(f"  Canapea reala: {REAL_SOFA_L}m x {REAL_SOFA_W}m")
    print(f"  Camera reala:  {REAL_ROOM_W}m x {REAL_ROOM_L}m")
    print(f"{'='*70}")
    print(f"{'Config':<12} {'Sofa L(m)':<10} {'Sofa W(m)':<10} "
          f"{'Err L%':<8} {'Err W%':<8} {'Err med%':<10} {'Camera W':<10} {'Camera L'}")
    print("-" * 90)

    for cfg in CONFIGS:
        pgm, yml = find_map(args.results_dir, cfg, args.scenario)
        if pgm is None:
            print(f"{LABELS[cfg]:<12} {'—'}")
            continue

        img, res = load_map(pgm, yml)
        sofa = find_sofa(img, res)
        room = measure_room(img, res)

        if sofa is None:
            sofa_str = f"{'nu gasit':<10} {'—':<10} {'—':<8} {'—':<8} {'—':<10}"
        else:
            sl, sw = sofa
            el = pct_err(REAL_SOFA_L, sl)
            ew = pct_err(REAL_SOFA_W, sw)
            em = (el + ew) / 2
            sofa_str = f"{sl:<10.3f} {sw:<10.3f} {el:<8.2f} {ew:<8.2f} {em:<10.2f}"

        if room[0] is None:
            room_str = "— —"
        else:
            rw, rl = room
            room_str = f"{rw:.2f}m x {rl:.2f}m"

        print(f"{LABELS[cfg]:<12} {sofa_str} {room_str}")

    print("=" * 70)


if __name__ == "__main__":
    main()
