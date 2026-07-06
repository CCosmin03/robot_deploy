#!/usr/bin/env python3
"""
Patch-uieste DOAR campul completeness_pct in metrics.yaml-urile existente
pentru s3a si s3b, fara a recalcula RPE / Hz / SSIM / IoU / CPU / RAM.

Aria de referinta = pixelii LIBERI din harta GT (real_map_s3_lab.pgm),
care exclude automat canapeaua, dulapurile si celelalte obstacole.

Rulare (din directorul evaluation/):
  python3 scripts/patch_completeness_s3.py
"""

import os
import yaml
import numpy as np

EVAL_DIR    = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
RESULTS_DIR = os.path.join(EVAL_DIR, "results")
GT_PGM      = os.path.join(EVAL_DIR, "ground_truth", "real_map_s3_lab.pgm")


def read_pgm(path):
    from PIL import Image
    return np.array(Image.open(path).convert("L"))


def navigable_area_from_gt(gt_pgm_path, resolution=0.05):
    """Numara pixelii liberi (>=250) din harta GT si returneaza aria in m²."""
    arr = read_pgm(gt_pgm_path)
    free_px = int(np.sum(arr >= 250))
    return free_px * resolution * resolution


def slam_free_area(map_yaml_path):
    """Aria libera din harta SLAM in m²."""
    with open(map_yaml_path) as f:
        info = yaml.safe_load(f)
    res     = float(info.get("resolution", 0.05))
    pgm     = os.path.join(os.path.dirname(map_yaml_path),
                           info.get("image", "map.pgm"))
    arr     = read_pgm(pgm)
    free_px = int(np.sum(arr >= 250))
    return free_px * res * res


def main():
    if not os.path.exists(GT_PGM):
        print(f"[EROARE] Harta GT nu exista: {GT_PGM}")
        print("  Ruleaza mai intai: python3 scripts/generate_real_map_s3_lab.py ...")
        raise SystemExit(1)

    real_area = navigable_area_from_gt(GT_PGM)
    print(f"[GT] Aria navigabila din harta GT: {real_area:.4f} m²")

    patched = 0
    skipped = 0

    for scenario in ("s3a", "s3b"):
        for entry in sorted(os.listdir(RESULTS_DIR)):
            if not entry.startswith("c") or f"_{scenario}_" not in entry:
                continue
            result_dir   = os.path.join(RESULTS_DIR, entry)
            metrics_path = os.path.join(result_dir, "metrics.yaml")
            map_yaml     = os.path.join(result_dir, "map.yaml")
            map_pgm      = os.path.join(result_dir, "map.pgm")

            if not os.path.exists(metrics_path):
                skipped += 1
                continue
            if not os.path.exists(map_yaml) or not os.path.exists(map_pgm):
                print(f"  [SKIP] {entry} — map.yaml / map.pgm lipsa")
                skipped += 1
                continue

            try:
                slam_area    = slam_free_area(map_yaml)
                completeness = round(min(slam_area / real_area * 100.0, 100.0), 2)
            except Exception as e:
                print(f"  [EROARE] {entry}: {e}")
                skipped += 1
                continue

            with open(metrics_path) as f:
                metrics = yaml.safe_load(f) or {}

            metrics["completeness_pct"] = completeness

            with open(metrics_path, "w") as f:
                yaml.dump(metrics, f, default_flow_style=False, allow_unicode=True)

            print(f"  [OK] {entry}  completeness={completeness}%")
            patched += 1

    print(f"\nGata: {patched} patched, {skipped} sarite.")
    print("\nRegenerare raport:")
    print(f"  python3 {SCRIPT_DIR}/generate_report.py "
          f"--results-dir {RESULTS_DIR} "
          f"--output {os.path.join(EVAL_DIR, 'reports', 'report.txt')} "
          f"--csv {os.path.join(EVAL_DIR, 'reports', 'report.csv')}")


if __name__ == "__main__":
    main()
