#!/usr/bin/env python3
"""
Compara harta SLAM cu harta ground truth geometrica (Scenariul 1 — coridor drept).
Calculeaza SSIM, IoU, Area Error si genereaza overlay vizual.

Dependinte: pip3 install scikit-image pillow numpy pyyaml

Utilizare standalone:
  python3 compare_corridor_maps.py \
      --gt-pgm  evaluation/ground_truth/real_map_s1.pgm \
      --gt-yaml evaluation/ground_truth/real_map_s1.yaml \
      --slam-pgm  results/c1_s1_r1/map.pgm \
      --slam-yaml results/c1_s1_r1/map.yaml \
      --output  results/c1_s1_r1/map_compare.yaml \
      --overlay results/c1_s1_r1/overlay.png

Overlay: verde=overlap, rosu=GT liber+SLAM nu, albastru=SLAM liber+GT nu
"""

import argparse
import os

import numpy as np
import yaml

FREE_THRESH = 250   # >= 250 = liber confirmat (ROS: 254=liber)
                    # 205=necunoscut este EXCLUS intentionat — altfel GMapping/Hector
                    # care aloca grid initial mare par sa aiba arie uriasa
WALL_THRESH = 50    # <= 50  = perete (ROS: 0=ocupat)


# ── Citire imagine ─────────────────────────────────────────────────────────────

def _read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Fisier PGM invalid (magic={magic}): {path}")
        while True:
            line = f.readline().strip()
            if not line.startswith(b"#"):
                break
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        dtype = np.uint16 if maxval > 255 else np.uint8
        data = np.frombuffer(f.read(), dtype=dtype).reshape(h, w)
        if dtype == np.uint16:
            data = (data >> 8).astype(np.uint8)
    return data


def load_map(pgm_path, yaml_path):
    """Returneaza (array uint8, resolution, (origin_x, origin_y))."""
    try:
        from PIL import Image
        arr = np.array(Image.open(pgm_path).convert("L"))
    except ImportError:
        arr = _read_pgm(pgm_path)

    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    res    = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return arr, res, (float(origin[0]), float(origin[1]))


# ── Masti binare ───────────────────────────────────────────────────────────────

def free_mask(arr):
    return (arr >= FREE_THRESH).astype(np.uint8)


def _crop_to_content(mask):
    """Crop la bounding box al pixelilor nonzero. Returneaza subarray."""
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any():
        return mask
    r0, r1 = np.where(rows)[0][[0, -1]]
    c0, c1 = np.where(cols)[0][[0, -1]]
    return mask[r0:r1 + 1, c0:c1 + 1]


def _resize_nearest(arr, target_h, target_w):
    """Resize cu nearest-neighbor (PIL daca e disponibil, altfel numpy)."""
    if arr.shape == (target_h, target_w):
        return arr
    try:
        from PIL import Image
        return np.array(
            Image.fromarray(arr).resize((target_w, target_h), Image.NEAREST)
        )
    except ImportError:
        row_idx = np.round(np.linspace(0, arr.shape[0] - 1, target_h)).astype(int)
        col_idx = np.round(np.linspace(0, arr.shape[1] - 1, target_w)).astype(int)
        return arr[np.ix_(row_idx, col_idx)]


# ── Metrici ────────────────────────────────────────────────────────────────────

def compute_ssim(gt_bin, slam_bin):
    """
    SSIM pe mastile binare ale spatiului liber.
    Fallback la coeficient de corelatie daca scikit-image nu e instalat.
    """
    try:
        from skimage.metrics import structural_similarity as ssim
        gt_f   = gt_bin.astype(float)
        slam_f = slam_bin.astype(float)
        score, _ = ssim(gt_f, slam_f, full=True, data_range=1.0)
        return round(float(score), 4)
    except ImportError:
        gt_f   = gt_bin.astype(float)
        slam_f = slam_bin.astype(float)
        mu_g, mu_s = gt_f.mean(), slam_f.mean()
        std_g = gt_f.std()
        std_s = slam_f.std()
        if std_g < 1e-10 or std_s < 1e-10:
            return 1.0 if np.allclose(gt_f, slam_f) else 0.0
        cov = ((gt_f - mu_g) * (slam_f - mu_s)).mean()
        c1, c2 = 0.01 ** 2, 0.03 ** 2
        ssim_val = ((2 * mu_g * mu_s + c1) * (2 * cov + c2)) / \
                   ((mu_g**2 + mu_s**2 + c1) * (std_g**2 + std_s**2 + c2))
        return round(float(np.clip(ssim_val, -1.0, 1.0)), 4)


def compute_iou(gt_bin, slam_bin):
    """IoU al regiunilor libere."""
    inter = np.logical_and(gt_bin > 0, slam_bin > 0).sum()
    union = np.logical_or(gt_bin  > 0, slam_bin > 0).sum()
    if union == 0:
        return 0.0
    return round(float(inter) / float(union), 4)


def compute_area_error(gt_arr, slam_arr, gt_res, slam_res):
    """
    Area Error (%): |aria_libera_GT - aria_libera_SLAM| / aria_libera_GT * 100.
    Calculat pe hartile originale (inainte de resize), in metri patrati.
    """
    gt_free_m2   = float(np.sum(gt_arr   >= FREE_THRESH)) * gt_res   * gt_res
    slam_free_m2 = float(np.sum(slam_arr >= FREE_THRESH)) * slam_res * slam_res
    if gt_free_m2 <= 0:
        return -1.0
    err = abs(gt_free_m2 - slam_free_m2) / gt_free_m2 * 100.0
    return round(err, 2)


# ── Overlay vizual ─────────────────────────────────────────────────────────────

def save_overlay(gt_bin, slam_bin, output_path):
    """
    Overlay RGB:
      Verde   (0,200,0)   = ambii liberi (overlap)
      Rosu    (220,0,0)   = GT liber, SLAM nu
      Albastru(0,0,220)   = SLAM liber, GT nu
      Gri     (180,180,180) = ambii neliber (pereti/necunoscut)
    """
    try:
        from PIL import Image
    except ImportError:
        print("  [overlay] PIL nu e instalat — overlay nesalvat")
        return

    h, w = gt_bin.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)

    gt_b  = gt_bin  > 0
    sl_b  = slam_bin > 0

    rgb[gt_b  & sl_b]  = [0,   200, 0  ]  # overlap → verde
    rgb[gt_b  & ~sl_b] = [220, 0,   0  ]  # GT only → rosu
    rgb[~gt_b & sl_b]  = [0,   0,   220]  # SLAM only → albastru
    rgb[~gt_b & ~sl_b] = [180, 180, 180]  # nimic → gri

    try:
        Image.fromarray(rgb).save(output_path)
        print(f"  Overlay salvat: {output_path}")
    except Exception as e:
        print(f"  [overlay] eroare la salvare: {e}")


# ── Functia principala ─────────────────────────────────────────────────────────

def compare_maps(gt_pgm, gt_yaml, slam_pgm, slam_yaml,
                 output_yaml=None, overlay_png=None):
    """
    Compara harta SLAM cu GT. Returneaza dict cu ssim, iou, area_err_pct.
    Poate fi importata din compute_metrics.py.
    """
    gt_arr,   gt_res,   _ = load_map(gt_pgm,   gt_yaml)
    slam_arr, slam_res, _ = load_map(slam_pgm, slam_yaml)

    # Area Error pe hartile originale (inainte de orice transform)
    area_err = compute_area_error(gt_arr, slam_arr, gt_res, slam_res)

    # Masti binare ale spatiului liber
    gt_free   = free_mask(gt_arr)
    slam_free = free_mask(slam_arr)

    # Crop la continut util
    gt_crop   = _crop_to_content(gt_free)
    slam_crop = _crop_to_content(slam_free)

    if gt_crop.size == 0 or slam_crop.size == 0:
        results = {"ssim": -1.0, "iou": -1.0, "area_err_pct": area_err}
        _save_results(results, output_yaml)
        return results

    # Resize SLAM la dimensiunile GT pentru comparatie pixel-cu-pixel
    slam_resized = _resize_nearest(slam_crop, gt_crop.shape[0], gt_crop.shape[1])

    ssim_val = compute_ssim(gt_crop, slam_resized)
    iou_val  = compute_iou(gt_crop, slam_resized)

    results = {
        "ssim":         ssim_val,
        "iou":          iou_val,
        "area_err_pct": area_err,
    }

    if overlay_png:
        save_overlay(gt_crop, slam_resized, overlay_png)

    _save_results(results, output_yaml)
    return results


def _save_results(results, output_yaml):
    if output_yaml:
        with open(output_yaml, "w") as f:
            yaml.dump(results, f, default_flow_style=False)
        print(f"  Metrici harta salvate: {output_yaml}")
    for k, v in results.items():
        print(f"  {k}: {v}")


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compara harta SLAM cu harta GT geometrica (S1 coridor)."
    )
    parser.add_argument("--gt-pgm",    required=True,  help="PGM harta ground truth")
    parser.add_argument("--gt-yaml",   required=True,  help="YAML harta ground truth")
    parser.add_argument("--slam-pgm",  required=True,  help="PGM harta SLAM")
    parser.add_argument("--slam-yaml", required=True,  help="YAML harta SLAM")
    parser.add_argument("--output",    default=None,   help="YAML output metrici")
    parser.add_argument("--overlay",   default=None,   help="PNG overlay vizual")
    args = parser.parse_args()

    compare_maps(
        gt_pgm    = args.gt_pgm,
        gt_yaml   = args.gt_yaml,
        slam_pgm  = args.slam_pgm,
        slam_yaml = args.slam_yaml,
        output_yaml = args.output,
        overlay_png = args.overlay,
    )


if __name__ == "__main__":
    main()
