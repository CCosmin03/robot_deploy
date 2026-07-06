#!/usr/bin/env python3
"""
Compara harta SLAM cu GT pentru Scenariul 2 (Loop Closure - Canapea).

Spre deosebire de S1 (care compara spatiul LIBER), S2 compara zonele OCUPATE
(pereti + canapea), conform specificatiei din scenariu2.md:
  IoU = (Real ocupat ∩ SLAM ocupat) / (Real ocupat ∪ SLAM ocupat)

SSIM se calculeaza pe harta completa (toate valorile de gri, nu doar binar).

Overlay:
  Rosu   = Real ocupat, SLAM nu (obstacol nedetectat)
  Albastru = SLAM ocupat, Real nu (obstacol fictiv)
  Verde  = overlap ocupat (detectie corecta)
  Gri    = ambele libere

Utilizare standalone:
  python3 compare_maps_s2.py \
      --gt-pgm  ground_truth/real_map_s2_sofa.pgm \
      --gt-yaml ground_truth/real_map_s2_sofa.yaml \
      --slam-pgm  results/c1_s2_r1/map.pgm \
      --slam-yaml results/c1_s2_r1/map.yaml \
      --output results/c1_s2_r1/map_compare_s2.yaml \
      --overlay results/c1_s2_r1/overlay_s2.png
"""

import argparse
import os

import numpy as np
import yaml

FREE_THRESH = 250   # >= 250 = liber confirmat
OCC_THRESH  = 100   # <= 100 = ocupat (perete / canapea)
# 100 in loc de 50: captureaza si celulele "gri" din hartile SLAM unde
# probabilitatea de ocupare e ≥35% dar < 65% (ex: canapea vazuta partial)


# ── Citire harta ──────────────────────────────────────────────────────────────

def _read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Nu e PGM P5: {path}")
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
    try:
        from PIL import Image
        arr = np.array(Image.open(pgm_path).convert("L"))
    except ImportError:
        arr = _read_pgm(pgm_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    res    = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return arr, res, float(origin[0]), float(origin[1])


# ── Aliniere prin cross-corelatie ────────────────────────────────────────────

def _place(gt_arr, sl_arr, dy, dx):
    """Plaseaza sl_arr in canvas de dimensiunea gt_arr cu offset (dy, dx)."""
    h_gt, w_gt = gt_arr.shape
    h_sl, w_sl = sl_arr.shape
    aligned = np.full((h_gt, w_gt), 205, dtype=np.uint8)
    dst_r0 = max(0, dy);   dst_c0 = max(0, dx)
    dst_r1 = min(h_gt, dy + h_sl);  dst_c1 = min(w_gt, dx + w_sl)
    src_r0 = max(0, -dy);  src_c0 = max(0, -dx)
    src_r1 = src_r0 + (dst_r1 - dst_r0)
    src_c1 = src_c0 + (dst_c1 - dst_c0)
    if dst_r1 > dst_r0 and dst_c1 > dst_c0:
        aligned[dst_r0:dst_r1, dst_c0:dst_c1] = sl_arr[src_r0:src_r1, src_c0:src_c1]
    return aligned


def align_to_gt(gt_arr, gt_res, gt_ox, gt_oy,
                sl_arr, sl_res, sl_ox, sl_oy):
    """
    Aliniaza harta SLAM la GT prin cross-corelatie FFT pe celulele ocupate.
    Incearca 4 rotatii (0/90/180/270) si alege combinatia cu IoU maxim.
    Nu depinde de originile YAML — robusta la orice pozitie/orientare de start.
    """
    gt_occ = (gt_arr <= OCC_THRESH).astype(np.float32)

    try:
        from scipy.signal import fftconvolve
        use_fft = True
    except ImportError:
        use_fft = False

    best_aligned = np.full_like(gt_arr, 205)
    best_iou = -1.0
    best_rot = 0

    for k in range(4):
        rotated = np.rot90(sl_arr, k=k)
        h_sl, w_sl = rotated.shape
        sl_occ = (rotated <= OCC_THRESH).astype(np.float32)

        if use_fft:
            # Cross-corelatie FFT: kernel inversat = corelatie
            corr = fftconvolve(gt_occ, sl_occ[::-1, ::-1], mode='full')
            cy, cx = np.unravel_index(np.argmax(corr), corr.shape)
            dy = cy - (h_sl - 1)
            dx = cx - (w_sl - 1)
        else:
            # Fallback: centreaza harta SLAM in GT
            dy = (gt_arr.shape[0] - h_sl) // 2
            dx = (gt_arr.shape[1] - w_sl) // 2

        aligned = _place(gt_arr, rotated, dy, dx)
        iou = compute_iou_occupied(gt_arr, aligned)

        if iou > best_iou:
            best_iou = iou
            best_aligned = aligned
            best_rot = k * 90

    print(f"  [align] rotatie={best_rot}°  IoU={best_iou:.4f}")
    return best_aligned


# ── Metrici ───────────────────────────────────────────────────────────────────

def occupied_mask(arr):
    return (arr <= OCC_THRESH).astype(np.uint8)


def compute_iou_occupied(gt_arr, slam_arr):
    """IoU pe zonele OCUPATE (pereti + canapea)."""
    gt_occ   = occupied_mask(gt_arr)
    slam_occ = occupied_mask(slam_arr)

    # Excludem pixelii necunoscuti din SLAM (205) — ii tratam ca absenta de date
    slam_known = (slam_arr != 205)

    # Lucram doar in zona cunoscuta de ambele harti
    gt_occ_k   = gt_occ   & slam_known
    slam_occ_k = slam_occ & slam_known

    inter = int(np.logical_and(gt_occ_k, slam_occ_k).sum())
    union = int(np.logical_or( gt_occ_k, slam_occ_k).sum())

    if union == 0:
        return 0.0
    return round(float(inter) / float(union), 4)


def compute_ssim_map(gt_arr, slam_arr):
    """SSIM pe harta completa (nu binar) — compara structura generala."""
    # Inlocuim 205 (necunoscut SLAM) cu 205 si GT are tot 205 la margini
    # → includem doar zona unde ambele au date
    known = (gt_arr != 205) & (slam_arr != 205)
    if known.sum() < 100:
        return -1.0

    gt_f   = gt_arr.astype(float)
    slam_f = slam_arr.astype(float)

    try:
        from skimage.metrics import structural_similarity as ssim
        # SSIM pe imaginea completa (data_range=255)
        score, _ = ssim(gt_f, slam_f, full=True, data_range=255.0)
        return round(float(score), 4)
    except ImportError:
        # Fallback: corelatie normalizata pe zona cunoscuta
        g = gt_f[known]
        s = slam_f[known]
        mu_g, mu_s = g.mean(), s.mean()
        std_g, std_s = g.std(), s.std()
        if std_g < 1e-10 or std_s < 1e-10:
            return 1.0 if np.allclose(g, s) else 0.0
        cov = ((g - mu_g) * (s - mu_s)).mean()
        c1, c2 = 0.01**2, 0.03**2
        ssim_val = ((2*mu_g*mu_s + c1)*(2*cov + c2)) / \
                   ((mu_g**2 + mu_s**2 + c1)*(std_g**2 + std_s**2 + c2))
        return round(float(np.clip(ssim_val, -1.0, 1.0)), 4)


# ── Overlay ───────────────────────────────────────────────────────────────────

def save_overlay(gt_arr, slam_arr, output_path):
    """
    Overlay pe zonele ocupate:
      Verde   = ambele ocupate (detectie corecta)
      Rosu    = GT ocupat, SLAM nu (obstacol nedetectat de SLAM)
      Albastru= SLAM ocupat, GT nu (obstacol fictiv in SLAM)
      Gri     = ambele libere / necunoscute
    """
    try:
        from PIL import Image
    except ImportError:
        print("  [overlay] PIL nu e instalat — overlay nesalvat")
        return

    h, w = gt_arr.shape
    rgb = np.full((h, w, 3), 180, dtype=np.uint8)  # gri default

    gt_occ   = gt_arr   <= OCC_THRESH
    slam_occ = slam_arr <= OCC_THRESH
    slam_unk = slam_arr == 205

    rgb[gt_occ  & slam_occ]              = [0,   200, 0  ]   # overlap → verde
    rgb[gt_occ  & ~slam_occ & ~slam_unk] = [220, 0,   0  ]   # GT only → rosu
    rgb[~gt_occ & slam_occ]              = [0,   0,   220]   # SLAM only → albastru

    try:
        Image.fromarray(rgb).save(output_path)
        print(f"  Overlay salvat: {output_path}")
    except Exception as e:
        print(f"  [overlay] eroare: {e}")


# ── Functia principala ────────────────────────────────────────────────────────

def compare_maps_s2(gt_pgm, gt_yaml, slam_pgm, slam_yaml,
                    output_yaml=None, overlay_png=None):
    """
    Compara harta SLAM cu GT pentru S2.
    Returneaza dict cu ssim, iou, area_err_pct.
    Poate fi importata din compute_metrics.py.
    """
    gt_arr,   gt_res,   gt_ox,   gt_oy   = load_map(gt_pgm,   gt_yaml)
    slam_arr, slam_res, slam_ox, slam_oy = load_map(slam_pgm, slam_yaml)

    # Aliniere prin cross-corelatie (nu prin origine — sistemele de coord difera)
    slam_aligned = align_to_gt(gt_arr,   gt_res,   gt_ox,   gt_oy,
                                slam_arr, slam_res, slam_ox, slam_oy)

    iou_val  = compute_iou_occupied(gt_arr, slam_aligned)
    ssim_val = compute_ssim_map(gt_arr, slam_aligned)

    # Area Error: diferenta procent intre suprafata ocupata GT vs SLAM
    gt_occ_px   = int((gt_arr   <= OCC_THRESH).sum())
    slam_occ_px = int((slam_aligned <= OCC_THRESH).sum())
    if gt_occ_px > 0:
        area_err = round(float(abs(gt_occ_px - slam_occ_px)) / gt_occ_px * 100.0, 2)
    else:
        area_err = -1.0

    results = {
        "ssim":         ssim_val,
        "iou":          iou_val,
        "area_err_pct": area_err,
    }

    if overlay_png:
        save_overlay(gt_arr, slam_aligned, overlay_png)

    if output_yaml:
        with open(output_yaml, "w") as f:
            yaml.dump(results, f, default_flow_style=False)
        print(f"  Metrici S2 salvate: {output_yaml}")

    for k, v in results.items():
        print(f"  {k}: {v}")

    return results


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compara harta SLAM cu GT S2 (IoU pe zone ocupate)."
    )
    parser.add_argument("--gt-pgm",    required=True)
    parser.add_argument("--gt-yaml",   required=True)
    parser.add_argument("--slam-pgm",  required=True)
    parser.add_argument("--slam-yaml", required=True)
    parser.add_argument("--output",    default=None)
    parser.add_argument("--overlay",   default=None)
    args = parser.parse_args()

    compare_maps_s2(
        gt_pgm=args.gt_pgm, gt_yaml=args.gt_yaml,
        slam_pgm=args.slam_pgm, slam_yaml=args.slam_yaml,
        output_yaml=args.output, overlay_png=args.overlay,
    )


if __name__ == "__main__":
    main()
