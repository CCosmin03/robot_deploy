#!/usr/bin/env python3
"""Estimate S2 map metrics by detecting the room and sofa from a SLAM map."""

import argparse
import os
import sys

import numpy as np
import yaml

FREE_THRESH = 250   # >= 250 = liber confirmat
OCC_THRESH  = 100   # <= 100 = ocupat; 100 captureaza celulele "gri" din SLAM


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


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    res = float(meta.get("resolution", 0.05))
    pgm = os.path.join(os.path.dirname(yaml_path), meta.get("image", "map.pgm"))
    try:
        from PIL import Image
        arr = np.array(Image.open(pgm).convert("L"))
    except ImportError:
        arr = _read_pgm(pgm)
    return arr, res


def detect_room(arr):
    """
    Returneaza (r0, r1, c0, c1) bounding box al celei mai mari zone libere conexe.
    Foloseste cel mai mare component conex pentru a exclude celule libere straine
    (coridor de acces, zona de start) prezente in hartile Cartographer.
    """
    free = (arr >= FREE_THRESH).astype(np.uint8)
    try:
        from scipy import ndimage
        labeled, n = ndimage.label(free)
        if n == 0:
            return None
        sizes = ndimage.sum(free, labeled, range(1, n + 1))
        largest_lbl = int(np.argmax(sizes)) + 1
        room_mask = (labeled == largest_lbl)
    except ImportError:
        room_mask = free.astype(bool)
    rows = np.any(room_mask, axis=1)
    cols = np.any(room_mask, axis=0)
    if not rows.any():
        return None
    r0 = int(np.where(rows)[0][0])
    r1 = int(np.where(rows)[0][-1])
    c0 = int(np.where(cols)[0][0])
    c1 = int(np.where(cols)[0][-1])
    return r0, r1, c0, c1


def detect_sofa(arr, room_bbox, real_s1, real_s2, res):
    """
    Detecteaza canapeaua ca obstacol interior cu dimensiuni compatibile cu S1 x S2.
    - border de 3px eliminat (prinde pereti de 2px)
    - dilatare 2px inainte de componente conexe (uneste inelul fragmentat Cartographer)
    - filtrare dupa dimensiune: accepta doar obiecte in intervalul [0.3, 1.7] x dimensiunea reala
    - parcurge componentele de la cea mai mare la cea mai mica
    """
    r0, r1, c0, c1 = room_bbox
    interior = arr[r0:r1 + 1, c0:c1 + 1]
    occ_mask = (interior <= OCC_THRESH).astype(np.uint8)

    bw = 3
    border = np.zeros_like(occ_mask)
    border[:bw, :] = 1;  border[-bw:, :] = 1
    border[:, :bw] = 1;  border[:, -bw:] = 1
    inner_occ = occ_mask & (border == 0)

    if not inner_occ.any():
        return None

    tol_lo, tol_hi = 0.3, 1.7
    s_lg_exp = max(real_s1, real_s2) / res
    s_sm_exp = min(real_s1, real_s2) / res

    try:
        from scipy import ndimage

        struct  = np.ones((3, 3), dtype=bool)
        dilated = ndimage.binary_dilation(inner_occ.astype(bool),
                                          structure=struct, iterations=2)
        dilated = dilated & (border == 0)
        labeled, n = ndimage.label(dilated)
        if n == 0:
            return None

        sizes = np.array(ndimage.sum(dilated.astype(int), labeled, range(1, n + 1)))

        for lbl in (np.argsort(sizes)[::-1] + 1):
            comp = (labeled == int(lbl))
            cr = np.any(comp, axis=1);  cc = np.any(comp, axis=0)
            if not cr.any():
                continue
            sr0_l = int(np.where(cr)[0][0]);  sr1_l = int(np.where(cr)[0][-1])
            sc0_l = int(np.where(cc)[0][0]);  sc1_l = int(np.where(cc)[0][-1])
            h = sr1_l - sr0_l + 1;  w = sc1_l - sc0_l + 1
            s_lg = max(h, w);  s_sm = min(h, w)
            if (s_lg_exp * tol_lo <= s_lg <= s_lg_exp * tol_hi and
                    s_sm_exp * tol_lo <= s_sm <= s_sm_exp * tol_hi):
                return r0 + sr0_l, r0 + sr1_l, c0 + sc0_l, c0 + sc1_l

        print("[mape_s2] nicio componenta nu se incadreaza in dimensiunile asteptate ale canapeii")
        return None

    except ImportError:
        component = inner_occ.astype(bool)
        cr = np.any(component, axis=1);  cc = np.any(component, axis=0)
        if not cr.any():
            return None
        sr0 = int(np.where(cr)[0][0]);  sr1 = int(np.where(cr)[0][-1])
        sc0 = int(np.where(cc)[0][0]);  sc1 = int(np.where(cc)[0][-1])
        return r0 + sr0, r0 + sr1, c0 + sc0, c0 + sc1


def compute_mape_s2(map_yaml, scenario_cfg_path, scenario="s2"):
    """
    Calculeaza MAPE pe 8 masuratori pentru S2.
    Returneaza dict cu valori individuale si MAPE mediu.
    """
    NEG = {
        "mape_c1_pct": -1.0, "mape_c2_pct": -1.0,
        "mape_s1_pct": -1.0, "mape_s2_pct": -1.0,
        "mape_d1_pct": -1.0, "mape_d2_pct": -1.0,
        "mape_d3_pct": -1.0, "mape_d4_pct": -1.0,
        "mape_mean_pct": -1.0,
    }

    try:
        with open(scenario_cfg_path) as f:
            cfg_all = yaml.safe_load(f)
        cfg = cfg_all.get(scenario, {})

        real_c1 = float(cfg.get("room_length_m",  0))
        real_c2 = float(cfg.get("room_width_m",   0))
        real_s1 = float(cfg.get("sofa_length_m",  0))
        real_s2 = float(cfg.get("sofa_width_m",   0))
        real_d1 = float(cfg.get("dist_north_m",   0))
        real_d2 = float(cfg.get("dist_south_m",   0))
        real_d3 = float(cfg.get("dist_east_m",    0))
        real_d4 = float(cfg.get("dist_west_m",    0))

        if real_c1 <= 0 or real_c2 <= 0:
            print("[mape_s2] room_length_m / room_width_m necompletate in config")
            return NEG

        arr, res = load_map(map_yaml)
        room_bbox = detect_room(arr)
        if room_bbox is None:
            print("[mape_s2] nu s-a detectat camera din harta")
            return NEG

        r0, r1, c0, c1 = room_bbox

        # Dimensiuni camera masurate din SLAM
        map_c1_px = c1 - c0 + 1   # axa X = lungime
        map_c2_px = r1 - r0 + 1   # axa Y = latime

        # Conventia: C1 = dimensiunea mai mare, C2 = mai mica
        map_c_large = max(map_c1_px, map_c2_px) * res
        map_c_small = min(map_c1_px, map_c2_px) * res
        real_c_large = max(real_c1, real_c2)
        real_c_small = min(real_c1, real_c2)

        mape_c1 = float(abs(real_c_large - map_c_large) / real_c_large * 100)
        mape_c2 = float(abs(real_c_small - map_c_small) / real_c_small * 100)

        result = {
            "mape_c1_pct": round(mape_c1, 2),
            "mape_c2_pct": round(mape_c2, 2),
            "mape_s1_pct": -1.0,
            "mape_s2_pct": -1.0,
            "mape_d1_pct": -1.0,
            "mape_d2_pct": -1.0,
            "mape_d3_pct": -1.0,
            "mape_d4_pct": -1.0,
            "mape_mean_pct": -1.0,
        }

        errors = [mape_c1, mape_c2]

        # Detectie canapea si masuratori S1/S2/D1-D4
        if real_s1 > 0 and real_s2 > 0:
            sofa_bbox = detect_sofa(arr, room_bbox, real_s1, real_s2, res)
            if sofa_bbox is not None:
                sr0, sr1, sc0_s, sc1_s = sofa_bbox

                map_sl_px = sc1_s - sc0_s + 1   # X
                map_sw_px = sr1   - sr0   + 1   # Y

                # S1/S2: dimensiunea mai mare = S1 (lungime)
                map_s_large = max(map_sl_px, map_sw_px) * res
                map_s_small = min(map_sl_px, map_sw_px) * res
                real_s_large = max(real_s1, real_s2)
                real_s_small = min(real_s1, real_s2)

                mape_s1 = float(abs(real_s_large - map_s_large) / real_s_large * 100)
                mape_s2 = float(abs(real_s_small - map_s_small) / real_s_small * 100)
                result["mape_s1_pct"] = round(mape_s1, 2)
                result["mape_s2_pct"] = round(mape_s2, 2)
                errors += [mape_s1, mape_s2]

                # Distante canapea → pereti
                d_top   = float((sr0   - r0)   * res)
                d_bot   = float((r1    - sr1)  * res)
                d_right = float((c1    - sc1_s) * res)
                d_left  = float((sc0_s - c0)   * res)

                # Matching prin constrângere geometrica:
                # (d_top, d_bot) = axa N-S, (d_left, d_right) = axa E-V
                real_ns  = sorted([real_d1, real_d2])
                real_ew  = sorted([real_d3, real_d4])
                pair_tb  = sorted([d_top, d_bot])
                pair_lr  = sorted([d_left, d_right])

                err_tb_ns = sum(abs(a - b) for a, b in zip(pair_tb, real_ns))
                err_tb_ew = sum(abs(a - b) for a, b in zip(pair_tb, real_ew))

                if err_tb_ns <= err_tb_ew:
                    ns_meas, ew_meas = pair_tb, pair_lr
                else:
                    ns_meas, ew_meas = pair_lr, pair_tb

                real_all = sorted([real_d1, real_d2, real_d3, real_d4])
                map_all  = sorted(ns_meas + ew_meas)
                for i, (md, rd) in enumerate(zip(map_all, real_all)):
                    err = float(abs(rd - md) / rd * 100) if rd > 0 else -1.0
                    key = f"mape_d{i+1}_pct"
                    result[key] = round(err, 2) if err >= 0 else -1.0
                    if err >= 0:
                        errors.append(err)
            else:
                print("[mape_s2] canapeaua nu a fost detectata in harta SLAM")

        valid_errors = [e for e in errors if e >= 0]
        if valid_errors:
            result["mape_mean_pct"] = round(float(sum(valid_errors) / len(valid_errors)), 2)

        return result

    except Exception as e:
        print(f"[mape_s2] eroare: {e}")
        import traceback; traceback.print_exc()
        return NEG


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Calculeaza MAPE S2 (8 masuratori: camera, canapea, distante)."
    )
    parser.add_argument("--map",    required=True, help="map.yaml al hartii SLAM")
    parser.add_argument("--config", required=True, help="scenario_config.yaml")
    parser.add_argument("--output", default=None,  help="YAML output (optional)")
    args = parser.parse_args()

    result = compute_mape_s2(args.map, args.config)

    print("\n── MAPE S2 ─────────────────────────────────")
    labels = {
        "mape_c1_pct": "C1 Lungime camera (%)",
        "mape_c2_pct": "C2 Latime camera  (%)",
        "mape_s1_pct": "S1 Lungime canapea(%)",
        "mape_s2_pct": "S2 Latime canapea (%)",
        "mape_d1_pct": "D1 dist Nord      (%)",
        "mape_d2_pct": "D2 dist Sud       (%)",
        "mape_d3_pct": "D3 dist Est       (%)",
        "mape_d4_pct": "D4 dist Vest      (%)",
        "mape_mean_pct": "MAPE Mediu        (%)",
    }
    for k, label in labels.items():
        v = result.get(k, -1)
        print(f"  {label}: {v:.2f}" if v >= 0 else f"  {label}: —")

    if args.output:
        with open(args.output, "w") as f:
            yaml.dump(result, f, default_flow_style=False)
        print(f"\nSalvat: {args.output}")


if __name__ == "__main__":
    main()
