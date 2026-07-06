#!/usr/bin/env python3
"""Compute SLAM metrics from a single experiment run."""

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def load_tum(path):
    ts, pos, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = list(map(float, line.split()))
            if len(vals) != 8:
                continue
            ts.append(vals[0])
            pos.append(vals[1:4])
            quat.append(vals[4:8])
    return np.array(ts), np.array(pos), np.array(quat)


def _strip_tum(path):
    """Sterge spatii trailing din fisier TUM — evo refuza fisierele cu spatii."""
    try:
        with open(path, "r") as f:
            lines = f.readlines()
        with open(path, "w") as f:
            for line in lines:
                f.write(line.rstrip() + "\n")
    except Exception as e:
        print(f"[metrics] nu am putut curata TUM {path}: {e}")


def compute_rpe(traj_path, gt_path):
    """
    Calculeaza RPE (segmente de 1m) cu evo.
    Referinta = odometrie (gt_path generat de gen_gt_odom.py).
    Returneaza dict cu rpe_mean, rpe_std.
    """
    try:
        from evo.core import metrics, sync, trajectory as evo_traj
        from evo.tools import file_interface
        from evo.core.metrics import PoseRelation, StatisticsType
    except ImportError:
        print("[metrics] ATENTIE: evo nu e instalat (pip3 install evo)")
        return _rpe_manual(traj_path, gt_path)

    try:
        traj_ref = file_interface.read_tum_trajectory_file(gt_path)
        traj_est = file_interface.read_tum_trajectory_file(traj_path)

        traj_ref.timestamps -= traj_ref.timestamps[0]
        traj_est.timestamps -= traj_est.timestamps[0]

        traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est,
                                                          max_diff=0.1)
        traj_est = evo_traj.align_trajectory(traj_est, traj_ref,
                                             correct_scale=False,
                                             correct_only_scale=False)

        rpe = metrics.RPE(PoseRelation.translation_part,
                          delta=1.0, delta_unit=metrics.Unit.meters,
                          all_pairs=False)
        rpe.process_data((traj_ref, traj_est))

        return {
            "rpe_mean": round(rpe.get_statistic(StatisticsType.mean), 4),
            "rpe_std":  round(rpe.get_statistic(StatisticsType.std),  4),
        }
    except Exception as e:
        print(f"[metrics] eroare evo RPE: {e}")
        return _rpe_manual(traj_path, gt_path)


def _rpe_manual(traj_path, gt_path):
    """Fallback RPE manual fara evo."""
    try:
        ts_e, pos_e, _ = load_tum(traj_path)
        ts_g, pos_g, _ = load_tum(gt_path)
        ts_e = ts_e - ts_e[0]
        ts_g = ts_g - ts_g[0]

        matched_e, matched_g = [], []
        for i, t in enumerate(ts_e):
            j = np.argmin(np.abs(ts_g - t))
            if abs(ts_g[j] - t) < 0.1:
                matched_e.append(pos_e[i, :2])
                matched_g.append(pos_g[j, :2])

        if len(matched_e) < 2:
            return {"rpe_mean": -1.0, "rpe_std": -1.0}

        pe = np.array(matched_e)
        pg = np.array(matched_g)
        delta_e = np.diff(pe, axis=0)
        delta_g = np.diff(pg, axis=0)
        rpe_vals = np.linalg.norm(delta_e - delta_g, axis=1)

        return {
            "rpe_mean": round(float(np.mean(rpe_vals)), 4),
            "rpe_std":  round(float(np.std(rpe_vals)),  4),
        }
    except Exception as e:
        print(f"[metrics] eroare fallback RPE: {e}")
        return {"rpe_mean": -1.0, "rpe_std": -1.0}


def _read_pgm_raw(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Nu e P5 PGM: {path}")
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


def mape_dimensions(map_yaml_path, scenario_cfg, scenario):
    """
    Masoara lungimea si latimea spatiului liber din harta SLAM si le compara
    cu dimensiunile reale din scenario_config.yaml.

    Lungimea = bounding box complet pe axa lunga.
    Latimea  = mediana profilelor din zona centrala 50% a coridorului —
               exclude capetele unde virajul 180 si corectiile lasa pixeli
               liberi "extra" care supraevalueaza latimea.

    Area Error = |L_map * W_map - L_real * W_real| / (L_real * W_real) * 100
                 (dimensional, nu numarare pixeli — nu e influentat de
                  "tight vs fuzzy" al diferitelor algoritmi SLAM).

    Returneaza dict cu mape_length_pct, mape_width_pct, mape_mean_pct,
    area_err_pct.
    """
    NEG = {
        "mape_length_pct": -1.0, "mape_width_pct": -1.0,
        "mape_mean_pct":   -1.0, "area_err_pct":   -1.0,
    }
    try:
        if not scenario_cfg:
            return NEG

        with open(scenario_cfg) as f:
            cfg = yaml.safe_load(f)

        sc = cfg.get(scenario)
        if sc is None:
            return NEG

        real_length = sc.get("reference_length_m")
        real_width  = sc.get("reference_width_m")

        if not real_length or real_length <= 0:
            return NEG

        with open(map_yaml_path) as f:
            map_info = yaml.safe_load(f)

        res = float(map_info.get("resolution", 0.05))
        pgm_path = os.path.join(os.path.dirname(map_yaml_path),
                                map_info.get("image", "map.pgm"))

        try:
            from PIL import Image
            arr = np.array(Image.open(pgm_path).convert("L"))
        except ImportError:
            arr = _read_pgm_raw(pgm_path)

        free = arr >= 250

        # Bounding box al spatiului liber
        rows_with_free = np.any(free, axis=1)
        cols_with_free = np.any(free, axis=0)
        if not rows_with_free.any() or not cols_with_free.any():
            return NEG

        r_idx = np.where(rows_with_free)[0]
        c_idx = np.where(cols_with_free)[0]
        r0, r1 = int(r_idx[0]),  int(r_idx[-1])
        c0, c1 = int(c_idx[0]),  int(c_idx[-1])

        x_span_px = c1 - c0 + 1   # Python int
        y_span_px = r1 - r0 + 1   # Python int

        # Axa lunga = lungime; axa scurta = latime
        if x_span_px >= y_span_px:
            # Coridor orizontal: lungime pe X, latime pe Y
            map_length = float(x_span_px) * res
            # Profiluri de latime in zona centrala 50% (excluzand capetele cu virajul)
            q = max(1, x_span_px // 4)
            widths_px = [int(free[r0:r1 + 1, c].sum())
                         for c in range(c0 + q, c1 - q + 1)]
        else:
            # Coridor vertical: lungime pe Y, latime pe X
            map_length = float(y_span_px) * res
            q = max(1, y_span_px // 4)
            widths_px = [int(free[r, c0:c1 + 1].sum())
                         for r in range(r0 + q, r1 - q + 1)]

        map_width = float(np.median(widths_px)) * res if widths_px else 0.0

        err_length = float(abs(real_length - map_length) / real_length * 100.0)
        result = {
            "mape_length_pct": round(err_length, 2),
            "mape_width_pct":  -1.0,
            "mape_mean_pct":   -1.0,
            "area_err_pct":    -1.0,
        }

        if real_width and real_width > 0:
            err_width = float(abs(real_width - map_width) / real_width * 100.0)
            result["mape_width_pct"] = round(err_width, 2)
            result["mape_mean_pct"]  = round(float(err_length + err_width) / 2.0, 2)
            # Area Error dimensional (nu numarare pixeli)
            area_gt   = float(real_length * real_width)
            area_meas = float(map_length  * map_width)
            result["area_err_pct"] = round(
                abs(area_gt - area_meas) / area_gt * 100.0, 2)

        return result

    except Exception as e:
        print(f"[metrics] eroare mape_dimensions: {e}")
        return NEG


# ── Loop closure (S2) ─────────────────────────────────────────────────────────

def loop_closure_error(traj_path):
    """Distanta prima-ultima pozitie. Relevant doar pentru S2."""
    try:
        _, pos, _ = load_tum(traj_path)
        if len(pos) < 2:
            return -1.0
        dx = pos[-1, 0] - pos[0, 0]
        dy = pos[-1, 1] - pos[0, 1]
        return round(math.sqrt(dx * dx + dy * dy), 4)
    except Exception as e:
        print(f"[metrics] eroare loop closure: {e}")
        return -1.0


# ── Completitudine harta (S3) ─────────────────────────────────────────────────

def map_completeness(map_yaml_path, scenario_cfg, scenario):
    """Procent din suprafata reala acoperita de harta. Relevant pentru S3."""
    try:
        if not scenario_cfg:
            return -1.0
        with open(scenario_cfg) as f:
            cfg = yaml.safe_load(f)
        sc = cfg.get(scenario)
        if sc is None:
            return -1.0
        real_area = sc.get("room_area_m2")
        if not real_area or real_area <= 0:
            rl = sc.get("room_length_m", 0)
            rw = sc.get("room_width_m",  0)
            real_area = float(rl) * float(rw) if rl and rw else 0
        if not real_area or real_area <= 0:
            return -1.0

        with open(map_yaml_path) as f:
            map_info = yaml.safe_load(f)
        res = float(map_info.get("resolution", 0.05))
        pgm_path = os.path.join(os.path.dirname(map_yaml_path),
                                map_info.get("image", "map.pgm"))
        try:
            from PIL import Image
            arr = np.array(Image.open(pgm_path).convert("L"))
        except ImportError:
            arr = _read_pgm_raw(pgm_path)

        free_pixels = int(np.sum(arr >= 250))
        mapped_area = free_pixels * res * res
        return round(min(mapped_area / real_area * 100.0, 100.0), 2)
    except Exception as e:
        print(f"[metrics] eroare completeness: {e}")
        return -1.0


# ── Calitate harta (SSIM, IoU, Area Error) ───────────────────────────────────

def map_quality(gt_pgm, gt_yaml, slam_pgm, slam_yaml, scenario="s1"):
    """
    Compara harta SLAM cu GT.
    S1: compare_corridor_maps (IoU pe spatiu LIBER)
    S2: compare_maps_s2       (IoU pe zone OCUPATE — pereti + canapea)
    Returneaza dict cu ssim, iou, area_err_pct.
    """
    try:
        if scenario in ("s2", "s3a", "s3b"):
            from compare_maps_s2 import compare_maps_s2
            return compare_maps_s2(
                gt_pgm=gt_pgm, gt_yaml=gt_yaml,
                slam_pgm=slam_pgm, slam_yaml=slam_yaml,
            )
        else:
            from compare_corridor_maps import compare_maps
            return compare_maps(
                gt_pgm=gt_pgm, gt_yaml=gt_yaml,
                slam_pgm=slam_pgm, slam_yaml=slam_yaml,
            )
    except Exception as e:
        print(f"[metrics] eroare map_quality: {e}")
        return {"ssim": -1.0, "iou": -1.0, "area_err_pct": -1.0}


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config",       required=True)
    parser.add_argument("--scenario",     required=True)
    parser.add_argument("--run",          required=True, type=int)
    parser.add_argument("--traj",         default=None,
                        help="Traiectorie TUM estimata de SLAM")
    parser.add_argument("--gt",           default=None,
                        help="Traiectorie TUM referinta (odom, gen_gt_odom.py)")
    parser.add_argument("--map",          required=True,
                        help="Fisier .yaml al hartii SLAM")
    parser.add_argument("--gt-map",       default=None,
                        help="PGM harta ground truth geometrica (pentru SSIM/IoU)")
    parser.add_argument("--gt-map-yaml",  default=None,
                        help="YAML al hartii ground truth geometrice")
    parser.add_argument("--scenario-cfg", default=None,
                        help="scenario_config.yaml cu dimensiuni fizice")
    parser.add_argument("--startup-file", default=None,
                        help="Fisier cu startup time (scris de tf_to_tum_node)")
    parser.add_argument("--map-hz",       type=float, default=None,
                        help="Frecventa /map in Hz")
    parser.add_argument("--output",       required=True,
                        help="Fisier metrics.yaml output")
    args = parser.parse_args()

    results = {
        "config":   args.config,
        "scenario": args.scenario,
        "run":      args.run,
    }

    # ── RPE ───────────────────────────────────────────────────────────────────
    traj_ok = args.traj and os.path.exists(args.traj)
    gt_ok   = args.gt   and os.path.exists(args.gt)

    if traj_ok:
        _strip_tum(args.traj)
    if gt_ok:
        _strip_tum(args.gt)

    if traj_ok and gt_ok:
        results.update(compute_rpe(args.traj, args.gt))
    else:
        if not traj_ok:
            print("[metrics] Traiectorie lipsa — RPE = -1")
        elif not gt_ok:
            print("[metrics] GT (odom) nu e specificat — RPE = -1")
        results.update({"rpe_mean": -1.0, "rpe_std": -1.0})

    # ── Loop closure — doar S2 ────────────────────────────────────────────────
    if args.scenario == "s2":
        results["loop_closure_m"] = loop_closure_error(args.traj) if traj_ok else -1.0

    # ── MAPE dimensional ──────────────────────────────────────────────────────
    map_yaml_exists = os.path.exists(args.map)
    if map_yaml_exists:
        if args.scenario == "s2":
            # S2: MAPE pe 8 masuratori (camera, canapea, distante)
            try:
                from mape_s2 import compute_mape_s2
                mape_res = compute_mape_s2(args.map, args.scenario_cfg, "s2")
                results.update(mape_res)
                # mape_mean_pct e inclus in mape_res
            except Exception as e:
                print(f"[metrics] eroare mape_s2: {e}")
                results["mape_mean_pct"] = -1.0
        else:
            results.update(mape_dimensions(args.map, args.scenario_cfg, args.scenario))
    else:
        results.update({
            "mape_length_pct": -1.0,
            "mape_width_pct":  -1.0,
            "mape_mean_pct":   -1.0,
            "area_err_pct":    -1.0,
        })

    # ── Calitate harta: SSIM, IoU, Area Error — cu GT map ────────────────────
    gt_map_pgm  = args.gt_map
    gt_map_yaml = args.gt_map_yaml

    # Daca nu s-a dat --gt-map-yaml, incearca cu aceeasi baza ca --gt-map
    if gt_map_pgm and not gt_map_yaml:
        candidate = os.path.splitext(gt_map_pgm)[0] + ".yaml"
        if os.path.exists(candidate):
            gt_map_yaml = candidate

    slam_pgm = None
    if map_yaml_exists:
        with open(args.map) as f:
            _mi = yaml.safe_load(f)
        slam_pgm = os.path.join(os.path.dirname(args.map),
                                _mi.get("image", "map.pgm"))

    if (gt_map_pgm and os.path.exists(gt_map_pgm) and
            gt_map_yaml and os.path.exists(gt_map_yaml) and
            slam_pgm and os.path.exists(slam_pgm)):
        mq = map_quality(gt_map_pgm, gt_map_yaml, slam_pgm, args.map,
                         scenario=args.scenario)
        # area_err_pct vine deja din mape (dimensional pentru S1, pixel-occ pentru S2)
        if args.scenario != "s2":
            mq.pop("area_err_pct", None)
        results.update(mq)
    else:
        results.setdefault("ssim", -1.0)
        results.setdefault("iou",  -1.0)

    # ── Completitudine — doar S3 ──────────────────────────────────────────────
    if args.scenario in ("s3a", "s3b"):
        results["completeness_pct"] = (
            map_completeness(args.map, args.scenario_cfg, args.scenario)
            if map_yaml_exists else -1.0
        )

    # ── Frecventa harta ───────────────────────────────────────────────────────
    results["map_hz"] = args.map_hz if args.map_hz is not None else -1.0

    # ── Startup time ──────────────────────────────────────────────────────────
    if args.startup_file and os.path.exists(args.startup_file):
        try:
            results["startup_s"] = float(open(args.startup_file).read().strip())
        except Exception:
            results["startup_s"] = -1.0
    else:
        results["startup_s"] = -1.0

    # ── Salveaza ──────────────────────────────────────────────────────────────
    with open(args.output, "w") as f:
        yaml.dump(results, f, default_flow_style=False)

    print(f"[metrics] salvat -> {args.output}")
    for k, v in results.items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
