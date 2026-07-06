#!/usr/bin/env python3
"""Generate a compact summary report from evaluation results."""

import argparse
import os
import statistics
import csv

import yaml


CONFIGS   = ["c1", "c2", "c3", "c4", "c5", "c6"]
SCENARIOS = ["s1", "s2", "s3a", "s3b"]

CONFIG_LABELS = {
    "c1": "GMap enc",
    "c2": "GMap+EKF",
    "c3": "Hector",
    "c4": "Carto enc",
    "c5": "Carto+EKF",
    "c6": "Carto IMU",
}

SCENARIO_LABELS = {
    "s1":  "Coridor",
    "s2":  "Loop rect",
    "s3a": "Lab static",
    "s3b": "Lab dinamic",
}


def load_run(results_dir, cfg, sc, run):
    d      = os.path.join(results_dir, f"{cfg}_{sc}_r{run}")
    m_path = os.path.join(d, "metrics.yaml")
    r_path = os.path.join(d, "resources.yaml")

    m = {}
    if os.path.exists(m_path):
        with open(m_path) as f:
            m = yaml.safe_load(f) or {}

    r = {}
    if os.path.exists(r_path):
        with open(r_path) as f:
            r = yaml.safe_load(f) or {}

    return m, r


def mean_std(values):
    valid = [v for v in values if v is not None and v >= 0]
    if not valid:
        return None, None
    m = sum(valid) / len(valid)
    s = statistics.stdev(valid) if len(valid) > 1 else 0.0
    return m, s


def collect(results_dir):
    data = {}
    for cfg in CONFIGS:
        data[cfg] = {}
        for sc in SCENARIOS:
            runs_rpe       = []
            runs_rpe_std   = []
            runs_loop      = []
            runs_mape_l    = []
            runs_mape_w    = []
            runs_mape_mean = []
            runs_ssim      = []
            runs_iou       = []
            runs_area_err  = []
            runs_compl     = []
            runs_cpu       = []
            runs_ram       = []
            runs_hz        = []
            runs_startup   = []

            for run in range(1, 6):
                m, r = load_run(results_dir, cfg, sc, run)
                if not m:
                    continue

                runs_rpe.append(m.get("rpe_mean", -1))
                runs_rpe_std.append(m.get("rpe_std",  -1))
                runs_loop.append(m.get("loop_closure_m", -1))
                runs_mape_l.append(m.get("mape_length_pct", -1))
                runs_mape_w.append(m.get("mape_width_pct",  -1))
                runs_mape_mean.append(m.get("mape_mean_pct", -1))
                runs_ssim.append(m.get("ssim",         -1))
                runs_iou.append(m.get("iou",           -1))
                runs_area_err.append(m.get("area_err_pct", -1))
                runs_compl.append(m.get("completeness_pct", -1))
                runs_hz.append(m.get("map_hz",    -1))
                runs_startup.append(m.get("startup_s", -1))

                if r:
                    runs_cpu.append(r.get("cpu_percent", {}).get("mean", -1))
                    runs_ram.append(r.get("ram_mb",      {}).get("mean", -1))

            ms = mean_std
            data[cfg][sc] = {
                "rpe":          ms(runs_rpe),
                "rpe_std_intra": ms(runs_rpe_std),
                "loop":         ms(runs_loop),
                "mape_l":    ms(runs_mape_l),
                "mape_w":    ms(runs_mape_w),
                "mape_mean": ms(runs_mape_mean),
                "ssim":      ms(runs_ssim),
                "iou":       ms(runs_iou),
                "area_err":  ms(runs_area_err),
                "compl":     ms(runs_compl),
                "cpu":       ms(runs_cpu),
                "ram":       ms(runs_ram),
                "map_hz":    ms(runs_hz),
                "startup":   ms(runs_startup),
                "n_runs":    len(runs_rpe),
            }
    return data


def fmt(val, decimals=3, suffix=""):
    if val is None:
        return "  -  "
    return f"{val:.{decimals}f}{suffix}"


def print_scenario_table(data, out_lines):
    SEP = "━" * 90

    for sc in SCENARIOS:
        sc_label = SCENARIO_LABELS.get(sc, sc).upper()
        out_lines.append("")
        out_lines.append(f"SCENARIU — {sc_label}")
        out_lines.append(SEP)

        header = f"{'METRICĂ':<30}"
        for cfg in CONFIGS:
            header += f"  {CONFIG_LABELS[cfg]:>10}"
        out_lines.append(header)
        out_lines.append(SEP)

        # Randuri comune tuturor scenariilor
        metrics_rows = [
            ("RPE mediu (m)",        "rpe",            4),
            ("RPE std (m)",          "rpe_std_intra",  4),
        ]

        # S1: MAPE dimensional + calitate harta
        if sc == "s1":
            metrics_rows += [
                ("MAPE Lungime (%)",    "mape_l",    2),
                ("MAPE Latime (%)",     "mape_w",    2),
                ("MAPE Mediu (%)",      "mape_mean", 2),
                ("  MAPE std (%)",      "mape_mean", 2, 1),
                ("SSIM",               "ssim",      4),
                ("  SSIM std",         "ssim",      4, 1),
                ("IoU",                "iou",       4),
                ("  IoU std",          "iou",       4, 1),
                ("Area Error (%)",     "area_err",  2),
            ]

        # S2: MAPE complet (8 masuratori) + calitate harta
        if sc == "s2":
            metrics_rows += [
                ("MAPE mediu (%)",     "mape_mean", 2),
                ("  MAPE std (%)",     "mape_mean", 2, 1),
                ("SSIM",              "ssim",      4),
                ("  SSIM std",        "ssim",      4, 1),
                ("IoU",               "iou",       4),
                ("  IoU std",         "iou",       4, 1),
                ("Loop closure (m)",  "loop",      4),
            ]

        # S3: SSIM + IoU + completitudine
        if sc in ("s3a", "s3b"):
            metrics_rows += [
                ("SSIM",               "ssim",  4),
                ("  SSIM std",         "ssim",  4, 1),
                ("IoU",                "iou",   4),
                ("  IoU std",          "iou",   4, 1),
                ("Completitudine (%)", "compl", 1),
                ("  Compl std (%)",    "compl", 1, 1),
            ]

        # Resurse — toate scenariile
        metrics_rows += [
            ("CPU % mediu",          "cpu",       1),
            ("RAM MB mediu",         "ram",       1),
            ("Frecv. harta (Hz)",    "map_hz",    2),
        ]

        for row in metrics_rows:
            label, key, dec = row[0], row[1], row[2]
            idx = row[3] if len(row) > 3 else 0   # 0=mean, 1=std
            line = f"  {label:<28}"
            for cfg in CONFIGS:
                val = data[cfg][sc].get(key, (None, None))[idx]
                line += f"  {fmt(val, dec):>10}"
            out_lines.append(line)

        out_lines.append(SEP)

    out_lines.append("")
    out_lines.append("IMPACT MEDIU DINAMIC (S3B vs S3A)")
    out_lines.append(SEP)
    header = f"{'METRICĂ':<30}"
    for cfg in CONFIGS:
        header += f"  {CONFIG_LABELS[cfg]:>10}"
    out_lines.append(header)
    out_lines.append(SEP)

    for label, key in [("RPE degradare (m)", "rpe"), ("RPE degradare (%)", "rpe")]:
        line = f"  {label:<28}"
        for cfg in CONFIGS:
            m3a = data[cfg]["s3a"].get(key, (None, None))[0]
            m3b = data[cfg]["s3b"].get(key, (None, None))[0]
            if m3a is not None and m3b is not None and m3a >= 0 and m3b >= 0:
                diff = m3b - m3a
                if "%" in label:
                    diff = (diff / m3a * 100) if m3a > 0 else 0.0
                    line += f"  {fmt(diff, 2):>10}"
                else:
                    line += f"  {fmt(diff, 4):>10}"
            else:
                line += f"  {'  -  ':>10}"
        out_lines.append(line)
    out_lines.append(SEP)

    out_lines.append("")
    out_lines.append("RESURSE MEDII GLOBALE")
    out_lines.append(SEP)
    header = f"{'METRICĂ':<30}"
    for cfg in CONFIGS:
        header += f"  {CONFIG_LABELS[cfg]:>10}"
    out_lines.append(header)
    out_lines.append(SEP)

    for label, key, dec in [
        ("CPU % mediu global",  "cpu",     1),
        ("RAM MB mediu global", "ram",     1),
        ("Startup (s)",         "startup", 2),
    ]:
        line = f"  {label:<28}"
        for cfg in CONFIGS:
            vals = [data[cfg][sc].get(key, (None, None))[0] for sc in SCENARIOS]
            valid = [v for v in vals if v is not None and v >= 0]
            m = sum(valid) / len(valid) if valid else None
            line += f"  {fmt(m, dec):>10}"
        out_lines.append(line)
    out_lines.append(SEP)


def save_csv(data, csv_path):
    rows = []
    for cfg in CONFIGS:
        for sc in SCENARIOS:
            entry = data[cfg][sc]
            row = {
                "config":       cfg,
                "scenario":     sc,
                "rpe_mean":           entry["rpe"][0],
                "rpe_std_across_runs": entry["rpe"][1],
                "rpe_std_intra":      entry["rpe_std_intra"][0],
                "loop_m":       entry["loop"][0],
                "mape_l_pct":   entry["mape_l"][0],
                "mape_w_pct":   entry["mape_w"][0],
                "mape_mean_pct":entry["mape_mean"][0],
                "ssim":         entry["ssim"][0],
                "iou":          entry["iou"][0],
                "area_err_pct": entry["area_err"][0],
                "compl_pct":    entry["compl"][0],
                "cpu_pct":      entry["cpu"][0],
                "ram_mb":       entry["ram"][0],
                "map_hz":       entry["map_hz"][0],
                "startup_s":    entry["startup"][0],
                "n_runs":       entry["n_runs"],
            }
            rows.append(row)

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default=None,
                        help="Director cu rezultate (default: ../results)")
    parser.add_argument("--output", default="report.txt")
    parser.add_argument("--csv",    default="report.csv")
    args = parser.parse_args()

    if args.results_dir is None:
        script_dir  = os.path.dirname(os.path.abspath(__file__))
        results_dir = os.path.join(script_dir, "..", "results")
    else:
        results_dir = args.results_dir

    results_dir = os.path.abspath(results_dir)
    print(f"Citesc rezultate din: {results_dir}")

    data = collect(results_dir)

    lines = []
    lines.append("=" * 90)
    lines.append("  RAPORT EVALUARE SLAM — COMPARATIE 6 CONFIGURATII x 4 SCENARII")
    lines.append("=" * 90)
    print_scenario_table(data, lines)
    text = "\n".join(lines)

    print(text)

    out_path = os.path.abspath(args.output)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"\nRaport text salvat: {out_path}")

    csv_path = os.path.abspath(args.csv)
    save_csv(data, csv_path)
    print(f"CSV salvat: {csv_path}")


if __name__ == "__main__":
    main()
