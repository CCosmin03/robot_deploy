#!/usr/bin/env python3
"""
Script standalone (NU nod ROS) — monitorizeaza CPU% si RAM MB al procesului SLAM.
Foloseste psutil. Ruleaza in paralel cu SLAM, se opreste la SIGTERM/SIGINT.

Instalare dependinta:  pip3 install psutil pyyaml

Rulare:
  python3 monitor_resources.py --config c1 --output resources.yaml [--interval 0.5]
"""

import argparse
import signal
import sys
import time
import statistics

import psutil
import yaml


SLAM_KEYWORDS = {
    "c1": ["slam_gmapping"],
    "c2": ["slam_gmapping"],
    "c3": ["hector_mapping"],
    "c4": ["cartographer_node", "cartographer_nod"],
    "c5": ["cartographer_node", "cartographer_nod"],
    "c6": ["cartographer_node", "cartographer_nod"],
}


def find_slam_proc(config_id):
    keywords = SLAM_KEYWORDS.get(config_id, ["slam"])
    for proc in psutil.process_iter(["pid", "name", "cmdline", "status"]):
        try:
            name = proc.info["name"] or ""
            cmdline = " ".join(proc.info["cmdline"] or [])
            if proc.info["status"] == psutil.STATUS_ZOMBIE:
                continue
            for kw in keywords:
                if kw in name or kw in cmdline:
                    return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return None


def _stats(lst):
    if not lst:
        return {"mean": 0.0, "std": 0.0, "max": 0.0, "n": 0}
    return {
        "mean": round(sum(lst) / len(lst), 2),
        "std": round(statistics.stdev(lst) if len(lst) > 1 else 0.0, 2),
        "max": round(max(lst), 2),
        "n": len(lst),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, choices=list(SLAM_KEYWORDS.keys()))
    parser.add_argument("--output", required=True)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()

    running = [True]

    def stop(sig, frame):
        running[0] = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    # Asteapta procesul SLAM (max 60s — C5/C6 pornesc mai tarziu)
    proc = None
    deadline = time.time() + 60
    print(f"[monitor] caut procesul SLAM pentru {args.config}...", flush=True)
    while time.time() < deadline and running[0]:
        proc = find_slam_proc(args.config)
        if proc:
            break
        time.sleep(0.5)

    if proc is None:
        print("[monitor] ATENTIE: SLAM nu gasit, salvez date goale")
        _write(args.output, args.config, [], [])
        return

    print(f"[monitor] gasit PID={proc.pid} ({proc.name()})", flush=True)
    proc.cpu_percent(interval=None)  # primul apel e intotdeauna 0

    cpu_samples, ram_samples = [], []

    while running[0]:
        try:
            cpu = proc.cpu_percent(interval=args.interval)
            mem = proc.memory_info().rss / 1024 / 1024
            cpu_samples.append(cpu)
            ram_samples.append(mem)
        except psutil.NoSuchProcess:
            print("[monitor] procesul SLAM s-a terminat")
            break

    _write(args.output, args.config, cpu_samples, ram_samples)
    print(f"[monitor] salvat {len(cpu_samples)} esantioane -> {args.output}", flush=True)


def _write(path, config_id, cpu, ram):
    data = {
        "config": config_id,
        "cpu_percent": _stats(cpu),
        "ram_mb": _stats(ram),
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)


if __name__ == "__main__":
    main()
