#!/usr/bin/env python3
"""Summarize and optionally plot a Hugmy SB3 Monitor training run."""

import argparse
import csv
from pathlib import Path
import statistics


def monitor_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def rolling_mean(values, window):
    result = []
    running = 0.0
    for index, value in enumerate(values):
        running += value
        if index >= window:
            running -= values[index - window]
        result.append(running / min(index + 1, window))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="training output directory")
    parser.add_argument("--window", type=int, default=20)
    parser.add_argument("--plot", action="store_true",
                        help="write training_summary.png")
    args = parser.parse_args()

    output = Path(args.output).expanduser().resolve()
    monitor = output / "monitor.csv"
    with monitor.open(encoding="utf-8") as stream:
        stream.readline()  # SB3 Monitor metadata comment
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit(f"No completed episodes in {monitor}")

    rewards = [float(row["r"]) for row in rows]
    lengths = [int(row["l"]) for row in rows]
    wall_times = [float(row["t"]) for row in rows]
    steps = sum(lengths)
    wall_seconds = wall_times[-1]
    window = max(1, min(args.window, len(rows)))
    first = rewards[:window]
    latest = rewards[-window:]

    print(f"episodes:             {len(rows)}")
    print(f"completed env steps:  {steps}")
    print(f"wall time:            {wall_seconds / 3600.0:.2f} h")
    print(f"throughput:           {steps / wall_seconds:.2f} step/s")
    print(f"simulation RTF:       {0.1 * steps / wall_seconds:.3f}")
    print(f"first {window} reward mean: {statistics.mean(first):.3f}")
    print(f"last  {window} reward mean: {statistics.mean(latest):.3f}")
    print(f"last  {window} reward median: {statistics.median(latest):.3f}")
    print(f"best episode reward:  {max(rewards):.3f}")
    print(f"latest mean length:   {statistics.mean(lengths[-window:]):.1f} step")
    if "stride_success" in rows[0]:
        latest_rows = rows[-window:]
        success_rate = 100.0 * statistics.mean(
            monitor_bool(row["stride_success"]) for row in latest_rows)
        print(f"last  {window} episodes with a stride: {success_rate:.1f}%")
        if "completed_stride_count" in rows[0]:
            stride_counts = [
                int(float(row["completed_stride_count"]))
                for row in latest_rows]
            completed_lengths = [
                1000.0 * float(row["mean_completed_stride_m"])
                for row in latest_rows
                if int(float(row["completed_stride_count"])) > 0]
            runaway_rate = 100.0 * statistics.mean(
                monitor_bool(row["stride_runaway"]) for row in latest_rows)
            print(
                f"last  {window} completed strides/episode: "
                f"mean={statistics.mean(stride_counts):.2f}, "
                f"max={max(stride_counts)}")
            if completed_lengths:
                print(
                    f"last  {window} completed stride length: "
                    f"mean={statistics.mean(completed_lengths):.1f} mm")
            print(f"last  {window} runaway (>100 mm): {runaway_rate:.1f}%")
        else:
            overshoot_rate = 100.0 * statistics.mean(
                monitor_bool(row["stride_overshoot"]) for row in latest_rows)
            stride_progress = [
                1000.0 * float(row["stride_progress_m"])
                for row in latest_rows]
            print(f"last  {window} stride overshoot: {overshoot_rate:.1f}%")
            print(
                f"last  {window} qualified stride: "
                f"mean={statistics.mean(stride_progress):.1f} mm, "
                f"median={statistics.median(stride_progress):.1f} mm")
    if "target_direction" in rows[0]:
        for direction, label in ((1.0, "forward"), (-1.0, "backward")):
            selected = [
                row for row in rows
                if float(row.get("target_direction", 0.0)) == direction]
            if not selected:
                continue
            selected_rewards = [float(row["r"]) for row in selected]
            selected_progress = [
                float(row["total_progress_m"]) for row in selected]
            summary = (
                f"{label:8s}: episodes={len(selected):4d}, "
                f"reward mean={statistics.mean(selected_rewards):8.3f}, "
                f"progress mean={statistics.mean(selected_progress):+.4f} m")
            if "stride_success" in rows[0]:
                successes = 100.0 * statistics.mean(
                    monitor_bool(row["stride_success"]) for row in selected)
                summary += f", episodes with stride={successes:.1f}%"
                if "completed_stride_count" in rows[0]:
                    mean_count = statistics.mean(
                        int(float(row["completed_stride_count"]))
                        for row in selected)
                    runaways = 100.0 * statistics.mean(
                        monitor_bool(row["stride_runaway"])
                        for row in selected)
                    summary += (
                        f", strides/episode={mean_count:.2f}, "
                        f"runaway={runaways:.1f}%")
                else:
                    overshoots = 100.0 * statistics.mean(
                        monitor_bool(row["stride_overshoot"])
                        for row in selected)
                    summary += f", overshoot={overshoots:.1f}%"
            print(summary)

    if not args.plot:
        return
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(f"Plot requested but matplotlib is unavailable: {exc}")
    episode = list(range(1, len(rows) + 1))
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(episode, rewards, alpha=0.28, label="episode reward")
    axes[0].plot(episode, rolling_mean(rewards, window),
                 linewidth=2, label=f"rolling mean ({window})")
    axes[0].set_ylabel("reward")
    axes[0].grid(alpha=0.25)
    axes[0].legend()
    axes[1].plot(episode, lengths, alpha=0.5)
    axes[1].set_xlabel("completed episode")
    axes[1].set_ylabel("episode length [step]")
    axes[1].grid(alpha=0.25)
    figure.tight_layout()
    path = output / "training_summary.png"
    figure.savefig(path, dpi=150)
    print(f"plot:                 {path}")


if __name__ == "__main__":
    main()
