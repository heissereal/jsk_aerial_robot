#!/usr/bin/env python3
"""Summarize and optionally plot a Hugmy SB3 Monitor training run."""

import argparse
import csv
from pathlib import Path
import statistics


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
