#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def read_xy(path: Path):
    points = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row["x"]), float(row["y"])))
    return points


def evaluate(estimate, reference):
    n = min(len(estimate), len(reference))
    if n == 0:
        raise ValueError("empty trajectory")

    errors = []
    for i in range(n):
        dx = estimate[i][0] - reference[i][0]
        dy = estimate[i][1] - reference[i][1]
        errors.append(math.hypot(dx, dy))

    rmse = math.sqrt(sum(e * e for e in errors) / n)
    max_error = max(errors)
    drift = errors[-1]
    return rmse, max_error, drift


def main():
    parser = argparse.ArgumentParser(description="Evaluate 2D localization trajectory error.")
    parser.add_argument("--estimate", required=True, type=Path, help="CSV with x,y columns")
    parser.add_argument("--reference", required=True, type=Path, help="CSV with x,y columns")
    args = parser.parse_args()

    rmse, max_error, drift = evaluate(read_xy(args.estimate), read_xy(args.reference))
    print(f"RMSE: {rmse:.4f} m")
    print(f"Maximum error: {max_error:.4f} m")
    print(f"Drift: {drift:.4f} m")


if __name__ == "__main__":
    main()
