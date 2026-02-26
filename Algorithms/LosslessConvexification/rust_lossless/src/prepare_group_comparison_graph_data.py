import argparse
import csv
import math
import re
import sys
from pathlib import Path


METHOD_ORDER = {"zoh": 0, "cgl": 1}
RUN_TYPE_ORDER = {
    "ultra_short": 0,
    "very_short": 1,
    "short": 2,
    "med": 3,
    "long": 4,
    "extreme_long": 5,
}


def parse_args() -> argparse.Namespace:
    default_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Collect all group_comparison_results.csv files into a clean graphing CSV "
            "with columns: group_name, run_type, method, rmse_xyz, rmse_sigma, "
            "avg_solve_time, std_error_solve_time, time_of_flight_s."
        )
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=default_root,
        help=f"Root directory to scan (default: {default_root})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_root / "clean_group_graph_data.csv",
        help=(
            "Output CSV path "
            f"(default: {default_root / 'clean_group_graph_data.csv'})"
        ),
    )
    return parser.parse_args()


def method_from_candidate(candidate_csv: str) -> str:
    match = re.match(r"^trajectory_([^_]+)_", candidate_csv)
    if not match:
        raise ValueError(f"Could not parse method from candidate CSV '{candidate_csv}'")
    return match.group(1)


def read_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{csv_path} has no header row")

        required = {"run_type", "candidate_csv", "rmse_xyz", "rmse_sigma"}
        missing = required.difference(reader.fieldnames)
        if missing:
            raise ValueError(
                f"{csv_path} is missing required columns: {sorted(missing)}"
            )

        return list(reader)


def sort_key(row: dict[str, str]) -> tuple[int, str, int, int]:
    method = row["method"]
    run_type = row["run_type"]
    return (
        METHOD_ORDER.get(method, len(METHOD_ORDER)),
        row["group_name"],
        METHOD_ORDER.get(method, len(METHOD_ORDER)),
        RUN_TYPE_ORDER.get(run_type, len(RUN_TYPE_ORDER)),
    )


def standard_error(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance) / math.sqrt(len(values))


def read_simple_solve_stats(run_type_dir: Path) -> dict[str, tuple[str, str]]:
    exact = run_type_dir / "simple_solve_metrics.csv"
    if exact.exists() and exact.is_file():
        target = exact
    else:
        candidates = sorted(run_type_dir.glob("simple_solve_metrics*.csv"))
        if not candidates:
            return {}
        target = candidates[0]

    with target.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return {}
        required = {"group_name", "run_name", "fine_clarabel_solve_time_s"}
        if not required.issubset(reader.fieldnames):
            return {}
        rows = list(reader)

    values_by_group: dict[str, list[float]] = {}
    for row in rows:
        group = (row.get("group_name") or "").strip()
        run_name = (row.get("run_name") or "").strip()
        fine_time_raw = (row.get("fine_clarabel_solve_time_s") or "").strip()
        if not group or not fine_time_raw:
            continue
        if run_name.startswith("set_") and run_name.endswith("_avg"):
            continue
        try:
            fine_time = float(fine_time_raw)
        except ValueError:
            continue
        values_by_group.setdefault(group, []).append(fine_time)

    stats: dict[str, tuple[str, str]] = {}
    for group, values in values_by_group.items():
        if not values:
            continue
        avg = sum(values) / len(values)
        stderr = standard_error(values)
        stats[group] = (f"{avg:.6f}", f"{stderr:.6f}")
    return stats


def read_last_fine_time_of_flight(
    method_dir: Path, method: str, group_name: str, run_type: str
) -> str:
    solve_metrics_path = method_dir / "solve_metrics.csv"
    if not solve_metrics_path.exists() or not solve_metrics_path.is_file():
        return ""

    with solve_metrics_path.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return ""
        required = {"run_name", "pass", "time_of_flight_s"}
        if not required.issubset(reader.fieldnames):
            return ""

        prefix = f"{method}_{group_name}_{run_type}_"
        last_time_of_flight = ""
        for row in reader:
            if (row.get("pass") or "").strip() != "fine":
                continue
            run_name = (row.get("run_name") or "").strip()
            if not run_name.startswith(prefix):
                continue
            tof_raw = (row.get("time_of_flight_s") or "").strip()
            if tof_raw:
                last_time_of_flight = tof_raw

    return last_time_of_flight


def main() -> None:
    args = parse_args()
    root = args.root.resolve()
    output_path = args.output.resolve()

    collected: list[dict[str, str]] = []
    skipped_files = 0
    simple_stats_cache: dict[tuple[Path, str], dict[str, tuple[str, str]]] = {}
    tof_cache: dict[tuple[Path, str, str], str] = {}

    comparison_files = sorted(root.rglob("group_comparison_results.csv"))
    for comparison_csv in comparison_files:
        if not comparison_csv.is_file():
            continue

        group_name = comparison_csv.parent.name
        try:
            rows = read_rows(comparison_csv)
            for row in rows:
                candidate_csv = (row.get("candidate_csv") or "").strip()
                run_type = (row.get("run_type") or "").strip()
                method = method_from_candidate(candidate_csv)

                simple_cache_key = (comparison_csv.parent, run_type)
                if simple_cache_key not in simple_stats_cache:
                    simple_stats_cache[simple_cache_key] = read_simple_solve_stats(
                        comparison_csv.parent / run_type
                    )
                simple_group_name = f"{method}_{group_name}_{run_type}"
                avg_solve_time, std_error_solve_time = simple_stats_cache[
                    simple_cache_key
                ].get(simple_group_name, ("", ""))

                tof_key = (comparison_csv.parent, run_type, method)
                if tof_key not in tof_cache:
                    tof_cache[tof_key] = read_last_fine_time_of_flight(
                        comparison_csv.parent / run_type / method,
                        method,
                        group_name,
                        run_type,
                    )
                time_of_flight_s = tof_cache[tof_key]

                collected.append(
                    {
                        "group_name": group_name,
                        "run_type": run_type,
                        "method": method,
                        "rmse_xyz": (row.get("rmse_xyz") or "").strip(),
                        "rmse_sigma": (row.get("rmse_sigma") or "").strip(),
                        "avg_solve_time": avg_solve_time,
                        "std_error_solve_time": std_error_solve_time,
                        "time_of_flight_s": time_of_flight_s,
                    }
                )
        except ValueError as exc:
            skipped_files += 1
            print(f"Skipping {comparison_csv}: {exc}", file=sys.stderr)

    if not collected:
        raise RuntimeError(f"No rows collected from {root}")

    collected.sort(key=sort_key)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "group_name",
                "run_type",
                "method",
                "rmse_xyz",
                "rmse_sigma",
                "avg_solve_time",
                "std_error_solve_time",
                "time_of_flight_s",
            ],
        )
        writer.writeheader()
        writer.writerows(collected)

    print(
        f"Saved {len(collected)} rows from {len(comparison_files)} files to {output_path}"
    )
    if skipped_files:
        print(f"Skipped {skipped_files} files due to format issues.", file=sys.stderr)


if __name__ == "__main__":
    main()
