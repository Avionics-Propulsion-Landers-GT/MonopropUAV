import argparse
import csv
import math
from pathlib import Path


def resolve_csv_path(raw_path: str, project_root: Path) -> Path:
    candidate = Path(raw_path)
    if candidate.exists():
        return candidate

    project_candidate = project_root / raw_path
    if project_candidate.exists():
        return project_candidate

    raise FileNotFoundError(
        f"Could not find CSV '{raw_path}' as a direct path or under {project_root}"
    )


def read_numeric_csv(csv_path: Path) -> tuple[list[str], dict[str, list[float]]]:
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{csv_path} has no header row")

        fieldnames = reader.fieldnames
        columns: dict[str, list[float]] = {name: [] for name in fieldnames}

        for row_idx, row in enumerate(reader, start=2):
            for name in fieldnames:
                raw_value = (row.get(name) or "").strip()
                if raw_value == "":
                    raise ValueError(f"{csv_path}:{row_idx} column '{name}' is empty")
                try:
                    columns[name].append(float(raw_value))
                except ValueError as exc:
                    raise ValueError(
                        f"{csv_path}:{row_idx} column '{name}' is not numeric: {raw_value}"
                    ) from exc

    row_count = len(columns[fieldnames[0]])
    if row_count == 0:
        raise ValueError(f"{csv_path} has no data rows")
    return fieldnames, columns


def select_equally_spaced_indices(total_rows: int, nodes: int) -> list[int]:
    if total_rows <= 0:
        raise ValueError("total_rows must be positive")
    if nodes <= 0:
        raise ValueError("nodes must be positive")

    count = min(nodes, total_rows)
    if count == 1:
        return [0]

    return [int(i * (total_rows - 1) / (count - 1)) for i in range(count)]


def resample_columns(columns: dict[str, list[float]], indices: list[int]) -> dict[str, list[float]]:
    return {name: [values[i] for i in indices] for name, values in columns.items()}


def r2_score(ground_truth: list[float], candidate: list[float]) -> float:
    if len(ground_truth) != len(candidate):
        raise ValueError("ground_truth and candidate lengths must match")
    if not ground_truth:
        raise ValueError("ground_truth and candidate must not be empty")

    mean_ref = sum(ground_truth) / len(ground_truth)
    ss_res = sum((cand - truth) ** 2 for truth, cand in zip(ground_truth, candidate))
    ss_tot = sum((truth - mean_ref) ** 2 for truth in ground_truth)

    if ss_tot == 0.0:
        return 1.0 if ss_res == 0.0 else math.nan
    return 1.0 - (ss_res / ss_tot)


def compare_csvs(ground_truth_csv: Path, candidate_csv: Path, nodes: int) -> dict[str, float]:
    fields_truth, data_truth = read_numeric_csv(ground_truth_csv)
    fields_candidate, data_candidate = read_numeric_csv(candidate_csv)

    indices_truth = select_equally_spaced_indices(len(data_truth[fields_truth[0]]), nodes)
    indices_candidate = select_equally_spaced_indices(
        len(data_candidate[fields_candidate[0]]), nodes
    )
    sampled_truth = resample_columns(data_truth, indices_truth)
    sampled_candidate = resample_columns(data_candidate, indices_candidate)

    comparable_columns = [
        name for name in fields_truth if name != "t" and name in sampled_candidate
    ]
    if not comparable_columns:
        raise ValueError("No comparable columns found (excluding 't')")

    return {
        name: r2_score(sampled_truth[name], sampled_candidate[name])
        for name in comparable_columns
    }


def write_results(
    output_path: Path,
    ground_truth_csv: Path,
    candidate_csv: Path,
    metrics: dict[str, float],
) -> None:
    headers = ["ground_truth_csv", "candidate_csv"] + [f"r2_{name}" for name in metrics]
    row = [ground_truth_csv.name, candidate_csv.name] + [
        f"{metrics[name]:.10g}" for name in metrics
    ]

    needs_header = True
    if output_path.exists() and output_path.stat().st_size > 0:
        with output_path.open(newline="") as f:
            existing_header = next(csv.reader(f), [])
        if existing_header != headers:
            raise ValueError(
                f"{output_path} has incompatible header.\n"
                f"Existing: {existing_header}\n"
                f"Expected: {headers}"
            )
        needs_header = False

    with output_path.open("a" if output_path.exists() else "w", newline="") as f:
        writer = csv.writer(f)
        if needs_header:
            writer.writerow(headers)
        writer.writerow(row)


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Compare two trajectory CSVs using per-column R^2 on equally spaced samples."
    )
    parser.add_argument("ground_truth_csv", help="Ground-truth CSV path")
    parser.add_argument("candidate_csv", help="Candidate CSV path")
    parser.add_argument(
        "--nodes",
        type=int,
        default=100,
        help="Number of equally spaced rows to sample from each CSV (default: 100)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_root / "results.csv",
        help=f"Output CSV path (default: {project_root / 'results.csv'})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    project_root = Path(__file__).resolve().parents[1]

    ground_truth_csv = resolve_csv_path(args.ground_truth_csv, project_root)
    candidate_csv = resolve_csv_path(args.candidate_csv, project_root)

    metrics = compare_csvs(ground_truth_csv, candidate_csv, args.nodes)
    write_results(args.output, ground_truth_csv, candidate_csv, metrics)
    print(
        f"Saved comparison results to {args.output} "
        f"(ground truth: {ground_truth_csv.name}, candidate: {candidate_csv.name})"
    )


if __name__ == "__main__":
    main()
