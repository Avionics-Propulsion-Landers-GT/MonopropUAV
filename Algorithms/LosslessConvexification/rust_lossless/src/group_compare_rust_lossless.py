import argparse
import csv
import re
from pathlib import Path


def resolve_directory_path(raw_path: str, project_root: Path) -> Path:
    candidate = Path(raw_path)
    if candidate.exists() and candidate.is_dir():
        return candidate

    project_candidate = project_root / raw_path
    if project_candidate.exists() and project_candidate.is_dir():
        return project_candidate

    raise FileNotFoundError(
        f"Could not find directory '{raw_path}' as a direct path or under {project_root}"
    )


def run_number_from_candidate(candidate_csv: str) -> int:
    match = re.search(r"_(\d+)\.csv$", candidate_csv)
    if not match:
        raise ValueError(f"Could not parse run number from candidate CSV '{candidate_csv}'")
    return int(match.group(1))


def read_comparison_rows(csv_path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{csv_path} has no header row")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{csv_path} has no data rows")
    return reader.fieldnames, rows


def pick_candidate_row(
    rows: list[dict[str, str]],
    group_name: str,
    run_type: str,
    method: str,
    run_id: int,
) -> dict[str, str]:
    prefix = f"trajectory_{method}_{group_name}_{run_type}_"
    exact = f"{prefix}{run_id}.csv"

    exact_match = next((row for row in rows if row.get("candidate_csv", "") == exact), None)
    if exact_match is not None:
        return exact_match

    candidates = [
        row for row in rows if row.get("candidate_csv", "").startswith(prefix)
    ]
    if not candidates:
        raise ValueError(
            f"No {method} candidate rows found for run_type '{run_type}' "
            f"with prefix '{prefix}'"
        )

    return min(candidates, key=lambda row: run_number_from_candidate(row["candidate_csv"]))


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Collect one zoh and one cgl comparison result from each run type "
            "(default: short, long, med) into a single group_comparison_results.csv."
        )
    )
    parser.add_argument(
        "group_directory",
        help="Group directory containing run-type subdirectories (example: direct_descent)",
    )
    parser.add_argument(
        "--run-types",
        nargs="+",
        default=["short", "long", "med"],
        help="Run types to include (default: short long med)",
    )
    parser.add_argument(
        "--run-id",
        type=int,
        default=1,
        help="Run id to pick per method and run type (default: 1)",
    )
    parser.add_argument(
        "--comparison-file",
        default="comparison_results.csv",
        help="Per-run-type comparison filename (default: comparison_results.csv)",
    )
    parser.add_argument(
        "--output-name",
        default="group_comparison_results.csv",
        help="Output filename inside the group directory (default: group_comparison_results.csv)",
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=project_root,
        help=f"Project root used for path resolution (default: {project_root})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.run_id <= 0:
        raise ValueError("--run-id must be positive")

    project_root = args.project_root.resolve()
    group_dir = resolve_directory_path(args.group_directory, project_root).resolve()
    group_name = group_dir.name
    output_path = (group_dir / args.output_name).resolve()

    selected_rows: list[dict[str, str]] = []
    output_headers: list[str] | None = None

    for run_type in args.run_types:
        run_type_dir = group_dir / run_type
        if not run_type_dir.exists() or not run_type_dir.is_dir():
            raise FileNotFoundError(f"Run type directory not found: {run_type_dir}")

        comparison_path = run_type_dir / args.comparison_file
        if not comparison_path.exists() or not comparison_path.is_file():
            raise FileNotFoundError(f"Comparison CSV not found: {comparison_path}")

        headers, rows = read_comparison_rows(comparison_path)

        if output_headers is None:
            output_headers = ["run_type"] + headers
        elif headers != output_headers[1:]:
            raise ValueError(
                f"Header mismatch in {comparison_path}.\n"
                f"Expected: {output_headers[1:]}\n"
                f"Found: {headers}"
            )

        for method in ("zoh", "cgl"):
            picked = pick_candidate_row(rows, group_name, run_type, method, args.run_id)
            selected_rows.append({"run_type": run_type, **picked})

    if output_headers is None:
        raise RuntimeError("No rows selected")

    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=output_headers)
        writer.writeheader()
        writer.writerows(selected_rows)

    print(f"Saved {len(selected_rows)} rows to {output_path}")


if __name__ == "__main__":
    main()
