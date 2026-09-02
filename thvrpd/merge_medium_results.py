from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-campaign", type=Path, required=True)
    parser.add_argument("--source-final", type=Path, required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    target = args.target_campaign.resolve()
    source = args.source_final.resolve()
    target_manifest = _read_json(target / "campaign_manifest.json")
    source_manifest = _read_json(source / "campaign_manifest.json")
    _validate_contracts(target_manifest, source_manifest)
    protected_small = _small_artifact_hashes(target)
    source_prefix = str(source)
    target_prefix = str(target)

    for path in (target / "instances").glob("medium_*.json"):
        path.unlink()
    for path in (target / "cases").glob("medium_*"):
        shutil.rmtree(path)
    for path in target.glob("medium_*_solution_process_report.*"):
        path.unlink()

    for source_snapshot in sorted((source / "instances").glob("medium_*.json")):
        shutil.copy2(source_snapshot, target / "instances" / source_snapshot.name)
    for source_case in sorted((source / "cases").glob("medium_*")):
        destination = target / "cases" / source_case.name
        shutil.copytree(source_case, destination)
        for json_path in destination.rglob("*.json"):
            payload = _read_json(json_path)
            _write_json(json_path, _replace_prefix(payload, source_prefix, target_prefix))

    small_records = [record for record in target_manifest["cases"] if record["case_id"].startswith("small_")]
    medium_records = [
        _replace_prefix(record, source_prefix, target_prefix)
        for record in source_manifest["cases"]
    ]
    manifest = {
        **target_manifest,
        "schema_version": max(target_manifest.get("schema_version", 1), source_manifest.get("schema_version", 1)),
        "case_count": 16,
        "solver_run_count": 32,
        "case_order": [record["case_id"] for record in [*small_records, *medium_records]],
        "cases": [*small_records, *medium_records],
        "medium_result_source": source_prefix,
        "medium_dimensions": medium_records[0]["dimensions"],
    }
    _write_json(target / "campaign_manifest.json", manifest)

    target_preparation = _read_json(target / "snapshot_preparation.json")
    source_preparation = _read_json(source / "snapshot_preparation.json")
    small_preparation = [
        record for record in target_preparation["cases"] if record["case_id"].startswith("small_")
    ]
    medium_preparation = [
        _replace_prefix(record, source_prefix, target_prefix)
        for record in source_preparation["cases"]
    ]
    _write_json(target / "snapshot_preparation.json", {"cases": [*small_preparation, *medium_preparation]})

    target_summary = _read_json(target / "campaign_summary.json")
    source_summary = _read_json(source / "campaign_summary.json")
    small_summary = [row for row in target_summary if row["case_id"].startswith("small_")]
    medium_summary = [_replace_prefix(row, source_prefix, target_prefix) for row in source_summary]
    _write_json(target / "campaign_summary.json", [*small_summary, *medium_summary])
    _write_json(
        target / "campaign_status.json",
        {
            "expected_runs": 32,
            "completed_runs": 32,
            "pending_runs": 0,
            "failed_attempts": 0,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
    )

    for name in (
        "dense_small_medium_comparison.json",
        "dense_small_medium_comparison.csv",
        "dense_small_medium_comparison.md",
        "artifact_inventory.csv",
        "medium_results_deleted.json",
    ):
        path = target / name
        if path.exists():
            path.unlink()
    for case_id in (record["case_id"] for record in medium_records):
        subprocess.run(
            [
                sys.executable,
                "-m",
                "thvrpd.single_case_solution_report",
                "--campaign-dir",
                str(target),
                "--case-id",
                case_id,
            ],
            check=True,
            cwd=Path.cwd(),
        )
    from .dense_multiscale_campaign import _write_aggregate_report

    _write_aggregate_report(target, manifest)
    _write_json(
        target / "medium_results_replaced.json",
        {
            "replaced_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "source_final": source_prefix,
            "medium_case_ids": [record["case_id"] for record in medium_records],
            "medium_dimensions": medium_records[0]["dimensions"],
            "source_snapshot_hashes": {
                record["case_id"]: record["instance_snapshot_sha256"] for record in medium_records
            },
            "small_artifacts_verified": True,
        },
    )
    _write_json(
        target / "monitor_reported_cases.json",
        {
            "reported_cases": manifest["case_order"],
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "campaign_complete": True,
        },
    )
    if _small_artifact_hashes(target) != protected_small:
        raise RuntimeError("small-case artifacts changed during medium-result merge")
    from .dense_large_campaign import _artifact_inventory, _write_csv

    _write_csv(target / "artifact_inventory.csv", _artifact_inventory(target))


def _validate_contracts(target_manifest, source_manifest) -> None:
    target_ids = target_manifest.get("case_order", [])
    source_ids = source_manifest.get("case_order", [])
    if len(target_ids) != 8 or any(not case_id.startswith("small_") for case_id in target_ids):
        raise ValueError("target campaign must contain exactly eight small cases")
    if len(source_ids) != 8 or any(not case_id.startswith("medium_") for case_id in source_ids):
        raise ValueError("source campaign must contain exactly eight medium cases")
    if target_manifest["time_limit_seconds_per_run"] != source_manifest["time_limit_seconds_per_run"]:
        raise ValueError("source and target solver time limits differ")
    if target_manifest["threads"] != source_manifest["threads"]:
        raise ValueError("source and target thread settings differ")
    if target_manifest["truck_arc_probability"] != source_manifest["truck_arc_probability"]:
        raise ValueError("source and target truck probabilities differ")
    if target_manifest["hub_arc_probability"] != source_manifest["hub_arc_probability"]:
        raise ValueError("source and target hub probabilities differ")


def _small_artifact_hashes(campaign: Path) -> dict[str, str]:
    paths = []
    paths.extend((campaign / "instances").glob("small_*.json"))
    for case_dir in (campaign / "cases").glob("small_*"):
        paths.extend(path for path in case_dir.rglob("*") if path.is_file())
    paths.extend(campaign.glob("small_*_solution_process_report.*"))
    return {
        str(path.relative_to(campaign)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(paths)
    }


def _replace_prefix(value, source_prefix: str, target_prefix: str):
    if isinstance(value, str):
        return value.replace(source_prefix, target_prefix)
    if isinstance(value, list):
        return [_replace_prefix(item, source_prefix, target_prefix) for item in value]
    if isinstance(value, dict):
        return {key: _replace_prefix(item, source_prefix, target_prefix) for key, item in value.items()}
    return value


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=True), encoding="utf-8")


if __name__ == "__main__":
    main()
