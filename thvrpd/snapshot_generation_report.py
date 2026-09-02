from __future__ import annotations

import argparse
import json
from pathlib import Path

from .campaign import _expected_instance_config, campaign_cases
from .instance import read_instance_snapshot, validate_instance_case


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-dir", type=Path, required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    campaign_dir = args.campaign_dir.resolve()
    instances_dir = campaign_dir / "instances"
    cases = campaign_cases()
    expected_names = {f"{case.case_id}.json" for case in cases}
    actual_names = {path.name for path in instances_dir.glob("*.json")}
    if actual_names != expected_names:
        raise RuntimeError(
            f"snapshot set mismatch: missing={sorted(expected_names - actual_names)}, "
            f"unexpected={sorted(actual_names - expected_names)}"
        )

    manifest = json.loads((campaign_dir / "campaign_manifest.json").read_text(encoding="utf-8"))
    manifest_hashes = {
        record["case_id"]: record["instance_snapshot_sha256"]
        for record in manifest["cases"]
    }
    records = []
    for case in cases:
        path = instances_dir / f"{case.case_id}.json"
        instance, digest = read_instance_snapshot(path, expected_sha256=manifest_hashes[case.case_id])
        expected_config = _expected_instance_config(case)
        validate_instance_case(
            instance,
            requested_seed=case.seed,
            num_trucks=case.dimensions["num_trucks"],
            num_customers=case.dimensions["num_customers"],
            num_hubs=case.dimensions["num_hubs"],
            distribution=case.distribution,
            drones_per_truck=case.dimensions["drones_per_truck"],
            expected_config=expected_config,
        )
        if not instance.generation_feasibility_diagnostics:
            raise RuntimeError(f"{case.case_id} has no generation diagnostics")
        accepted = instance.generation_feasibility_diagnostics[-1]
        status = str(accepted["status"])
        if status not in {"feasible_unchanged", "repaired"}:
            raise RuntimeError(f"{case.case_id} has invalid accepted status {status}")
        diagnostics = accepted.get("diagnostics", {})
        hamming_edits = 0
        projection = None
        certificate_valid = True
        if status == "repaired":
            if diagnostics.get("projection_type") != "strengthened_arc":
                raise RuntimeError(f"{case.case_id} did not use strengthened-arc projection")
            certificate = diagnostics.get("certificate_validation", {})
            if certificate.get("valid") is not True:
                raise RuntimeError(f"{case.case_id} has an invalid repair certificate")
            if diagnostics.get("original_arc_counts") != diagnostics.get("repaired_arc_counts"):
                raise RuntimeError(f"{case.case_id} changed an internal arc-class count")
            mandatory = set(instance.mandatory_drone_customers)
            if any(left in mandatory or right in mandatory for left, right in instance.truck_arcs):
                raise RuntimeError(f"{case.case_id} has a truck arc incident to a mandatory-drone customer")
            hamming_edits = int(diagnostics["hamming_edit_count"])
            projection = diagnostics.get("strengthened_arc_projection")
            certificate_valid = True

        records.append(
            {
                "case_id": case.case_id,
                "scale": case.scale,
                "distribution": case.distribution,
                "requested_seed": case.seed,
                "realized_seed": instance.config.seed,
                "generation_attempt": instance.generation_attempt,
                "generation_seconds": instance.generation_feasibility_time,
                "accepted_status": status,
                "repaired": status == "repaired",
                "hamming_edit_count": hamming_edits,
                "projection": projection,
                "certificate_valid": certificate_valid,
                "truck_arc_count": len(instance.truck_arcs),
                "drone_arc_count": len(instance.drone_arcs),
                "mandatory_drone_customer_count": len(instance.mandatory_drone_customers),
                "snapshot_file": str(path),
                "snapshot_sha256": digest,
            }
        )

    attempt_dirs = tuple(campaign_dir.glob("cases/**/attempt_*"))
    if attempt_dirs:
        raise RuntimeError(f"solver attempt directories exist despite snapshot-only generation: {attempt_dirs}")
    report = {
        "campaign_dir": str(campaign_dir),
        "snapshot_count": len(records),
        "solver_attempt_count": 0,
        "repaired_count": sum(record["repaired"] for record in records),
        "unchanged_count": sum(not record["repaired"] for record in records),
        "total_generation_seconds": sum(record["generation_seconds"] for record in records),
        "maximum_generation_seconds": max(record["generation_seconds"] for record in records),
        "total_hamming_edits": sum(record["hamming_edit_count"] for record in records),
        "cases": records,
    }
    (campaign_dir / "snapshot_generation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )
    (campaign_dir / "snapshot_generation_report.md").write_text(
        _markdown(report),
        encoding="utf-8",
    )


def _markdown(report: dict[str, object]) -> str:
    lines = [
        "# Current-Paper Snapshot Generation Report",
        "",
        f"- Snapshots: {report['snapshot_count']}",
        f"- Solver attempts: {report['solver_attempt_count']}",
        f"- Repaired: {report['repaired_count']}",
        f"- Unchanged: {report['unchanged_count']}",
        f"- Total feasibility-generation time: {report['total_generation_seconds']:.3f} s",
        f"- Maximum case generation time: {report['maximum_generation_seconds']:.3f} s",
        f"- Total Hamming edits: {report['total_hamming_edits']}",
        "",
        "| Case | Status | Realized seed | Attempt | Time (s) | Edits | Certificate | SHA-256 |",
        "|---|---|---:|---:|---:|---:|---|---|",
    ]
    for record in report["cases"]:
        lines.append(
            f"| {record['case_id']} | {record['accepted_status']} | {record['realized_seed']} | "
            f"{record['generation_attempt']} | {record['generation_seconds']:.3f} | "
            f"{record['hamming_edit_count']} | {record['certificate_valid']} | "
            f"`{record['snapshot_sha256']}` |"
        )
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
