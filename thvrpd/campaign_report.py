from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-dir", type=Path, required=True)
    args = parser.parse_args()
    build_campaign_report(args.campaign_dir)


def build_campaign_report(campaign_dir: Path) -> None:
    manifest = _read_json(campaign_dir / "campaign_manifest.json")
    status = _read_json(campaign_dir / "campaign_status.json")
    if status["completed_runs"] != manifest["solver_run_count"] or status["pending_runs"] != 0:
        raise ValueError("campaign is incomplete")
    summary = _read_json(campaign_dir / "campaign_summary.json")
    summary_by_case_solver = {(row["case_id"], row["solver"]): row for row in summary}

    bpc_rows: list[dict[str, Any]] = []
    compact_rows: list[dict[str, Any]] = []
    pricing_rows: list[dict[str, Any]] = []
    tree_rows: list[dict[str, Any]] = []
    comparison_rows: list[dict[str, Any]] = []
    checks: list[dict[str, Any]] = []

    for case in manifest["cases"]:
        case_id = case["case_id"]
        bpc_summary = summary_by_case_solver[(case_id, "bpc")]
        compact_summary = summary_by_case_solver[(case_id, "compact")]
        bpc_path = Path(bpc_summary["result_file"])
        compact_path = Path(compact_summary["result_file"])
        bpc = _read_json(bpc_path)
        compact = _read_json(compact_path)
        base = {
            "case_id": case_id,
            "scale": case["scale"],
            "distribution": case["distribution"],
            "seed": case["seed"],
        }
        bpc_rows.append({
            **base,
            **{f"result_{key}": value for key, value in bpc.items() if key not in {"bpc_stats", "routes"}},
            **{f"bpc_{key}": value for key, value in bpc["bpc_stats"].items() if key != "pricing_diagnostics"},
            "result_routes": bpc.get("routes", []),
        })
        compact_rows.append({**base, **compact})
        for index, diagnostic in enumerate(bpc["bpc_stats"]["pricing_diagnostics"], 1):
            pricing_rows.append({**base, "pricing_call_index": index, **diagnostic})
        tree_path = bpc_path.parent / "bpc_progress.jsonl"
        for line in tree_path.read_text(encoding="utf-8").splitlines():
            event = json.loads(line)
            node = event["node"]
            tree_rows.append({
                **base,
                "event_sequence": event["event_sequence"],
                "event": event["event"],
                "updated_at": event["updated_at"],
                **{f"node_{key}": value for key, value in node.items()},
                **{f"stat_{key}": value for key, value in event["stats"].items()},
                "extra": event.get("extra"),
            })
        comparison_rows.append({
            **base,
            "bpc_status": bpc["status"],
            "compact_status": compact["status"],
            "bpc_objective_full": bpc.get("objective_full"),
            "compact_objective_full": compact.get("objective_full"),
            "bpc_minus_compact_objective": _difference(bpc.get("objective_full"), compact.get("objective_full")),
            "bpc_lower_bound_full": bpc.get("lower_bound_full"),
            "compact_bound_full": compact.get("objective_bound_full"),
            "bpc_gap": bpc.get("gap_full"),
            "compact_gap": compact.get("mip_gap"),
            "bpc_runtime": bpc.get("runtime"),
            "compact_runtime": compact.get("runtime"),
            "bpc_nodes": bpc.get("nodes_processed"),
            "compact_nodes": compact.get("node_count"),
            "bpc_drone_sorties": bpc.get("service_metrics", {}).get("drone_sorties"),
            "compact_drone_sorties": compact.get("service_metrics", {}).get("drone_sorties"),
            "bpc_mean_core_equivalent": bpc_summary["mean_core_equivalent"],
            "compact_mean_core_equivalent": compact_summary["mean_core_equivalent"],
        })
        bpc_log = bpc_path.parent / "gurobi_logs" / "root_compact.log"
        compact_log = compact_path.parent / "gurobi_logs" / "compact_arc.log"
        checks.append({
            **base,
            "instance_configs_match": bpc["instance_config"] == compact["instance_config"],
            "bpc_service_feasible": bpc.get("service_metrics", {}).get("service_feasible"),
            "bpc_payload_feasible": bpc.get("service_metrics", {}).get("payload_feasible"),
            "compact_service_feasible": compact.get("service_metrics", {}).get("service_feasible"),
            "compact_payload_feasible": compact.get("service_metrics", {}).get("payload_feasible"),
            "bpc_tree_event_count": sum(1 for _ in tree_path.open("r", encoding="utf-8")),
            "bpc_pricing_diagnostic_count": len(bpc["bpc_stats"]["pricing_diagnostics"]),
            "bpc_warm_start_log_nonempty": bpc_log.stat().st_size > 0,
            "compact_gurobi_log_nonempty": compact_log.stat().st_size > 0,
            "bpc_bound_not_above_incumbent": (
                bpc.get("lower_bound_full") is None
                or bpc.get("objective_full") is None
                or bpc["lower_bound_full"] <= bpc["objective_full"] + 1e-8
            ),
            "compact_optimum_not_worse_than_bpc_incumbent": (
                compact.get("status_code") != 2
                or compact.get("objective_full") is None
                or bpc.get("objective_full") is None
                or compact["objective_full"] <= bpc["objective_full"] + 1e-8
            ),
        })

    _write_csv(campaign_dir / "bpc_full_stats.csv", bpc_rows)
    _write_csv(campaign_dir / "compact_full_stats.csv", compact_rows)
    _write_csv(campaign_dir / "bpc_pricing_calls.csv", pricing_rows)
    _write_csv(campaign_dir / "bpc_tree_evolution.csv", tree_rows)
    _write_csv(campaign_dir / "case_comparison.csv", comparison_rows)
    inventory = _artifact_inventory(campaign_dir)
    _write_csv(campaign_dir / "artifact_inventory.csv", inventory)
    verification = {
        "case_count": len(manifest["cases"]),
        "solver_run_count": len(summary),
        "bpc_pricing_call_count": len(pricing_rows),
        "bpc_tree_event_count": len(tree_rows),
        "artifact_count": len(inventory),
        "all_instance_configs_match": all(check["instance_configs_match"] for check in checks),
        "all_bpc_solutions_feasible": all(
            check["bpc_service_feasible"] and check["bpc_payload_feasible"] for check in checks
        ),
        "all_compact_incumbents_feasible": all(
            check["compact_service_feasible"] and check["compact_payload_feasible"] for check in checks
        ),
        "all_bpc_histories_nonempty": all(check["bpc_tree_event_count"] > 0 for check in checks),
        "all_gurobi_logs_nonempty": all(
            check["bpc_warm_start_log_nonempty"] and check["compact_gurobi_log_nonempty"] for check in checks
        ),
        "all_bpc_bounds_not_above_incumbents": all(check["bpc_bound_not_above_incumbent"] for check in checks),
        "all_compact_optima_not_worse_than_bpc_incumbents": all(
            check["compact_optimum_not_worse_than_bpc_incumbent"] for check in checks
        ),
        "case_checks": checks,
    }
    (campaign_dir / "verification_report.json").write_text(json.dumps(verification, indent=2), encoding="utf-8")


def _difference(left: float | None, right: float | None) -> float | None:
    if left is None or right is None:
        return None
    return left - right


def _artifact_inventory(campaign_dir: Path) -> list[dict[str, Any]]:
    excluded = {"artifact_inventory.csv"}
    rows = []
    for path in sorted(path for path in campaign_dir.rglob("*") if path.is_file() and path.name not in excluded):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        rows.append({
            "relative_path": str(path.relative_to(campaign_dir)),
            "size_bytes": path.stat().st_size,
            "sha256": digest,
        })
    return rows


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key)) for key in fieldnames})


def _csv_value(value: Any) -> Any:
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, separators=(",", ":"))
    return value


if __name__ == "__main__":
    main()
