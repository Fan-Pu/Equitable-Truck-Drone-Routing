from __future__ import annotations

import csv
import json
from pathlib import Path


def load_manual_service_deadline_bounds(path: Path) -> dict[str, float]:
    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(payload, dict) and "deadlines" in payload:
            payload = payload["deadlines"]
        if not isinstance(payload, dict):
            raise ValueError("manual service deadline JSON must be a customer-to-bound object")
        return {str(customer): float(bound) for customer, bound in payload.items()}
    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames is None or "customer" not in reader.fieldnames or "deadline" not in reader.fieldnames:
                raise ValueError("manual service deadline CSV must contain customer and deadline columns")
            return {str(row["customer"]): float(row["deadline"]) for row in reader}
    raise ValueError("manual service deadline file must be JSON or CSV")
