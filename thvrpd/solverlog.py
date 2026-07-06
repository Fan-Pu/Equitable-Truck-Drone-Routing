from __future__ import annotations

from pathlib import Path


def configure_gurobi_logging(model, log_file: str | None) -> None:
    if log_file is None:
        model.Params.OutputFlag = 0
        return
    path = Path(log_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    model.Params.LogToConsole = 0
    model.Params.LogFile = str(path)
