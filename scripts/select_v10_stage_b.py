"""Apply the frozen Stage-A ranking and select exactly sixteen V10 candidates."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v10/development"


def main() -> int:
    payload = json.loads((OUT / "stage_a_summary.json").read_text(encoding="utf-8"))
    if payload["candidate_count"] != 64 or payload["sample_count_each"] != 12:
        raise RuntimeError("Stage A evidence drift")

    def rank(row: dict) -> tuple:
        strong = row["cohorts"]["STRONG_NEAR_SIMULTANEOUS"]["position_rmse_3d_m"] + row["cohorts"]["STRONG_PREEXISTING"]["position_rmse_3d_m"]
        return (
            -row["safety_rate"], -row["success_rate"], row["position_rmse_3d_m"], strong,
            row["orientation_rmse_deg"], row["total_acceleration_effort"], row["solve_time_p95_ms"], row["candidate_id"],
        )

    ranked = sorted(payload["summaries"], key=rank)
    result = {
        "source": "stage_a_summary.json", "selection_rule": "finite/stable implicit in completed cases; safety, success, position, strong position, orientation, effort, runtime, id",
        "selected_ids": [row["candidate_id"] for row in ranked[:16]],
        "ranked": [{"rank": index, "candidate_id": row["candidate_id"], "safety": row["safety_rate"], "success": row["success_rate"], "position": row["position_rmse_3d_m"], "runtime_ms": row["solve_time_p95_ms"]} for index, row in enumerate(ranked, 1)],
        "holdout_accessed": False,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "stage_b_selection.json").open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"selected": result["selected_ids"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
