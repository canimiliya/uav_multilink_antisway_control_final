"""Freeze post-Self-freeze ablation and representative trace protocol."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SELF = ROOT / "reproducibility/v5/self"
SELF_FREEZE_HEAD = "7d13b06cba1594b8d46c7d0e1688ffb578371ba4"


def read(name: str) -> dict: return json.loads((SELF / name).read_text(encoding="utf-8"))
def git(*args: str) -> str: return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def main() -> int:
    if git("rev-parse", "HEAD") != SELF_FREEZE_HEAD: raise RuntimeError("ablation protocol must directly follow Self freeze")
    frozen = read("self_freeze.json"); full = dict(frozen["parameters"])
    shock = {**full, "candidate_id": "ablation_shock_only", "conflict_gain": 0.0, "cancellation_gain": 0.0, "slew_reserve_fraction": 0.0, "amplitude_reserve_fraction": 0.0, "offset_disengage_rate": 1.0, "offset_engage_rate": 1.0}
    bumpless = {**shock, "candidate_id": "ablation_plus_bumpless", "offset_disengage_rate": full["offset_disengage_rate"], "offset_engage_rate": full["offset_engage_rate"]}
    coordination = {**bumpless, "candidate_id": "ablation_plus_coordination_headroom", "cancellation_gain": full["cancellation_gain"], "slew_reserve_fraction": full["slew_reserve_fraction"], "amplitude_reserve_fraction": full["amplitude_reserve_fraction"]}
    payload = {
        "status": "FROZEN_AFTER_SELF_FREEZE_BEFORE_ABLATION_PERFORMANCE", "self_freeze_head": SELF_FREEZE_HEAD,
        "self_candidate_unchanged": frozen["candidate_id"], "selection_authority": False, "return_to_tuning_forbidden": True,
        "cumulative_levels": [
            {"id": "legacy_self_a_034", "source": "frozen baseline reuse", "meaning": "legacy Self"},
            {"id": "cart_b_014", "source": "frozen V5 baseline reuse", "meaning": "CART mechanisms"},
            {"id": "ablation_shock_only", "source": "new post-freeze diagnostic", "meaning": "+ causal shock detection and direct robust coordination", "parameters": shock},
            {"id": "ablation_plus_bumpless", "source": "new post-freeze diagnostic", "meaning": "+ rate-limited bumpless engagement", "parameters": bumpless},
            {"id": "ablation_plus_coordination_headroom", "source": "new post-freeze diagnostic", "meaning": "+ cancellation coordination and slew reserve", "parameters": coordination},
            {"id": frozen["candidate_id"], "source": "frozen Stage-C evidence", "meaning": "+ geometric disturbance-task conflict index (full SATC)", "parameters": full},
        ],
        "new_diagnostic_run_candidates": ["ablation_shock_only", "ablation_plus_bumpless", "ablation_plus_coordination_headroom"],
        "sample_bank": "unchanged 120-sample V5 Development",
        "representative_traces": {
            "normal": "v5d_calm_00", "strong_aligned": "v5d_strong_simultaneous_aligned_00",
            "strong_opposed": "v5d_strong_simultaneous_opposed_00", "strong_cross": "v5d_strong_simultaneous_cross_00",
        },
        "trace_controller": frozen["candidate_id"], "holdout_executed": False,
    }
    path = SELF / "ablation_protocol.json"
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"new_ablation_candidates": 3, "trace_cases": 4, "selection_authority": False}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
