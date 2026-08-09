"""Run frozen post-freeze ablations and four SATC mechanism traces."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner
from uav_sway.v5.satc_ofmpc import SATCOFMPC


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v5/r0"
SELF = ROOT / "reproducibility/v5/self"
TRACE_ROWS: list[dict] = []


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


class TracingSATC(SATCOFMPC):
    def command(self, observation, reference, dt=0.05):
        command = super().command(observation, reference, dt); d = self.diagnostics; state = np.asarray(observation.full_state_error)
        axis = np.asarray(observation.task_state.cutter_axis_world)
        row = {
            "time_s": len(TRACE_ROWS) * float(dt), "shock_score": d.shock_score, "reference_shock": d.reference_shock,
            "innovation_shock": d.innovation_shock, "innovation_norm": d.innovation_norm, "innovation_rate": d.innovation_rate,
            "residual_unrepresented_norm": d.residual_unrepresented_norm, "trust": d.trust, "conflict_index": d.conflict_index,
            "cancellation_index": d.cancellation_index, "offset_engagement": d.offset_engagement, "coordination_weight": d.coordination_weight,
            "slew_headroom": d.slew_headroom, "amplitude_headroom": d.amplitude_headroom, "constraint_activity": d.constraint_activity,
            "steady_target_state_norm": float(np.linalg.norm(d.steady_target_state)),
            "steady_command_x": d.steady_command[0], "steady_command_y": d.steady_command[1], "steady_command_z": d.steady_command[2],
            "backbone_x": d.backbone_command[0], "backbone_y": d.backbone_command[1], "backbone_z": d.backbone_command[2],
            "mpc_x": d.mpc_command[0], "mpc_y": d.mpc_command[1], "mpc_z": d.mpc_command[2],
            "final_x": command[0], "final_y": command[1], "final_z": command[2],
            "position_error_m": float(np.linalg.norm(observation.task_state.tip_position_world - reference.tip_position_world)),
            "orientation_error_deg": float(np.rad2deg(np.arccos(np.clip(axis @ np.array([1.0, 0.0, 0.0]), -1.0, 1.0)))),
            "joint_angle_max_deg": float(np.rad2deg(np.max(np.abs(state[10:15])))), "joint_rate_max_rad_s": float(np.max(np.abs(state[15:20]))),
        }
        TRACE_ROWS.append(row); return command


def main() -> int:
    protocol = read(SELF / "ablation_protocol.json"); manifest = read(R0 / "development_manifest.json"); samples = manifest["samples"]
    all_rows = []; summaries = []
    for level in protocol["cumulative_levels"]:
        if level["id"] not in protocol["new_diagnostic_run_candidates"]: continue
        summary, values = runner.run_bank("satc_ofmpc", level["parameters"], samples, "ablation", 24)
        summaries.append(summary); all_rows.extend(values)
    write_csv(SELF / "ablation_new_results.csv", all_rows)
    (SELF / "ablation_new_summary.json").write_text(json.dumps({"summaries": summaries, "selection_authority": False, "holdout_executed": False}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")

    original = runner.SATCOFMPC; runner.SATCOFMPC = TracingSATC
    try:
        by_id = {row["sample_id"]: row for row in samples}; frozen = read(SELF / "self_freeze.json")
        trace_dir = SELF / "traces"; trace_dir.mkdir(parents=True, exist_ok=True)
        for stratum, sample_id in protocol["representative_traces"].items():
            TRACE_ROWS.clear(); runner.core.run_case("satc_ofmpc", frozen["parameters"], by_id[sample_id])
            write_csv(trace_dir / f"{stratum}.csv", list(TRACE_ROWS))
    finally:
        runner.SATCOFMPC = original
    print(json.dumps({"ablation_runs": len(all_rows), "trace_files": 4, "selection_authority": False}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
