"""Reproduce and summarize the seven frozen V3-R1 direct-tip PID failures."""

from __future__ import annotations

import csv
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from run_v3_r1_baselines import read_json, run_case


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility/v3/r1"
OUT = ROOT / "reproducibility/v3/r1r1"
UNSAFE = (
    "calm_face_diagonal_05",
    "constant_1.5_axis_05",
    "constant_1.5_face_diagonal_05",
    "constant_1.5_face_diagonal_06",
    "constant_1.5_face_diagonal_09",
    "constant_1.5_face_diagonal_10",
    "constant_3_face_diagonal_05",
)


def _execute(job: tuple[dict, dict, str]) -> dict:
    parameters, sample, path = job
    return run_case("pid", parameters, sample, path)


def _trace_summary(path: Path) -> dict:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    numeric = lambda key: [float(row[key]) for row in rows]
    boolean = lambda key: [row[key].lower() == "true" for row in rows]
    unsafe_indices = [index for index, safe in enumerate(boolean("safe")) if not safe]
    return {
        "trace": "temporary:" + path.name,
        "first_unsafe_time_s": None if not unsafe_indices else float(rows[unsafe_indices[0]]["time"]),
        "max_position_error_m": max(numeric("position_error_3d_m")),
        "min_tip_height_m": min(numeric("tip_z")),
        "max_abs_roll_deg": max(abs(value) for value in numeric("roll_deg")),
        "max_abs_pitch_deg": max(abs(value) for value in numeric("pitch_deg")),
        "max_abs_raw_command_m_s2": max(abs(value) for key in ("raw_ax", "raw_ay", "raw_az") for value in numeric(key)),
        "saturated_log_fraction": sum(boolean("saturated")) / len(rows),
        "slew_limited_log_fraction": sum(boolean("slew_limited")) / len(rows),
        "max_abs_integral": max(abs(value) for key in ("integral_x", "integral_y", "integral_z") for value in numeric(key)),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    traces = ROOT / ".benchmarks/v3_r1r1_pid_failure_audit"
    traces.mkdir(parents=True, exist_ok=True)
    parameters = read_json(R1 / "pid_freeze.json")["parameters"]
    manifest = read_json(R1 / "development_evaluation_manifest.json")["samples"]
    by_id = {sample["sample_id"]: sample for sample in manifest}
    jobs = [(parameters, by_id[sample_id], str(traces / f"{sample_id}.csv")) for sample_id in UNSAFE]
    with ProcessPoolExecutor(max_workers=len(jobs)) as pool:
        results = list(pool.map(_execute, jobs))
    cases = []
    for sample_id, result in zip(UNSAFE, results, strict=True):
        cases.append({**result, **_trace_summary(traces / f"{sample_id}.csv")})
    audit = {
        "audit": "V3-R1 direct-tip PID negative-result failure audit",
        "source_candidate": parameters["candidate_id"],
        "development_only": True,
        "holdout_executed": False,
        "unsafe_cases_reproduced": len(cases),
        "ROOT_CAUSE_PRIMARY": "The direct-tip PID closes a high-gain acceleration loop around a flexible five-link tip without anchoring the UAV to the equilibrium-mapped reference. Downward and coupled diagonal commands excite large UAV/chain motion until attitude or tip-height safety fails.",
        "ROOT_CAUSE_SECONDARY": "The conditional anti-windup sign test is reversed: when integration pushes a constrained command farther into positive or negative saturation/slew, the old implementation accepts rather than freezes that integral update.",
        "REFERENCE_MAPPING": "The frozen p_uav_ref = p_tip_target - r_tip_equilibrium mapping is dimensionally and sign correct; it was computed but not used by V3TaskPID's direct-tip feedback law.",
        "PROPOSED_CORRECTION": "Preserve V3TaskPID as negative evidence. Add a classical cascaded controller with equilibrium UAV-reference anchoring, bounded causal tip-error reference correction, UAV PID/PD, derivative-on-measurement, and direction-correct saturation/slew-aware conditional integration.",
        "EVIDENCE": {
            "raw_trace_policy": "Seven temporary 5 ms traces were summarized into this JSON and excluded from Git to avoid redundant multi-megabyte evidence.",
            "unsafe_sample_ids": list(UNSAFE),
            "failure_reasons": sorted({reason for case in cases for reason in case["safety_failure_reasons"]}),
            "case_diagnostics": cases,
            "antiwindup_logic": "For positive error the integral command increment is negative. Under negative constraint raw-candidate is negative, so the old sign(error)==sign(raw-candidate) test is false and incorrectly permits integration; signs reverse symmetrically for negative error.",
        },
    }
    (OUT / "r1_pid_failure_audit.json").write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"unsafe_cases_reproduced": len(cases), "failure_reasons": audit["EVIDENCE"]["failure_reasons"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
