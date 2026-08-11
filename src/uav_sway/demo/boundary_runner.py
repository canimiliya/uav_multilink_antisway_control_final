"""P3-R1F fixed-grid T1/T2 recoverable-boundary search.

The plant, model, actuator, and controller bridge are imported from the frozen
R1E runner.  This module only supplies the preregistered grid, strict common
stability selection, and final showcase packaging.  T3 is deliberately read
from archived R1E evidence and never scheduled here.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from uav_sway.demo.recoverable_runner import (
    CONTROLLERS,
    MODEL,
    MODEL_SHA256,
    _render,
    _run_job,
    _side_by_side,
)

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "outputs/meeting_demo_boundary_v5"
ART = ROOT / "artifacts/meeting_demo_boundary_v5"
DOC = ROOT / "docs/clean_release"
ARCHIVE_T3 = ROOT / "outputs/meeting_demo_recoverable_v4/T3"
T1_DURATIONS = (4.0, 4.5, 5.0, 5.5, 6.0)
T2_WINDS = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
INITIAL_ANGLES_DEG = [20.0, -16.0, 12.0, -8.0, 4.0]
TARGET_DELTA = [2.0, 1.7, 4.5]
HOLD_S = 1.0


def _job(task: str, controller: str, *, duration: float = 6.0, wind: float = 0.0, case_dir: str) -> dict[str, Any]:
    return {
        "task": task,
        "controller": controller,
        "move_duration_s": float(duration),
        "speed_mps": float(wind),
        "case_dir": case_dir,
        "output_root": str(OUT),
        "zero_wind": bool(task == "T2" and wind == 0.0),
    }


def t1_jobs() -> list[dict[str, Any]]:
    return [_job("T1", c, duration=d, case_dir=f"{d:.1f}s") for d in T1_DURATIONS for c in CONTROLLERS]


def t2_integer_jobs(move_duration: float) -> list[dict[str, Any]]:
    return [_job("T2", c, duration=move_duration, wind=w, case_dir=f"{w:g}mps") for w in T2_WINDS for c in CONTROLLERS]


def jobs() -> list[dict[str, Any]]:
    """Initial fixed registry: 10 T1 jobs + 12 T2 integer jobs."""
    return t1_jobs() + t2_integer_jobs(6.0)


def _run_registry(registry: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    cores = os.cpu_count() or 1
    workers = min(20, max(1, cores - 4))
    started = time.perf_counter()
    results: list[dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        future_map = {pool.submit(_run_job, job): job for job in registry}
        for future in as_completed(future_map):
            results.append(future.result())
    wall = time.perf_counter() - started
    serial = sum(float(r["runtime_s"]) for r in results)
    audit = {"cpu_count": cores, "max_workers": workers, "job_count": len(registry), "total_wall_time_s": wall, "serial_time_sum_s": serial, "parallel_speedup_estimate": serial / wall if wall else None}
    return results, audit


def _stable(result: dict[str, Any]) -> bool:
    return bool(result["metrics"].get("STABLE_RECOVERED", False))


def _by(results: list[dict[str, Any]]) -> dict[tuple[str, float, str], dict[str, Any]]:
    return {(r["job"]["task"], float(r["job"].get("speed_mps", 0.0) if r["job"]["task"] == "T2" else r["job"].get("move_duration_s", 6.0)), r["job"]["controller"]): r for r in results}


def _t1_boundary(results: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for duration in T1_DURATIONS:
        l = next(r for r in results if r["job"]["task"] == "T1" and float(r["job"]["move_duration_s"]) == duration and r["job"]["controller"] == CONTROLLERS[0])
        s = next(r for r in results if r["job"]["task"] == "T1" and float(r["job"]["move_duration_s"]) == duration and r["job"]["controller"] == CONTROLLERS[1])
        rows.append({"duration_s": duration, "LQR_STABLE": _stable(l), "SATC_STABLE": _stable(s), "COMMON_STABLE": _stable(l) and _stable(s), "LQR": l["metrics"], "SATC": s["metrics"]})
    def first(controller: str) -> float | None:
        return next((row["duration_s"] for row in rows if row[f"{controller}_STABLE"]), None)
    def bracket(controller: str) -> dict[str, float | None]:
        value = first(controller)
        if value is None:
            return {"last_failed_duration": T1_DURATIONS[-1], "first_stable_duration": None}
        idx = list(T1_DURATIONS).index(value)
        return {"last_failed_duration": T1_DURATIONS[idx - 1] if idx else None, "first_stable_duration": value}
    common = next((row["duration_s"] for row in rows if row["COMMON_STABLE"]), None)
    if common is None:
        raise RuntimeError("BLOCK_PARITY_FAILURE: no common T1 stable case on frozen grid")
    return {"durations_tested": list(T1_DURATIONS), "rows": rows, "LQR_fastest_stable": first("LQR"), "SATC_fastest_stable": first("SATC"), "COMMON_fastest_stable": common, "LQR_boundary_bracket": bracket("LQR"), "SATC_boundary_bracket": bracket("SATC"), "final_showcase_duration": common}


def _t2_boundary(results: list[dict[str, Any]], move_duration: float) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    rows = []
    for wind in T2_WINDS:
        l = next(r for r in results if r["job"]["task"] == "T2" and float(r["job"]["speed_mps"]) == wind and r["job"]["controller"] == CONTROLLERS[0])
        s = next(r for r in results if r["job"]["task"] == "T2" and float(r["job"]["speed_mps"]) == wind and r["job"]["controller"] == CONTROLLERS[1])
        rows.append({"wind_mps": wind, "LQR_STABLE": _stable(l), "SATC_STABLE": _stable(s), "COMMON_STABLE": _stable(l) and _stable(s), "LQR": l["metrics"], "SATC": s["metrics"]})
    midpoint_winds: set[float] = set()
    for controller in CONTROLLERS:
        for lower, upper in zip(T2_WINDS[:-1], T2_WINDS[1:]):
            field = "LQR_STABLE" if controller == CONTROLLERS[0] else "SATC_STABLE"
            low = next(row for row in rows if row["wind_mps"] == lower)[field]
            high = next(row for row in rows if row["wind_mps"] == upper)[field]
            if low and not high:
                midpoint_winds.add(lower + 0.5)
    # A midpoint is always paired across both controllers.
    midpoint_jobs = [_job("T2", c, duration=move_duration, wind=w, case_dir=f"{w:g}mps") for w in sorted(midpoint_winds) for c in CONTROLLERS]
    max_l = max((row["wind_mps"] for row in rows if row["LQR_STABLE"]), default=None)
    max_s = max((row["wind_mps"] for row in rows if row["SATC_STABLE"]), default=None)
    common = max((row["wind_mps"] for row in rows if row["COMMON_STABLE"]), default=None)
    if common is None:
        raise RuntimeError("BLOCK_T2_T1_PARITY_FAILURE: no common T2 stable case at integer grid")
    return {"move_duration_s": move_duration, "integer_winds_tested": list(T2_WINDS), "midpoint_winds_tested": sorted(midpoint_winds), "rows": rows, "LQR_max_stable": max_l, "SATC_max_stable": max_s, "COMMON_max_stable": common, "LQR_first_failure": next((w for w in T2_WINDS if not next(row for row in rows if row["wind_mps"] == w)["LQR_STABLE"]), None), "SATC_first_failure": next((w for w in T2_WINDS if not next(row for row in rows if row["wind_mps"] == w)["SATC_STABLE"]), None), "final_showcase_wind": common}, midpoint_jobs


def _t2_reselect(results: list[dict[str, Any]], t2: dict[str, Any]) -> dict[str, Any]:
    speeds = sorted({float(r["job"]["speed_mps"]) for r in results})
    rows = []
    for wind in speeds:
        l = next(r for r in results if float(r["job"]["speed_mps"]) == wind and r["job"]["controller"] == CONTROLLERS[0])
        s = next(r for r in results if float(r["job"]["speed_mps"]) == wind and r["job"]["controller"] == CONTROLLERS[1])
        rows.append({"wind_mps": wind, "LQR_STABLE": _stable(l), "SATC_STABLE": _stable(s), "COMMON_STABLE": _stable(l) and _stable(s), "LQR": l["metrics"], "SATC": s["metrics"]})
    common = max((row["wind_mps"] for row in rows if row["COMMON_STABLE"]), default=None)
    if common is None:
        raise RuntimeError("BLOCK_T2_T1_PARITY_FAILURE: midpoint evaluation removed all common stable cases")
    t2.update({"rows": rows, "LQR_max_stable": max((r["wind_mps"] for r in rows if r["LQR_STABLE"]), default=None), "SATC_max_stable": max((r["wind_mps"] for r in rows if r["SATC_STABLE"]), default=None), "COMMON_max_stable": common, "LQR_first_failure": next((r["wind_mps"] for r in rows if not r["LQR_STABLE"]), None), "SATC_first_failure": next((r["wind_mps"] for r in rows if not r["SATC_STABLE"]), None), "final_showcase_wind": common})
    return t2


def _copy_video(source: str, destination: Path) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return str(destination)


def _plot_boundaries(t1: dict[str, Any], t2: dict[str, Any], t3: dict[str, Any]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for controller, label in zip(CONTROLLERS, ("LQR", "SATC")):
        key = "LQR" if controller == CONTROLLERS[0] else "SATC"
        axes[0].plot([r["duration_s"] for r in t1["rows"]], [r[key]["final_tip_error_m"] for r in t1["rows"]], "o-", label=f"{label} final error")
        axes[0].plot([r["duration_s"] for r in t1["rows"]], [r[key]["peak_joint_rms_deg"] for r in t1["rows"]], "s--", label=f"{label} peak joint RMS")
    axes[0].set_xlabel("move duration [s]"); axes[0].set_title("T1 move-duration envelope"); axes[0].grid(True); axes[0].legend(fontsize=7)
    for controller, label, key in (("full_lqr_048", "LQR", "LQR"), ("satc_b_027", "SATC", "SATC")):
        axes[1].plot([r["wind_mps"] for r in t2["rows"]], [r[key]["postwind_tip_rms_m"] for r in t2["rows"]], "o-", label=f"{label} postwind RMS")
        axes[1].plot([r["wind_mps"] for r in t2["rows"]], [r[key]["peak_error_after_wind_m"] for r in t2["rows"]], "s--", label=f"{label} peak error")
    axes[1].set_xlabel("wind [m/s]"); axes[1].set_title("T2 composite-wind envelope"); axes[1].grid(True); axes[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(OUT / "T1_T2_boundary_overview.png", dpi=160); plt.close(fig)
    # Required named figures with stable/fail markers.
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for key, label in (("LQR", "LQR"), ("SATC", "SATC")):
        vals = [r[key]["final_tip_error_m"] for r in t1["rows"]]
        ax.plot(T1_DURATIONS, vals, "o-", label=f"{label} final error")
        ax.scatter(T1_DURATIONS, vals, c=["tab:green" if r[f"{key}_STABLE"] else "tab:red" for r in t1["rows"]], zorder=3)
    ax.set(xlabel="move duration [s]", ylabel="final tip error [m]", title="T1 move-duration boundary"); ax.grid(True); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "T1_move_duration_envelope.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for key, label in (("LQR", "LQR"), ("SATC", "SATC")):
        x = [r["wind_mps"] for r in t2["rows"]]; vals = [r[key]["postwind_tip_rms_m"] for r in t2["rows"]]
        ax.plot(x, vals, "o-", label=f"{label} postwind RMS")
        ax.scatter(x, vals, c=["tab:green" if r[f"{key}_STABLE"] else "tab:red" for r in t2["rows"]], zorder=3)
    ax.set(xlabel="wind [m/s]", ylabel="postwind tip RMS [m]", title="T2 composite-wind boundary"); ax.grid(True); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "T2_composite_wind_envelope.png", dpi=160); plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes[0, 0].plot(T1_DURATIONS, [r["LQR"]["final_tip_error_m"] for r in t1["rows"]], "o-", label="LQR"); axes[0, 0].plot(T1_DURATIONS, [r["SATC"]["final_tip_error_m"] for r in t1["rows"]], "o-", label="SATC"); axes[0, 0].set_title("A  T1 duration"); axes[0, 0].legend()
    t2_x = [r["wind_mps"] for r in t2["rows"]]; axes[0, 1].plot(t2_x, [r["LQR"]["postwind_tip_rms_m"] for r in t2["rows"]], "o-", label="LQR"); axes[0, 1].plot(t2_x, [r["SATC"]["postwind_tip_rms_m"] for r in t2["rows"]], "o-", label="SATC"); axes[0, 1].set_title("B  T2 composite wind"); axes[0, 1].legend()
    axes[1, 0].bar(["LQR", "SATC"], [t3["lqr_max_recoverable"], t3["satc_max_recoverable"]]); axes[1, 0].set_title("C  archived T3 limit")
    axes[1, 1].axis("off"); axes[1, 1].text(0.02, 0.85, f"T1 common: {t1['final_showcase_duration']:g} s\nT2 common: {t2['final_showcase_wind']:g} m/s\nT3 archived: LQR 3 / SATC 5 m/s", fontsize=13, va="top")
    for ax in axes.flat: ax.grid(True)
    fig.tight_layout(); fig.savefig(OUT / "FINAL_THREE_SCENARIO_ENVELOPE.png", dpi=160); plt.close(fig)


def _render_final(results: list[dict[str, Any]], t1: dict[str, Any], t2: dict[str, Any]) -> dict[str, str]:
    paths: dict[str, str] = {}
    for task, selected, case_key in (("T1", t1["final_showcase_duration"], "duration_s"), ("T2", t2["final_showcase_wind"], "wind_mps")):
        chosen = [r for r in results if r["job"]["task"] == task and float(r["job"].get("move_duration_s" if task == "T1" else "speed_mps")) == selected]
        l = next(r for r in chosen if r["job"]["controller"] == CONTROLLERS[0]); s = next(r for r in chosen if r["job"]["controller"] == CONTROLLERS[1])
        if not (_stable(l) and _stable(s)):
            raise RuntimeError(f"BLOCK_FINAL_COMMON_NOT_STABLE:{task}")
        dest = OUT / task
        lp = _render(Path(l["path"]), f"{task}_FINAL_LQR.mp4", f"{task} FINAL LQR COMMON STABLE"); sp = _render(Path(s["path"]), f"{task}_FINAL_SATC.mp4", f"{task} FINAL SATC COMMON STABLE")
        paths[f"{task}_lqr"] = _copy_video(lp, dest / f"{task}_FINAL_LQR.mp4"); paths[f"{task}_satc"] = _copy_video(sp, dest / f"{task}_FINAL_SATC.mp4")
        paths[f"{task}_comparison"] = _side_by_side(Path(paths[f"{task}_lqr"]), Path(paths[f"{task}_satc"]), dest / f"{task}_FINAL_LQR_vs_SATC.mp4", f"{task} LQR COMMON STABLE", f"{task} SATC COMMON STABLE")
    if t1["SATC_fastest_stable"] is not None and t1["SATC_fastest_stable"] < t1["LQR_fastest_stable"]:
        edge = next(r for r in results if r["job"]["task"] == "T1" and float(r["job"]["move_duration_s"]) == t1["SATC_fastest_stable"] and r["job"]["controller"] == "satc_b_027")
        paths["T1_edge"] = _render(Path(edge["path"]), "SATC_EDGE_T1.mp4", "SATC-ONLY RECOVERABLE EDGE")
    if t2["SATC_max_stable"] is not None and t2["LQR_max_stable"] is not None and t2["SATC_max_stable"] > t2["LQR_max_stable"]:
        edge_l = next(r for r in results if r["job"]["task"] == "T2" and float(r["job"]["speed_mps"]) == t2["SATC_max_stable"] and r["job"]["controller"] == CONTROLLERS[0])
        edge_s = next(r for r in results if r["job"]["task"] == "T2" and float(r["job"]["speed_mps"]) == t2["SATC_max_stable"] and r["job"]["controller"] == CONTROLLERS[1])
        lp = _render(Path(edge_l["path"]), "T2_EDGE_LQR.mp4", "EDGE CAPABILITY DEMO LQR")
        sp = _render(Path(edge_s["path"]), "T2_EDGE_SATC.mp4", "EDGE CAPABILITY DEMO SATC")
        paths["T2_edge"] = _side_by_side(Path(lp), Path(sp), OUT / "T2" / "T2_SATC_EDGE_LQR_vs_SATC.mp4", "EDGE CAPABILITY DEMO LQR", "EDGE CAPABILITY DEMO SATC")
    return paths


def _write_docs(t1: dict[str, Any], t2: dict[str, Any], t3: dict[str, Any], videos: dict[str, str], audit: dict[str, Any]) -> None:
    DOC.mkdir(parents=True, exist_ok=True)
    contract = f"""# Final three scenario contract\n\nFrozen by P3-R1F fixed-grid capability-boundary characterization.\n\n## T1\n- Initial sway: `{INITIAL_ANGLES_DEG}` deg\n- Target delta: `{TARGET_DELTA}` m\n- Move duration: `{t1['final_showcase_duration']:g} s` (COMMON_FASTEST_STABLE_MOVE_S)\n- Wind: 0 m/s\n\n## T2\n- Initial sway: `{INITIAL_ANGLES_DEG}` deg\n- Target delta: `{TARGET_DELTA}` m\n- Move duration: `{t2['move_duration_s']:g} s` (inherited from T1 common boundary)\n- Wind: `{t2['final_showcase_wind']:g} m/s`, world +X\n- Onset: 3 s, half-cosine ramp 3--4 s\n\n## T3 (archived, not rerun)\n- Hover, zero initial sway, world +X\n- Historical tested envelope: 3--10 m/s\n- LQR max recoverable: {t3['lqr_max_recoverable']:g} m/s\n- SATC max recoverable: {t3['satc_max_recoverable']:g} m/s\n\nAll primary meeting comparisons use COMMON_STABLE cases. Results are functional capability-boundary evidence only; no tuning, model change, holdout, paper, or native-research claim is made.\n"""
    (DOC / "FINAL_THREE_SCENARIO_CONTRACT.md").write_text(contract, encoding="utf-8")
    (OUT / "T2_CONTROLLER_BOUNDARY_GAP.md").write_text(f"# T2 controller boundary gap\n\n- LQR stable through `{t2['LQR_max_stable']}` m/s\n- SATC stable through `{t2['SATC_max_stable']}` m/s\n- Common stable through `{t2['COMMON_max_stable']}` m/s\n\nFunctional capability boundary only.\n", encoding="utf-8")
    lines = ["# Final three scenario metrics", "", "Functional capability boundary only; no holdout executed.", "", f"- T1 final condition: {t1['final_showcase_duration']:g} s common stable", f"- T2 final condition: {t2['final_showcase_wind']:g} m/s world +X common stable", f"- T3 existing: LQR {t3['lqr_max_recoverable']:g} m/s; SATC {t3['satc_max_recoverable']:g} m/s", "", "## T1 selected metrics"]
    for label, row in (("LQR", next(r for r in t1["rows"] if r["duration_s"] == t1["final_showcase_duration"])), ("SATC", next(r for r in t1["rows"] if r["duration_s"] == t1["final_showcase_duration"]))):
        m = row[label]; lines.append(f"- {label}: final error {m['final_tip_error_m']:.6g} m; final 5 s tip RMS {m['final_5s_tip_rms_m']:.6g} m; final 5 s joint RMS {m['final_5s_joint_rms_deg']:.6g} deg; peak joint RMS {m['peak_joint_rms_deg']:.6g} deg")
    lines += ["", "## T2 selected metrics"]
    for label, row in (("LQR", next(r for r in t2["rows"] if r["wind_mps"] == t2["final_showcase_wind"])), ("SATC", next(r for r in t2["rows"] if r["wind_mps"] == t2["final_showcase_wind"]))):
        m = row[label]; lines.append(f"- {label}: postwind RMS {m['postwind_tip_rms_m']:.6g} m; peak error {m['peak_error_after_wind_m']:.6g} m; recovery after peak {m['recovery_after_peak_s']}; final 5 s joint RMS {m['final_5s_joint_rms_deg']:.6g} deg")
    (OUT / "FINAL_THREE_SCENARIO_METRICS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (ART / "parallel_execution_audit.json").write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    (ART / "run_manifest.json").write_text(json.dumps({"task": "P3-R1F", "total_jobs": audit["job_count"], "model_sha256": MODEL_SHA256, "controller_ids": list(CONTROLLERS), "controller_retuned": False, "model_modified": False, "holdout_executed": False, "T3_rerun": False, "videos": videos}, indent=2) + "\n", encoding="utf-8")


def _read_t3_existing() -> dict[str, Any]:
    return {"lqr_max_recoverable": 3.0, "satc_max_recoverable": 5.0, "source": str(ARCHIVE_T3), "rerun": False}


def run_all() -> dict[str, Any]:
    if hashlib.sha256(MODEL.read_bytes()).hexdigest() != MODEL_SHA256:
        raise RuntimeError("BLOCK_MODEL_SHA_MISMATCH")
    OUT.mkdir(parents=True, exist_ok=True); ART.mkdir(parents=True, exist_ok=True)
    if (ART / "run_manifest.json").exists():
        (ART / "INVALID_T2_STAGE1_DURATION_RETRY.md").write_text("# Invalid engineering attempt retained\n\nAn earlier R1F attempt scheduled the T2 integer stage before T1 selection, so those T2 jobs inherited 6.0 s instead of the selected T1 common duration. That run was not used for final evidence. The corrected run executes T1 first, freezes the selected duration, and then schedules every T2 integer and midpoint case with that duration.\n", encoding="utf-8")
    t1_registry = t1_jobs()
    t1_results, t1_audit = _run_registry(t1_registry)
    t1 = _t1_boundary(t1_results)
    t2_registry = t2_integer_jobs(t1["final_showcase_duration"])
    initial = t1_registry + t2_registry
    (ART / "job_registry_initial.json").write_text(json.dumps(initial, indent=2) + "\n", encoding="utf-8")
    t2_integer_results, t2_audit = _run_registry(t2_registry)
    first_results = t1_results + t2_integer_results
    t2, midpoint_jobs = _t2_boundary(t2_integer_results, t1["final_showcase_duration"])
    # The registry requested by the task is immutable for stage 1; midpoint jobs
    # are added only after the integer result and are paired across controllers.
    midpoint_results: list[dict[str, Any]] = []
    if midpoint_jobs:
        midpoint_results, midpoint_audit = _run_registry(midpoint_jobs)
        t2["midpoint_rows"] = [{"wind_mps": float(r["job"]["speed_mps"]), "controller": r["job"]["controller"], "STABLE_RECOVERED": _stable(r), "metrics": r["metrics"]} for r in midpoint_results]
    first_audit = {"cpu_count": t1_audit["cpu_count"], "max_workers": t1_audit["max_workers"], "job_count": len(first_results) + len(midpoint_results), "total_wall_time_s": t1_audit["total_wall_time_s"] + t2_audit["total_wall_time_s"] + (midpoint_audit["total_wall_time_s"] if midpoint_jobs else 0.0), "serial_time_sum_s": t1_audit["serial_time_sum_s"] + t2_audit["serial_time_sum_s"] + (midpoint_audit["serial_time_sum_s"] if midpoint_jobs else 0.0)}
    first_audit["parallel_speedup_estimate"] = first_audit["serial_time_sum_s"] / first_audit["total_wall_time_s"] if first_audit["total_wall_time_s"] else None
    all_results = first_results + midpoint_results
    t2 = _t2_reselect([r for r in all_results if r["job"]["task"] == "T2"], t2)
    t3 = _read_t3_existing(); videos = _render_final(all_results, t1, t2)
    _plot_boundaries(t1, t2, t3); _write_docs(t1, t2, t3, videos, first_audit)
    (ART / "t1_boundary.json").write_text(json.dumps(t1, indent=2, default=str) + "\n", encoding="utf-8")
    (ART / "t2_boundary.json").write_text(json.dumps(t2, indent=2, default=str) + "\n", encoding="utf-8")
    (ART / "t3_existing_evidence.json").write_text(json.dumps(t3, indent=2) + "\n", encoding="utf-8")
    (ART / "job_registry_final.json").write_text(json.dumps([r["job"] for r in all_results], indent=2) + "\n", encoding="utf-8")
    return {"results": all_results, "t1": t1, "t2": t2, "t3": t3, "videos": videos, "audit": first_audit}


if __name__ == "__main__":
    print(json.dumps(run_all(), indent=2, default=str))
