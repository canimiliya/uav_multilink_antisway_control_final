"""Close V3-R3 from frozen Development evidence without new simulations."""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility" / "v3" / "r1"
R1R1 = ROOT / "reproducibility" / "v3" / "r1r1"
R2 = ROOT / "reproducibility" / "v3" / "r2"
R3 = ROOT / "reproducibility" / "v3" / "r3"
START_HEAD = "472fc54a82c3c50c328622ca10cb1eebeb2c61f9"
PROTOCOL_HEAD = "39a01592429897df943f306ec61c70deedabad3e"
EXPECTED_TREES = {
    "reproducibility/v2": "78db9b506b7649c68cbba2ba1c6d9c5d569962a7",
    "reproducibility/v3/r0": "36c63351b55e827e21423cd792ab5b96b294a479",
    "reproducibility/v3/r1": "39855236f5c2890f184b38dd4d07394921b2cc23",
    "reproducibility/v3/r1r1": "345472cc7ce95bcfe4660defc1c7c2194eb23fc4",
    "reproducibility/v3/r2": "5db2db085ac2c3167eb7ff2d15b49cd5bca37f61",
}


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def number(row: dict, key: str) -> float | None:
    value = row.get(key, "")
    return None if value in {None, "", "None"} else float(value)


def boolean(row: dict, key: str) -> bool:
    return str(row[key]).lower() == "true"


def aggregate_from_row(row: dict) -> dict:
    integer = ("sample_count", "safe_sample_count", "task_success_count")
    floating = (
        "safety_rate", "success_rate", "acquisition_median_s", "position_rmse_3d_m",
        "orientation_rmse_deg", "ramp_peak_position_error_m",
        "ramp_steady_state_position_error_m", "total_acceleration_effort", "solve_time_p95_ms",
    )
    return {
        "candidate_id": row["candidate_id"],
        **{key: int(row[key]) for key in integer},
        **{key: number(row, key) for key in floating},
    }


def pair(selected: list[dict], comparator: list[dict], comparator_name: str) -> dict:
    left = {row["sample_id"]: row for row in selected}
    right = {row["sample_id"]: row for row in comparator}
    if set(left) != set(right) or len(left) != 75:
        raise RuntimeError(f"{comparator_name} exact pairing failed")
    ids = sorted(left)
    differences = np.asarray([
        number(right[name], "position_rmse_3d_m") - number(left[name], "position_rmse_3d_m")
        for name in ids
    ])
    acquisition_pairs = [
        number(right[name], "acquisition_time_s") - number(left[name], "acquisition_time_s")
        for name in ids
        if number(right[name], "acquisition_time_s") is not None
        and number(left[name], "acquisition_time_s") is not None
    ]
    return {
        "pairing": "exact sample_id",
        "comparator": comparator_name,
        "pair_count": len(ids),
        "sample_ids_sha256": hashlib.sha256("\n".join(ids).encode()).hexdigest(),
        "position_difference_definition": "comparator_i - paper_i; positive favors Paper",
        "position_difference_mean_m": float(np.mean(differences)),
        "position_difference_median_m": float(np.median(differences)),
        "position_positive_fraction": float(np.mean(differences > 0.0)),
        "acquisition_common_pair_count": len(acquisition_pairs),
        "acquisition_difference_median_s": float(np.median(acquisition_pairs)) if acquisition_pairs else None,
        "formal_10000_bootstrap_executed": False,
        "bootstrap_reason": "reserved for future frozen Holdout; Paper candidate did not pass Development gate",
    }


def main() -> int:
    if subprocess.run(["git", "merge-base", "--is-ancestor", PROTOCOL_HEAD, "HEAD"], cwd=ROOT).returncode:
        raise RuntimeError("Paper protocol freeze is not an ancestor")
    actual_trees = {path: git("rev-parse", f"HEAD:{path}") for path in EXPECTED_TREES}
    if actual_trees != EXPECTED_TREES:
        raise RuntimeError(f"protected tree drift: {actual_trees}")

    contract = read_json(R1R1 / "advanced_numeric_win_contract.json")
    round_b = read_csv(R3 / "paper_round_b.csv")
    finalist_metrics = [aggregate_from_row(row) for row in round_b]
    near = finalist_metrics[0]
    if near["candidate_id"] != "paper_a_001":
        raise RuntimeError("unexpected preregistered Round-B ranking")
    near_parameters = read_json(R3 / "paper_round_b_summary.json")["best"]["parameters"]

    gates = {
        "safety": near["safety_rate"] >= contract["safety_threshold_rate"],
        "success": near["success_rate"] >= contract["success_threshold_rate"],
        "position": near["position_rmse_3d_m"] <= contract["position_rmse_max_m"],
        "acquisition": near["acquisition_median_s"] <= contract["acquisition_noninferiority_max_s"],
        "ramp": near["ramp_peak_position_error_m"] <= contract["ramp_peak_max_m"]
                or near["ramp_steady_state_position_error_m"] <= contract["ramp_steady_max_m"],
        "runtime": near["solve_time_p95_ms"] <= 50.0,
    }
    eligible = all(gates.values())
    if eligible:
        raise RuntimeError("finalizer is the no-win closure path but an eligible candidate exists")

    selected_rows = [row for row in read_csv(R3 / "development_results_round_b.csv") if row["candidate_id"] == near["candidate_id"]]
    traditional_rows = [
        row for row in read_csv(R1 / "traditional_development_results.csv")
        if row["method"] == "full_lqr" and row["candidate_id"] == "full_lqr_048"
    ]
    self_rows = [row for row in read_csv(R2 / "development_results_round_b.csv") if row["candidate_id"] == "self_a_034"]
    primary = read_json(R1R1 / "primary_traditional_baseline.json")["selected_metrics"]
    self_metrics = read_json(R2 / "self_freeze.json")["metrics"]
    paired_primary = pair(selected_rows, traditional_rows, "full_lqr_048")
    paired_self = pair(selected_rows, self_rows, "self_a_034")
    write_json(R3 / "paired_primary_comparison.json", paired_primary)
    write_json(R3 / "paired_self_comparison.json", paired_self)

    with (R3 / "development_results.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(selected_rows)

    improvements = {
        "position_fraction": (primary["position_rmse_3d_m"] - near["position_rmse_3d_m"]) / primary["position_rmse_3d_m"],
        "acquisition_fraction": (primary["acquisition_median_s"] - near["acquisition_median_s"]) / primary["acquisition_median_s"],
        "ramp_peak_fraction": (primary["ramp_peak_position_error_m"] - near["ramp_peak_position_error_m"]) / primary["ramp_peak_position_error_m"],
        "ramp_steady_fraction": (primary["ramp_steady_state_position_error_m"] - near["ramp_steady_state_position_error_m"]) / primary["ramp_steady_state_position_error_m"],
    }
    deficits = {
        "position_above_limit_m": near["position_rmse_3d_m"] - contract["position_rmse_max_m"],
        "position_relative_limit_miss_fraction": near["position_rmse_3d_m"] / contract["position_rmse_max_m"] - 1.0,
        "acquisition_above_limit_s": near["acquisition_median_s"] - contract["acquisition_noninferiority_max_s"],
        "acquisition_relative_limit_miss_fraction": near["acquisition_median_s"] / contract["acquisition_noninferiority_max_s"] - 1.0,
    }
    near_payload = {
        "candidate_id": near["candidate_id"],
        "parameters": near_parameters,
        "metrics": near,
        "gates": gates,
        "eligible": False,
        "win_level": "NO_WIN",
        "improvement_vs_primary": improvements,
        "gate_deficits": deficits,
        "reason": "Position and acquisition miss the immutable Development contract; no search expansion or paper switching is authorized."
    }
    write_json(R3 / "near_miss.json", near_payload)
    write_json(R3 / "paper_ablation.json", {
        "executed": False,
        "reason": "No Paper candidate passed the Development gate, so no candidate was frozen and the post-freeze ablation precondition was not met.",
        "paper_candidate_frozen": False,
        "holdout_executed": False,
    })
    write_json(R3 / "implementation_audit.json", {
        "method": "LV2026-SPACC-ADAPTED-3D",
        "protocol_freeze_head": PROTOCOL_HEAD,
        "protocol_freeze_is_ancestor": True,
        "frozen_v3_case_runner_modified": False,
        "paper_runner_reuses_frozen_case_execution_by_process_local_controller_factory": True,
        "self_module_imported_or_used": False,
        "traditional_gain_values_used_as_frozen_backbones_only": True,
        "three_axis_output": True,
        "common_limiter": True,
        "future_wind_or_target_used": False,
        "holdout_manifest_loaded": False,
    })
    write_json(R3 / "paper_execution_audit.json", {
        "smoke": {"candidates": 1, "cases_each": 1, "authoritative": False},
        "round_a": {"unique_candidates": 48, "cases_each": 14, "authoritative_cases": 672},
        "round_b": {"unchanged_finalists": 6, "cases_each": 75, "authoritative_cases": 450},
        "total_unique_candidates": 48,
        "refinement_rounds": 0,
        "workers": 8,
        "blas_threads_per_worker": 1,
        "gpu_used": False,
        "gpu_reason": "MuJoCo and the small NumPy controller path expose no CUDA execution path.",
        "non_authoritative_startup_events": [
            {
                "stage": "implementation unit-test launcher",
                "authoritative_case_count": 0,
                "outcome": "conda run returned Windows CLR HRESULT 0x80004005 before Python started",
                "recovery": "used the same uav_sway environment's direct Python executable; tests and all authoritative runs then completed"
            }
        ],
        "failed_or_interrupted_authoritative_runs": 0,
        "holdout_executed": False,
    })
    write_json(R3 / "safety_audit.json", {
        "round_a_all_safe": all(row["safety_rate"] == 1.0 for row in finalist_metrics) and all(float(row["safety_rate"]) == 1.0 for row in read_csv(R3 / "paper_round_a.csv")),
        "round_b_all_safe": all(row["safety_rate"] == 1.0 for row in finalist_metrics),
        "near_miss_safe_samples": near["safe_sample_count"],
        "near_miss_sample_count": near["sample_count"],
        "holdout_executed": False,
    })
    write_json(R3 / "holdout_access_audit.json", {
        "holdout_manifest_loaded": False,
        "holdout_execution_path_present_in_paper_runner": False,
        "forbidden_seeds_3000_3019_used": False,
        "forbidden_wind_2_0_or_3_5_m_s_used": False,
        "forbidden_ramp_3_5_m_s_used": False,
        "holdout_executed": False,
    })
    gate = {
        "task": "V3-R3-PAPER-ADVANCED-SELECTION-ADAPTATION-DEVELOPMENT-AND-FREEZE-R1",
        "start_head": START_HEAD,
        "paper_protocol_freeze_head": PROTOCOL_HEAD,
        "evidence_head_before_closure_commit": git("rev-parse", "HEAD"),
        "protected_trees": actual_trees,
        "traditional_unchanged": True,
        "self_unchanged": True,
        "advanced_contract_unchanged": True,
        "paper": "LV2026-SPACC-ADAPTED-3D",
        "paper_exact_reproduction": False,
        "paper_candidate_frozen": False,
        "near_miss_candidate": near["candidate_id"],
        "near_miss_gates": gates,
        "win_level": "NO_WIN",
        "paper_route_closed": True,
        "ablation_executed": False,
        "holdout_executed": False,
        "result": "CLOSED_WITH_NO_V3_PAPER_ADVANCED_WIN"
    }
    write_json(R3 / "gate.json", gate)

    report = f"""# V3 Paper-Advanced Report

## 结论

V3-R3 已按冻结协议完成，但没有 Paper candidate 同时通过全部 Development 门槛。路线以 `CLOSED_WITH_NO_V3_PAPER_ADVANCED_WIN` 关闭；没有换论文、追加搜索、修改 Traditional/Self，也没有访问 Holdout。

## 论文与适配

- 论文：*Modeling and Control for UAV with Off-center Slung Load*（Lv et al., 2026, arXiv preprint）
- 适配名：`LV2026-SPACC-ADAPTED-3D`
- 声明：adapted，非 exact reproduction
- Protocol freeze：`{PROTOCOL_HEAD}`
- 核心保留：以悬挂点三维加速度调节 alpha/beta 摆动；五连杆用 cutter-tip 到 UAV 的等效摆向量表示。
- 冻结接口：输出 `[ax, ay, az]`，统一经过 `±2 m/s²` 幅值和 `0.25 m/s²/update` slew，再进入同一 geometric inner loop。

## 搜索

- Round A：48 个唯一候选 × 14 Development core = 672 cases。
- Round B：Top-6 原参数 × 75 Development = 450 cases。
- Refinement：0；GPU：未使用；Holdout：未执行。

## 最佳 Near-Miss

`{near['candidate_id']}`：75/75 safe，{near['task_success_count']}/75 success（{near['success_rate']:.4f}），position RMSE `{near['position_rmse_3d_m']:.9f} m`，acquisition `{near['acquisition_median_s']:.3f} s`，ramp peak/steady `{near['ramp_peak_position_error_m']:.9f}/{near['ramp_steady_state_position_error_m']:.9f} m`，runtime P95 `{near['solve_time_p95_ms']:.4f} ms`。

它通过 safety、success、ramp、runtime，但 position 比硬门槛高 `{deficits['position_above_limit_m']:.9f} m`，acquisition 比硬门槛慢 `{deficits['acquisition_above_limit_s']:.5f} s`，因此 `WIN_LEVEL = NO_WIN`。

相对 Primary Full-LQR，position 改善 `{100*improvements['position_fraction']:.3f}%`，ramp peak 改善 `{100*improvements['ramp_peak_fraction']:.3f}%`，ramp steady 改善 `{100*improvements['ramp_steady_fraction']:.3f}%`；但 acquisition 恶化 `{abs(100*improvements['acquisition_fraction']):.3f}%`。相对冻结 Self，Paper 在 success、position、acquisition 和 ramp 上均更弱。

## 消融与 Holdout

没有 candidate 通过 Development gate，因此没有 Paper freeze；按“先冻结、后消融”的合同，消融未执行。Holdout manifest、3000-3019 seeds、2.0/3.5 m/s 风和 3.5 m/s ramp 均未访问。
"""
    (R3 / "V3_PAPER_ADVANCED_REPORT.md").write_text(report, encoding="utf-8", newline="\n")
    print(json.dumps({"result": gate["result"], "near_miss": near_payload}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
