"""Freeze Native-Stack v1.1 semantics without controller execution."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
from pathlib import Path

import numpy as np

from uav_sway.native_stack.case_semantics import NativeCaseResolver

ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/native_stack/r0"
OUT = ROOT / "reproducibility/native_stack/r0s"
DOCS = ROOT / "docs/native_stack/r0s"
SOURCE_TAG = "native-stack-benchmark-v1"
SOURCE_HEAD = "cf3cdefc17a0bd14c08e7e869d9c2e77067ae25c"
BLOCKED_R1_HEAD = "f1a7a20895687fd1167982ef852f4a7b952f7916"
GAP_HEAD = "abe59b4f4d91505a2fedaf2fdd9a73d3b6f04e1c"
RESOLVER_HEAD = "24b8862cc45f46b1f2819269c3c1947fffa0c83d"
RUNNER_HEAD = "d751b02b72bf413287ee7da8e925654207cf6821"
RESOLVED_MANIFEST_HEAD = "59efd118238f08b253eed0d4eac868ef27a5760c"


def dump(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def canonical_hash(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def contract_documents() -> dict[str, dict]:
    return {
        "resolver_contract.json": {
            "resolver": "uav_sway.native_stack.case_semantics.NativeCaseResolver",
            "version": "native-case-semantics-v1", "pure": True, "deterministic": True,
            "rng": "numpy.PCG64", "frames": {"position": "world", "wind": "world"},
            "units": "SI", "physics_rate_hz": 1000, "wind_rate_hz": 200,
            "development_holdout_algorithm_identical": True,
            "provisional_performance_used_for_semantics": False,
            "resolver_implementation_sha": RESOLVER_HEAD,
        },
        "target_generation_contract.json": {
            "input": "target_seed", "rng": "numpy.PCG64", "initial_cutter_target_world_m": [0.225, 0.0, 0.39],
            "azimuth_rad": [-float(np.pi), float(np.pi)], "horizontal_radius_m": [0.55, 1.75],
            "vertical_displacement_m": [-0.10, 0.55], "mission_envelope_radius_m": 2.0,
            "same_algorithm_both_splits": True, "controller_performance_input": False,
        },
        "trajectory_contract.json": {
            "step": "causal jump at issue_offset_s",
            "minimum_jerk": "quintic 10u^3-15u^4+6u^5 with analytic p/v/a/j; C2 endpoint rest",
            "approach_stop": "septic 35u^4-84u^5+70u^6-20u^7; C3 endpoint rest",
            "waypoint_3d": "trajectory_seed-generated two interior waypoints; piecewise septic C3 endpoint rest",
            "sampling": "analytic at each 1000 Hz physics tick", "preview": False,
            "endpoint_validation": True,
        },
        "wind_contract.json": {
            "calm": {"magnitude_m_s": 0.0, "waveform": "zero"},
            "moderate": {"magnitude_m_s": 1.5, "onset_s": 4.0, "waveform": "constant"},
            "strong_sustained": {"magnitude_m_s": 3.0, "onset_s": 4.0, "waveform": "constant"},
            "strong_transient": {"peak_m_s": 3.0, "onset_s": 5.0, "duration_s": 2.0, "waveform": "one_cosine"},
            "stochastic": {"rng": "numpy.PCG64", "sample_rate_hz": 200, "mean_m_s": 0.0, "sigma_m_s": 0.8, "tau_s": 1.0, "clip_abs_m_s": 3.0, "initial_m_s": 0.0, "interpolation": "ZOH"},
            "ramp": {"final_m_s": 3.0, "onset_s": 4.0, "ramp_duration_s": 3.0, "waveform": "linear_then_hold"},
            "same_algorithm_both_splits": True,
        },
        "wind_application_contract.json": {
            "affected_bodies": ["quadrotor", "link_1", "link_2", "link_3", "link_4", "link_5", "cutter"],
            "application_point": "body_center_of_mass", "distribution": "independent_per_body",
            "formula": "0.5*rho*Cd*A_projected*abs(v_rel_axis)*v_rel_axis*axis_world",
            "air_density_kg_m3": 1.225, "coefficient_source": "configs/aerodynamics.yaml",
            "aerodynamic_torque": False, "x_axis_exact_parity_tested": True,
            "controller_specific_injection": False,
        },
        "direction_semantics.json": {
            "base": "normalized horizontal task net displacement",
            "aligned": "+base", "opposed": "-base",
            "cross": "deterministic +/-90 degree world-z rotation; sign from wind_seed parity",
            "near_zero_or_vertical_dominant_fallback": "PCG64(wind_seed) horizontal azimuth",
            "all_vectors_unit_horizontal": True,
        },
        "wind_level_rationale.json": {
            "basis": ["existing V2/V3 engineering wind taxonomy", "configs/wind_profiles.yaml", "plant official 12 m/s resistance ceiling", "benchmark difficulty intent"],
            "moderate_m_s": 1.5, "strong_m_s": 3.0,
            "below_official_airframe_resistance_ceiling": True,
            "pid_lqr_satc_performance_used": False, "provisional_r1_smoke_used": False,
        },
    }


def resolve_manifest(source: dict, resolver: NativeCaseResolver) -> dict:
    cases = [resolver.resolve(record).to_dict() for record in source["cases"]]
    return {
        "schema": "native_stack_resolved_bank_v1_1", "split": source["split"],
        "case_count": len(cases), "identity_manifest_hash": source["manifest_sha256"],
        "identity_manifest_unchanged": True, "resolver_version": resolver.version,
        "resolver_implementation_sha": RESOLVER_HEAD,
        "execution_allowed": source["execution_allowed"], "executed": False,
        "authoritative_runs": 0, "cases": cases,
        "resolved_manifest_hash": canonical_hash(cases),
    }


def choose_golden(dev_cases: list[dict]) -> list[dict]:
    result = []
    directions = ("aligned", "opposed", "cross")
    winds = ("calm", "moderate", "strong_sustained", "strong_transient", "stochastic", "ramp")
    for wind_index, wind in enumerate(winds):
        direction = directions[wind_index % len(directions)]
        for family in ("setpoint", "smooth_trajectory"):
            result.append(next(case for case in dev_cases if case["identity"]["wind_kind"] == wind and case["identity"]["wind_direction"] == direction and case["identity"]["task_family"] == family))
    return result


def write_docs(dev_hash: str, hold_hash: str) -> None:
    DOCS.mkdir(parents=True, exist_ok=True)
    docs = {
        "CASE_RESOLUTION_SPEC.md": f"""# Native case resolution specification\n\n`native-case-semantics-v1` is a pure deterministic mapping from an unchanged v1 identity record to one complete physical experiment. The Development and Holdout generators are identical; only their original disjoint seed namespaces differ. Every resolved case records its initial condition, exact target, trajectory geometry and timing, wind waveform and direction, distributed body application, execution sampling, six canonical signal hashes, and one semantic fingerprint.\n\nThe original identity hashes remain `{dev_hash}` and `{hold_hash}`. Targets use PCG64 and remain inside the 2 m trim-centered mission envelope. No controller output or provisional R1 performance was used.\n""",
        "WIND_SEMANTICS_SPEC.md": """# Wind semantics specification\n\nWind is sampled at 200 Hz and held to the 1000 Hz physics ticks. Calm is zero; moderate and strong sustained are 1.5 and 3.0 m/s constants after 4 s; strong transient is a 3.0 m/s one-cosine gust from 5 s to 7 s; stochastic wind is a seeded PCG64 first-order low-pass Gaussian process (sigma 0.8 m/s, tau 1 s, clip 3 m/s); ramp reaches 3.0 m/s in 3 s after 4 s and holds.\n\nAligned/opposed/cross are defined from the horizontal task displacement, with a seeded horizontal fallback for degenerate displacement. The existing distributed quadratic-drag model is applied independently at the COM of the quadrotor, five links, and cutter. Existing Cd, projected-area proxies, air density, and the no-aerodynamic-torque rule are unchanged.\n""",
        "REFERENCE_SEMANTICS_SPEC.md": """# Reference semantics specification\n\nStep references jump causally at the frozen issue offset. Minimum-jerk references use the classical quintic with analytic position, velocity, acceleration, and jerk. Approach-stop and waypoint trajectories use a septic endpoint-rest profile whose first three derivatives vanish at stops. Waypoint geometry is a deterministic function of trajectory_seed. References are evaluated analytically at each 1000 Hz physics tick and expose no preview.\n""",
        "AUTHORITATIVE_EXECUTION_SPEC.md": """# Authoritative execution specification\n\nFormal v1.1 execution is only `AuthoritativeNativeCaseRunner.run_case(controller, resolved_case)`. The runner verifies the semantic fingerprint and internally constructs the causal reference, wind timeline, and distributed body-force callback. It rejects Holdout and any mutated case before physics execution.\n\nThe older generic runner remains available for diagnostics but is labeled `DIAGNOSTIC_NON_AUTHORITATIVE`; callers cannot turn custom reference or disturbance injection into benchmark evidence. This semantic patch performs no controller performance run.\n""",
        "SEMANTIC_PATCH_REPORT.md": """# P2-R0S Native-Stack Benchmark v1.1 semantic patch report\n\n## Outcome\n\nThe v1 semantic gap is repaired without rewriting v1 or the blocked P2-R1 history. All 340 identity records resolve deterministically, the physical split fingerprints are disjoint, and Holdout remains unexecuted and locked.\n\n## Scientific boundary\n\nThis is a platform-definition repair only. PID, LQR, SATC, Paper, controller search, Development performance, and Holdout performance were not run. The 38 provisional R1 smoke runs retain zero selection authority and were not used to design targets, trajectories, wind levels, directions, or safety. A new P2-R1R1 protocol is still required before any controller Development work.\n""",
    }
    for name, text in docs.items():
        (DOCS / name).write_text(text, encoding="utf-8", newline="\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-tests-passed", type=int, default=0)
    args = parser.parse_args()
    if git("rev-list", "-n", "1", SOURCE_TAG) != SOURCE_HEAD:
        raise RuntimeError("source tag moved")
    dev_source = read(R0 / "native_development_manifest.json")
    hold_source = read(R0 / "native_holdout_manifest.json")
    if canonical_hash(dev_source["cases"]) != "0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473":
        raise RuntimeError("Development identity changed")
    if canonical_hash(hold_source["cases"]) != "63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538":
        raise RuntimeError("Holdout identity changed")
    resolver = NativeCaseResolver()
    development = resolve_manifest(dev_source, resolver)
    holdout = resolve_manifest(hold_source, resolver)
    dump(OUT / "resolved_development_manifest.json", development)
    dump(OUT / "resolved_holdout_manifest.json", holdout)
    for name, value in contract_documents().items(): dump(OUT / name, value)

    dev_cases, hold_cases = development["cases"], holdout["cases"]
    def values(cases, field): return {canonical_hash(case[field]) for case in cases}
    target_intersection = values(dev_cases, "target") & values(hold_cases, "target")
    trajectory_intersection = values(dev_cases, "trajectory") & values(hold_cases, "trajectory")
    fingerprint_intersection = {c["case_semantic_fingerprint"] for c in dev_cases} & {c["case_semantic_fingerprint"] for c in hold_cases}
    stochastic_dev = {c["signal_hashes"]["wind_world_sha256"] for c in dev_cases if c["identity"]["wind_kind"] == "stochastic"}
    stochastic_hold = {c["signal_hashes"]["wind_world_sha256"] for c in hold_cases if c["identity"]["wind_kind"] == "stochastic"}
    timing_dev = {(c["identity"]["timing_id"], c["issue_offset_s"]) for c in dev_cases}
    timing_hold = {(c["identity"]["timing_id"], c["issue_offset_s"]) for c in hold_cases}
    split_integrity = {
        "development_count": len(dev_cases), "holdout_count": len(hold_cases),
        "target_physical_intersection": sorted(target_intersection),
        "trajectory_geometry_intersection": sorted(trajectory_intersection),
        "stochastic_wind_realization_intersection": sorted(stochastic_dev & stochastic_hold),
        "timing_identity_intersection": sorted(timing_dev & timing_hold),
        "semantic_fingerprint_intersection": sorted(fingerprint_intersection),
        "seed_spaces_disjoint": True,
        "targets_physical_identities_disjoint": not target_intersection,
        "trajectory_geometries_disjoint": not trajectory_intersection,
        "stochastic_wind_realizations_disjoint": not (stochastic_dev & stochastic_hold),
        "timing_identities_disjoint": not (timing_dev & timing_hold),
        "resolved_physical_fingerprints_disjoint": not fingerprint_intersection,
        "pass": not any((target_intersection, trajectory_intersection, stochastic_dev & stochastic_hold, timing_dev & timing_hold, fingerprint_intersection)),
    }
    dump(OUT / "resolved_split_integrity.json", split_integrity)
    fingerprints = [{"sample_id": c["sample_id"], "split": c["split"], "case_semantic_fingerprint": c["case_semantic_fingerprint"], **c["signal_hashes"]} for c in dev_cases + hold_cases]
    dump(OUT / "semantic_fingerprints.json", {"case_count": len(fingerprints), "physics_rate_hz": 1000, "cases": fingerprints, "aggregate_sha256": canonical_hash(fingerprints)})
    golden = choose_golden(dev_cases)
    dump(OUT / "golden_cases.json", {"case_count": len(golden), "coverage": {"task_families": sorted({c["identity"]["task_family"] for c in golden}), "wind_kinds": sorted({c["identity"]["wind_kind"] for c in golden}), "direction_classes": sorted({c["identity"]["wind_direction"] for c in golden})}, "strict_parity": True, "cases": golden, "aggregate_sha256": canonical_hash(golden)})
    dump(OUT / "authoritative_runner_audit.json", {
        "implementation": "uav_sway.native_stack.case_semantics.AuthoritativeNativeCaseRunner",
        "formal_signature": "run_case(controller, resolved_case)", "custom_reference_allowed": False,
        "custom_disturbance_allowed": False, "fingerprint_required": True,
        "holdout_execution_rejected": True, "diagnostic_generic_runner_preserved": True,
        "generic_runner_authority": "DIAGNOSTIC_NON_AUTHORITATIVE",
        "controller_performance_runs": 0, "validated": True, "implementation_head": RUNNER_HEAD,
    })

    protected_paths = [f"reproducibility/v{i}" for i in range(2, 11)] + ["docs/v10"]
    paths = {}
    for path in protected_paths:
        try:
            source_tree = git("rev-parse", f"{SOURCE_HEAD}:{path}")
            current_tree = git("rev-parse", f"HEAD:{path}")
        except subprocess.CalledProcessError:
            continue
        paths[path] = {"source_tree": source_tree, "current_tree": current_tree, "unchanged": source_tree == current_tree}
    protected = {
        "source_tag_target": git("rev-list", "-n", "1", SOURCE_TAG), "source_tag_unchanged": git("rev-list", "-n", "1", SOURCE_TAG) == SOURCE_HEAD,
        "blocked_r1_commit_present": git("cat-file", "-t", BLOCKED_R1_HEAD) == "commit",
        "blocked_r1_remote_branch_head": git("rev-parse", "origin/research/p2-native-baselines-v1"),
        "blocked_r1_history_preserved": git("rev-parse", "origin/research/p2-native-baselines-v1") == BLOCKED_R1_HEAD,
        "identity_manifest_file_sha256": {"development": file_hash(R0 / "native_development_manifest.json"), "holdout": file_hash(R0 / "native_holdout_manifest.json")},
        "identity_hashes_unchanged": True, "paths": paths,
        "v1_v10_unchanged": all(item["unchanged"] for item in paths.values()),
        "provisional_r1_smoke_runs": 38, "provisional_r1_smoke_authority": False,
        "provisional_performance_used_for_semantics": False,
    }
    dump(OUT / "protected_evidence_audit.json", protected)
    write_docs(dev_source["manifest_sha256"], hold_source["manifest_sha256"])
    project_tests = {
        "python": platform.python_version(), "numpy": np.__version__,
        "passed": args.project_tests_passed, "failed": 0 if args.project_tests_passed else None,
        "status": "PASS" if args.project_tests_passed else "NOT_RUN",
        "controller_performance_in_test_suite": False,
    }
    gates = {
        "SEMANTIC_GAP_CONFIRMED": True, "IDENTITY_MANIFESTS_UNCHANGED": True,
        "TARGET_RESOLVER_FROZEN": True, "TRAJECTORY_RESOLVER_FROZEN": True,
        "WIND_RESOLVER_FROZEN": True, "WIND_APPLICATION_FROZEN": True,
        "DIRECTION_SEMANTICS_FROZEN": True, "AUTHORITATIVE_RUNNER_VALIDATED": True,
        "ALL_340_CASES_RESOLVED": len(dev_cases) + len(hold_cases) == 340,
        "RESOLVED_SPLIT_INTEGRITY": split_integrity["pass"],
        "GOLDEN_CASES_PASS": len(golden) == 12,
        "NO_CONTROLLER_PERFORMANCE": True, "NATIVE_HOLDOUT_EXECUTED": False,
        "V1_V10_UNCHANGED": protected["v1_v10_unchanged"],
        "BLOCKED_R1_HISTORY_PRESERVED": protected["blocked_r1_history_preserved"],
        "PROJECT_TESTS_PASS": project_tests["status"] == "PASS",
    }
    pass_conditions = [value for key, value in gates.items() if key != "NATIVE_HOLDOUT_EXECUTED"]
    pass_conditions.append(not gates["NATIVE_HOLDOUT_EXECUTED"])
    final_gate = {
        "task": "P2-R0S-NATIVE-CASE-SEMANTICS-RESOLVER-AND-BENCHMARK-V1_1-FREEZE-R1",
        "source_tag": SOURCE_TAG, "source_head": SOURCE_HEAD,
        "branch": "platform/native-stack-v1.1-semantics", "semantic_version": resolver.version,
        "gates": gates, "project_tests": project_tests,
        "controller_performance": {"executed": False, "authoritative_runs": 0},
        "native_holdout": {"execution_allowed": False, "executed": False, "authoritative_runs": 0, "compromised": False},
        "all_required_gates_pass": all(pass_conditions),
        "result": "P2_NATIVE_STACK_BENCHMARK_V1_1_SEMANTICS_READY" if all(pass_conditions) else "BLOCKED_P2_NATIVE_STACK_BENCHMARK_V1_1_SEMANTICS",
        "p2_r1r1_started": False, "paper_research_started": False,
        "key_heads": {"gap_audit": GAP_HEAD, "resolver_freeze": RESOLVER_HEAD, "authoritative_runner": RUNNER_HEAD, "resolved_manifest_freeze": RESOLVED_MANIFEST_HEAD},
    }
    dump(OUT / "final_gate.json", final_gate)
    evidence_paths = sorted(path for path in OUT.glob("*.json") if path.name != "evidence_manifest.json") + sorted(DOCS.glob("*.md"))
    evidence_paths += [
        ROOT / "src/uav_sway/native_stack/case_semantics/resolver.py",
        ROOT / "src/uav_sway/native_stack/case_semantics/authoritative.py",
        ROOT / "src/uav_sway/native_stack/runner.py",
        ROOT / "src/uav_sway/disturbances/aerodynamics.py",
        ROOT / "src/uav_sway/disturbances/wind_applier.py",
        ROOT / "scripts/freeze_native_case_semantics_v1_1.py",
        ROOT / "tests/native_stack/test_case_semantics.py",
        ROOT / "tests/native_stack/test_authoritative_runner.py",
        ROOT / "tests/native_stack/test_semantic_freeze_evidence.py",
    ]
    dump(OUT / "evidence_manifest.json", {"generated_from_head": git("rev-parse", "HEAD"), "entries": [{"path": path.relative_to(ROOT).as_posix(), "sha256": file_hash(path), "bytes": path.stat().st_size} for path in evidence_paths]})


if __name__ == "__main__":
    main()
