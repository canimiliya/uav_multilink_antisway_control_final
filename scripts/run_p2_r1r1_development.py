"""Run preregistered P2-R1R1 stages through AuthoritativeNativeCaseRunner."""

from __future__ import annotations

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

from uav_sway.native_stack.r1r1_controllers import candidate_registries
from uav_sway.native_stack.r1r1_evaluation import aggregate, evaluate_case

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r1"
FOLDERS = {"native_pid":"traditional/pid", "native_full_lqr":"traditional/full_lqr", "native_task_lqr":"traditional/task_lqr", "satc_native":"satc"}
RATES = {"native_pid":(100,500), "native_full_lqr":(100,500), "native_task_lqr":(100,500), "satc_native":(100,500)}
INCUMBENTS = {
    "native_pid":{"family":"incumbent","method_id":"corrected_pid+LegacyTaskLevelAdapter","historical_id":"corrected_pid","kp":.12,"kd":.55,"tip_kp":.05,"tip_kd":.02},
    "native_full_lqr":{"family":"incumbent","method_id":"full_lqr_048+LegacyTaskLevelAdapter","historical_id":"full_lqr_048","kp":.15,"kd":.65,"tip_kp":.06,"tip_kd":.025},
    "native_task_lqr":{"family":"incumbent","method_id":"task_lqr_009+LegacyTaskLevelAdapter","historical_id":"task_lqr_009","kp":.18,"kd":.75,"tip_kp":.07,"tip_kd":.03},
    "satc_native":{"family":"incumbent","method_id":"satc_b_027+LegacyTaskLevelAdapter","historical_id":"satc_b_027","kp":.20,"kd":.80,"tip_kp":.08,"tip_kd":.035},
}


def serial(spec):
    value = {**spec, "family": spec.get("family")}
    if "gains" in value: value["gains"] = asdict(value["gains"])
    return value


def execute(spec, identities, rates, workers):
    payloads = [(spec, identity, rates[0], rates[1]) for identity in identities]
    if workers == 1: return [evaluate_case(x) for x in payloads]
    with ProcessPoolExecutor(max_workers=workers) as pool: return list(pool.map(evaluate_case, payloads, chunksize=1))


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def rank(summary):
    return (-summary["safety_rate"], -summary["success_rate"], summary["setpoint_position_rmse_m"]+summary["trajectory_position_rmse_m"], summary["method_id"])


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--family", choices=FOLDERS, required=True); parser.add_argument("--stage", choices=["a","b","incumbent"], required=True); parser.add_argument("--workers", type=int, default=8); args = parser.parse_args()
    manifest = json.loads((ROOT/"reproducibility/native_stack/r0s/resolved_development_manifest.json").read_text(encoding="utf-8"))
    by_id = {x["sample_id"]:x["identity"] for x in manifest["cases"]}
    registry = candidate_registries()[args.family]
    specs = [{**x,"family":args.family} for x in registry]
    folder = BASE/FOLDERS[args.family]; folder.mkdir(parents=True,exist_ok=True)
    (folder/"candidate_registry.json").write_text(json.dumps([serial(x) for x in specs],indent=2,sort_keys=True)+"\n",encoding="utf-8")
    if args.stage == "a":
        ids = json.loads((BASE/"protocol/stage_a_manifest.json").read_text(encoding="utf-8"))["case_ids"]
        summaries=[]; all_rows=[]
        for spec in specs:
            print(f"STAGE_A {spec['method_id']}",flush=True); rows=execute(spec,[by_id[x] for x in ids],RATES[args.family],args.workers); all_rows.extend(rows); summaries.append(aggregate(rows))
        summaries.sort(key=rank); write_csv(folder/"stage_a_results.csv",all_rows); write_csv(folder/"stage_a_summary.csv",summaries)
        selected=[x["method_id"] for x in summaries[:2]]
        (folder/"stage_a_selection.json").write_text(json.dumps({"family":args.family,"selected_for_stage_b":selected,"selection_rule":"frozen deterministic rank","case_count":32},indent=2,sort_keys=True)+"\n",encoding="utf-8")
    elif args.stage == "b":
        selection=json.loads((folder/"stage_a_selection.json").read_text(encoding="utf-8"))["selected_for_stage_b"]
        finalists=[x for x in specs if x["method_id"] in selection]
        incumbent={**INCUMBENTS[args.family]}
        all_specs=[incumbent]+finalists; all_rows=[]; summaries=[]
        identities=[x["identity"] for x in manifest["cases"]]
        for spec in all_specs:
            rates=(20,200) if spec["family"]=="incumbent" else RATES[args.family]
            print(f"STAGE_B {spec['method_id']} cases=200",flush=True); rows=execute(spec,identities,rates,args.workers); all_rows.extend(rows); summaries.append(aggregate(rows))
        summaries.sort(key=rank); write_csv(folder/"development_results.csv",all_rows); write_csv(folder/"development_summary.csv",summaries)
        selected=summaries[0]; competence=(selected["safety_rate"]>=.98 and selected["catastrophic_count"]<=4 and selected["success_rate"]>=.70 and selected["setpoint_position_rmse_m"]<=1.25 and selected["trajectory_position_rmse_m"]<=1.50 and selected["deadline_miss_rate"]<=.01)
        selected_spec=next(x for x in all_specs if x["method_id"]==selected["method_id"])
        selected_rate=(20,200) if selected_spec["family"]=="incumbent" else RATES[args.family]
        freeze={"family":args.family,"incumbent":incumbent["method_id"],"challengers":selection,"selected":selected["method_id"],"selected_spec":serial(selected_spec),"metrics":selected,"competence_pass":competence,"native_rate_hz":selected_rate,"unique_configs":len(specs),"complete_development_cases":200}
        if "q" in selected_spec:
            controller=__import__("uav_sway.native_stack.r1r1_evaluation",fromlist=["build_controller"]).build_controller(selected_spec); freeze["linear_model_audit"]=controller.diagnostics()["linear_model_audit"]
        (folder/"freeze.json").write_text(json.dumps(freeze,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        print(json.dumps({"selected":selected["method_id"],"competence":competence,"safety":selected["safety_rate"],"success":selected["success_rate"]}),flush=True)
    else:
        incumbent={**INCUMBENTS[args.family]}; identities=[x["identity"] for x in manifest["cases"]]
        exact_path=folder/"incumbent_results_exact.csv"
        if exact_path.exists():
            print(f"REUSE_EXACT_INCUMBENT {incumbent['method_id']} cases=200",flush=True)
            with exact_path.open(encoding="utf-8",newline="") as h: exact=list(csv.DictReader(h))
        else:
            print(f"EXACT_INCUMBENT {incumbent['method_id']} cases=200",flush=True); exact=execute(incumbent,identities,(20,200),args.workers); write_csv(exact_path,exact)
        existing=[]
        with (folder/"development_results.csv").open(encoding="utf-8",newline="") as h: existing=list(csv.DictReader(h))
        provisional=[r for r in existing if r["method_id"]==incumbent["method_id"]]
        provisional_path=folder/"incumbent_results_provisional_no_authority.csv"
        if provisional and not provisional_path.exists(): write_csv(provisional_path,provisional)
        existing=[r for r in existing if r["method_id"]!=incumbent["method_id"]]
        # Restore numeric types for aggregation after CSV round trip.
        boolean={"safe","success","catastrophic"}; numeric={"position_rmse_m","position_p90_m","position_p95_m","velocity_rmse_m_s","orientation_rmse_rad","endpoint_position_error_m","endpoint_velocity_error_m_s","physical_effort","thrust_saturation_rate","torque_saturation_rate","wrench_rate_rms","runtime_mean_ms","runtime_p95_ms","runtime_p99_ms","runtime_max_ms","deadline_miss_rate"}
        for row in exact+existing:
            for key in boolean:
                if isinstance(row[key],str): row[key]=row[key].lower()=="true"
            for key in numeric:
                if isinstance(row[key],str): row[key]=float(row[key])
            if isinstance(row.get("acquisition_time_s"),str): row["acquisition_time_s"]=float(row["acquisition_time_s"]) if row["acquisition_time_s"] not in {"","None"} else None
        combined=exact+existing; write_csv(folder/"development_results.csv",combined)
        summaries=[]
        for method in sorted({r["method_id"] for r in combined}): summaries.append(aggregate([r for r in combined if r["method_id"]==method]))
        summaries.sort(key=rank); write_csv(folder/"development_summary.csv",summaries); selected=summaries[0]
        competence=(selected["safety_rate"]>=.98 and selected["catastrophic_count"]<=4 and selected["success_rate"]>=.70 and selected["setpoint_position_rmse_m"]<=1.25 and selected["trajectory_position_rmse_m"]<=1.50 and selected["deadline_miss_rate"]<=.01)
        selection=json.loads((folder/"stage_a_selection.json").read_text(encoding="utf-8"))["selected_for_stage_b"]; specs_by_id={x["method_id"]:x for x in specs}; selected_spec=incumbent if selected["method_id"]==incumbent["method_id"] else specs_by_id[selected["method_id"]]
        selected_rate=(20,200) if selected_spec["family"]=="incumbent" else RATES[args.family]
        freeze={"family":args.family,"incumbent":incumbent["method_id"],"incumbent_exact_historical_parameters":True,"provisional_incumbent_runs_preserved":bool(provisional),"provisional_incumbent_selection_authority":False,"challengers":selection,"selected":selected["method_id"],"selected_spec":serial(selected_spec),"metrics":selected,"competence_pass":competence,"native_rate_hz":selected_rate,"unique_configs":len(specs),"complete_development_cases":200}
        if "q" in selected_spec:
            controller=__import__("uav_sway.native_stack.r1r1_evaluation",fromlist=["build_controller"]).build_controller(selected_spec); freeze["linear_model_audit"]=controller.diagnostics()["linear_model_audit"]
        (folder/"freeze.json").write_text(json.dumps(freeze,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        print(json.dumps({"selected":selected["method_id"],"competence":competence,"exact_incumbent":True}),flush=True)


if __name__ == "__main__": main()
