"""Close P2-R1R1 at the frozen Traditional competence hard failure."""
from __future__ import annotations
import csv,hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]; BASE=ROOT/"reproducibility/native_stack/r1r1"; DOC=ROOT/"docs/native_stack/r1r1"
def read(p): return json.loads(p.read_text(encoding="utf-8"))
def write(p,v): p.parent.mkdir(parents=True,exist_ok=True); p.write_text(json.dumps(v,indent=2,sort_keys=True)+"\n",encoding="utf-8")
def writecsv(p,rows):
 p.parent.mkdir(parents=True,exist_ok=True)
 with p.open("w",encoding="utf-8",newline="") as h: w=csv.DictWriter(h,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
def unchanged(paths): return subprocess.run(["git","diff","--quiet","native-stack-benchmark-v1.1","--",*paths],cwd=ROOT).returncode==0
def main():
 paths={"pid":"traditional/pid","full_lqr":"traditional/full_lqr","task_lqr":"traditional/task_lqr"}; freezes={k:read(BASE/v/"freeze.json") for k,v in paths.items()}; rows=[]
 for k,f in freezes.items(): rows.append({"family":k,"method":f["selected"],"competence_pass":f["competence_pass"],"outer_rate_hz":f["native_rate_hz"][0],"inner_rate_hz":f["native_rate_hz"][1],**f["metrics"]})
 writecsv(BASE/"comparison/native_rate_table.csv",rows)
 def rank(f):
  m=f["metrics"]; return (-m["safety_rate"],-m["success_rate"],m["setpoint_position_rmse_m"],m["trajectory_position_rmse_m"],m["strong_p90_m"],m["method_id"])
 ordered=sorted(freezes.values(),key=rank); diagnostic=ordered[0]
 write(BASE/"traditional/primary_traditional.json",{"status":"NOT_FROZEN","reason":"zero of three Traditional families passed preregistered competence","diagnostic_best_only":diagnostic["selected"],"paper_comparator_authority":False})
 keys={"best_safety":("safety_rate",max),"best_success":("success_rate",max),"best_setpoint":("setpoint_position_rmse_m",min),"best_trajectory":("trajectory_position_rmse_m",min),"best_acquisition":("acquisition_time_s",min),"best_strong_mean":("strong_mean_m",min),"best_strong_p90":("strong_p90_m",min),"best_orientation":("orientation_rmse_rad",min),"best_effort":("physical_effort",min),"best_runtime":("runtime_p95_ms",min)}; env={"status":"DIAGNOSTIC_NOT_FROZEN","paper_comparator_authority":False,"entries":{}}
 for label,(key,fn) in keys.items():
  valid=[f for f in freezes.values() if f["metrics"].get(key) is not None]; x=fn(valid,key=lambda f:f["metrics"][key]); env["entries"][label]={"method":x["selected"],"value":x["metrics"][key]}
 write(BASE/"traditional/traditional_envelope.json",env)
 satc_stage_a=read(BASE/"satc/stage_a_selection.json"); satc_stage_b_path=BASE/"satc/freeze.json"
 if satc_stage_b_path.exists():
  satc_stage_b=read(satc_stage_b_path); satc_stage_b["status"]="DIAGNOSTIC_STAGE_B_NOT_STACK_FREEZE"; satc_stage_b["selection_authority"]=False; satc_stage_b["blocked_by_prior_gate"]="BLOCKED_P2_TRADITIONAL_COMPETENCE"; write(satc_stage_b_path,satc_stage_b)
 satc_status={"status":"NOT_FROZEN","reason":"Traditional competence hard gate failed before SATC qualification","stage_a_complete":True,"stage_a_selected":satc_stage_a["selected_for_stage_b"],"stage_b_role":"diagnostic_no_selection_authority" if (BASE/"satc/development_results.csv").exists() else "interrupted_not_run_to_completion","advanced_qualified":False,"bootstrap":"NOT_RUN","equal_rate":"NOT_RUN"}; write(BASE/"satc/satc_stack_freeze.json",satc_status)
 write(BASE/"comparison/task_family_analysis.json",{"status":"BLOCKED_BEFORE_FORMAL_COMPARISON","traditional":{k:f["metrics"] for k,f in freezes.items()},"satc_qualification":"NOT_RUN"})
 write(BASE/"comparison/actuator_analysis.json",{"status":"Traditional-only diagnostic","requested_clipped_applied_recorded":True,"methods":{x["method"]:{k:x[k] for k in ("physical_effort","thrust_saturation_rate","torque_saturation_rate","wrench_rate_rms")} for x in rows}})
 write(BASE/"comparison/deadline_analysis.json",{"deadline_definition":"component wall time > preregistered period","methods":{x["method"]:{k:x[k] for k in ("runtime_mean_ms","runtime_p95_ms","runtime_p99_ms","runtime_max_ms","deadline_miss_rate")} for x in rows}})
 hold=read(ROOT/"reproducibility/native_stack/r0s/resolved_holdout_manifest.json"); fp=len({x["case_semantic_fingerprint"] for x in hold["cases"]}); hold_ok=hold["identity_manifest_hash"]=="63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538" and hold["resolved_manifest_hash"]=="4a4b5d92027760e0d37176b7f768746c690a4ca53236ec60224cd423d6582df0" and not hold["execution_allowed"] and not hold["executed"] and hold["authoritative_runs"]==0 and fp==140
 write(BASE/"final/holdout_status.json",{"identity_hash":hold["identity_manifest_hash"],"resolved_hash":hold["resolved_manifest_hash"],"execution_allowed":False,"executed":False,"authoritative_runs":0,"compromised":not hold_ok,"semantic_fingerprint_count":fp,"semantic_fingerprint_check":hold_ok,"old_holdout_accessed":False})
 protected={"V1_V10_UNCHANGED":unchanged(["reproducibility/v2","reproducibility/v3","reproducibility/v4","reproducibility/v5","reproducibility/v6","reproducibility/v7","reproducibility/v8","reproducibility/v9","reproducibility/v10","src/uav_sway/v3","src/uav_sway/v4","src/uav_sway/v5","src/uav_sway/v6","src/uav_sway/v7","src/uav_sway/v8","src/uav_sway/v9","src/uav_sway/v10"]),"BENCHMARK_V1_1_UNCHANGED":unchanged(["reproducibility/native_stack/r0","reproducibility/native_stack/r0s","src/uav_sway/native_stack/case_semantics","src/uav_sway/native_stack/runner.py","src/uav_sway/native_stack/controller.py","src/uav_sway/native_stack/actuation.py","docs/native_stack/r0s"])}
 gate={"task":"P2-R1R1-NATIVE-TRADITIONAL-AND-SATC-BASELINE-DEVELOPMENT-FREEZE-R1","result":"BLOCKED_P2_TRADITIONAL_COMPETENCE","competent_traditional_count":0,"native_pid_frozen":True,"native_full_lqr_frozen":True,"native_task_lqr_frozen":True,"primary_native_traditional_frozen":False,"traditional_envelope_frozen":False,"satc_stack_frozen":False,"satc_native_advanced_qualified":False,"native_holdout_executed":False,"new_paper_selected":False,"paper_search":False,"paper_download":False,"paper_implementation":False,"paper_performance":False,**protected}; write(BASE/"final/final_gate.json",gate)
 write(BASE/"final/claim_matrix.json",{"allowed":["Development-only Traditional family results","correct P2-R1R1 block"],"forbidden":{"Primary Traditional frozen":True,"SATC Advanced":True,"Paper":True,"Holdout performance":True},"satc_stage_b_selection_authority":False,"new_paper_selected":False})
 DOC.mkdir(parents=True,exist_ok=True)
 for key,title in (("pid","NATIVE_PID_REPORT.md"),("full_lqr","NATIVE_FULL_LQR_REPORT.md"),("task_lqr","NATIVE_TASK_LQR_REPORT.md")): (DOC/title).write_text(f"# {title[:-3].replace('_',' ')}\n\nSelected family member: `{freezes[key]['selected']}`. Competence: **FAIL**. Full 200-case Development evidence is frozen; exact historical incumbent parameters were used.\n\n```json\n{json.dumps(freezes[key]['metrics'],indent=2,sort_keys=True)}\n```\n",encoding="utf-8")
 (DOC/"SATC_NATIVE_REPORT.md").write_text("# SATC Native report\n\nSATC qualification and freeze are NOT RUN because the prior Traditional competence hard gate failed. Stage A and any already-started Stage B runs are diagnostic only.\n",encoding="utf-8")
 (DOC/"DEVELOPMENT_COMPARISON.md").write_text("# Development comparison\n\nAll three Traditional families failed the frozen competence contract on complete 200-case Development. No formal Primary or Envelope exists, so SATC-vs-Primary, bootstrap, and Equal-Rate qualification are NOT RUN.\n",encoding="utf-8")
 (DOC/"P2_R2_ENTRY_CRITERIA.md").write_text("# P2-R2 entry criteria\n\nP2-R2 is not authorized. Required P2-R1R1 PASS is absent; Paper search, implementation, and performance remain forbidden.\n",encoding="utf-8")
 (DOC/"P2_R1R1_FINAL_REPORT.md").write_text("# P2-R1R1 final report\n\nResult: **BLOCKED_P2_TRADITIONAL_COMPETENCE**. PID, Full-LQR, and Task-LQR each completed the frozen 200-case Development protocol, including exact historical-incumbent comparison, and all three failed competence. Native Holdout remained unopened for performance.\n",encoding="utf-8")
 entries=[]
 for p in sorted([x for x in BASE.rglob("*") if x.is_file() and x.name!="evidence_manifest.json"]+[x for x in DOC.rglob("*") if x.is_file()]): entries.append({"path":p.relative_to(ROOT).as_posix(),"bytes":p.stat().st_size,"sha256":hashlib.sha256(p.read_bytes()).hexdigest()})
 write(BASE/"final/evidence_manifest.json",{"entry_count":len(entries),"entries":entries}); print(json.dumps(gate,indent=2,sort_keys=True))
if __name__=="__main__": main()
