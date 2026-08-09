"""Execute all four frozen V3 controllers on the one-shot Holdout manifest."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from run_v3_r1_baselines import read_json, run_case, sha256  # noqa: E402


R4 = ROOT / "reproducibility/v3/r4"
MANIFEST = R4 / "holdout_execution_manifest.json"
PROTOCOL = R4 / "holdout_protocol_freeze.json"
STATE = R4 / "holdout_execution_state.json"
RESULTS = R4 / "holdout_results.csv"
CACHE = R4 / ".holdout_case_cache"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def csv_value(value: object) -> object:
    if isinstance(value, (list, dict)):
        return json.dumps(value, sort_keys=True, separators=(",", ":"))
    return value


def write_csv(path: Path, rows: list[dict]) -> None:
    columns = sorted({key for row in rows for key in row if not key.startswith("_")})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_value(row.get(key, "")) for key in columns})


def load_controllers(manifest: dict) -> list[tuple[str, dict]]:
    result = []
    for item in manifest["controllers"]:
        freeze = read_json(ROOT / item["freeze"])
        parameters = freeze["parameters"]
        if parameters["candidate_id"] != item["candidate_id"]:
            raise RuntimeError(f"frozen candidate drift: {item['candidate_id']}")
        result.append((item["kind"], parameters))
    return result


def cache_path(kind: str, candidate_id: str, sample_id: str) -> Path:
    return CACHE / f"{kind}__{candidate_id}__{sample_id}.json"


def execute_job(kind: str, parameters: dict, sample: dict) -> dict:
    return run_case(kind, parameters, sample)


def verify_frozen_inputs(protocol: dict) -> None:
    for relative, expected in protocol["file_sha256"].items():
        actual = sha256(ROOT / relative)
        if actual != expected:
            raise RuntimeError(f"frozen input drift: {relative}: {actual} != {expected}")
    for relative, expected in protocol["protected_trees"].items():
        actual = git("rev-parse", f"HEAD:{relative}")
        if actual != expected:
            raise RuntimeError(f"protected tree drift: {relative}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--protocol-freeze-head", required=True)
    parser.add_argument("--resume-infrastructure", action="store_true")
    parser.add_argument("--retry-reason")
    args = parser.parse_args()
    if args.workers < 1 or args.workers > 24:
        raise ValueError("workers must be in [1,24]")
    if git("branch", "--show-current") != "research-v3":
        raise RuntimeError("wrong branch")
    head = git("rev-parse", "HEAD")
    if head != args.protocol_freeze_head:
        raise RuntimeError("execution HEAD is not the declared protocol freeze HEAD")
    protocol = read_json(PROTOCOL)
    manifest = read_json(MANIFEST)
    verify_frozen_inputs(protocol)
    if manifest["sample_count"] != 57 or manifest["authoritative_run_count"] != 228:
        raise RuntimeError("Holdout execution manifest count drift")
    if RESULTS.exists():
        raise RuntimeError("one-shot Holdout results already exist; second execution refused")

    retries: list[dict] = []
    if STATE.exists():
        previous = read_json(STATE)
        if previous["status"] == "COMPLETED":
            raise RuntimeError("one-shot Holdout already completed")
        if not args.resume_infrastructure or not args.retry_reason:
            raise RuntimeError("interrupted execution requires --resume-infrastructure and --retry-reason")
        if previous["protocol_freeze_head"] != head:
            raise RuntimeError("resume HEAD differs from original execution HEAD")
        retries = list(previous.get("retries", []))
        retries.append({"reason": args.retry_reason, "same_commit": True, "same_config": True, "scope": "missing_samples_only"})
    elif args.resume_infrastructure:
        raise RuntimeError("no interrupted Holdout state exists")
    elif git("status", "--porcelain"):
        raise RuntimeError("Holdout must start from a clean protocol-freeze worktree")

    CACHE.mkdir(parents=True, exist_ok=True)
    controllers = load_controllers(manifest)
    samples = manifest["samples"]
    jobs = [(kind, parameters, sample) for sample in samples for kind, parameters in controllers]
    pending = [job for job in jobs if not cache_path(job[0], job[1]["candidate_id"], job[2]["sample_id"]).exists()]
    write_json(STATE, {
        "status": "RUNNING",
        "protocol_freeze_head": head,
        "manifest_sha256": sha256(MANIFEST),
        "authoritative_jobs": len(jobs),
        "completed_from_checkpoint": len(jobs) - len(pending),
        "pending": len(pending),
        "workers": args.workers,
        "retries": retries,
        "scientific_results_inspected_during_execution": False,
    })

    try:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(execute_job, *job): job for job in pending}
            completed = len(jobs) - len(pending)
            for future in as_completed(futures):
                kind, parameters, sample = futures[future]
                row = future.result()
                write_json(cache_path(kind, parameters["candidate_id"], sample["sample_id"]), row)
                completed += 1
                if completed % 12 == 0 or completed == len(jobs):
                    print(f"v3-r4-holdout {completed}/{len(jobs)}", flush=True)
    except BaseException as exc:
        write_json(STATE, {
            "status": "INTERRUPTED_REQUIRES_CLASSIFICATION",
            "protocol_freeze_head": head,
            "manifest_sha256": sha256(MANIFEST),
            "authoritative_jobs": len(jobs),
            "completed_checkpoints": len(list(CACHE.glob("*.json"))),
            "exception_type": type(exc).__name__,
            "exception_message": str(exc),
            "retries": retries,
            "instruction": "Retry missing samples only if this is a pure infrastructure failure. If code must change, declare HOLDOUT_COMPROMISED_REQUIRES_NEW_RESEARCH_VERSION.",
        })
        raise

    rows = []
    for kind, parameters, sample in jobs:
        path = cache_path(kind, parameters["candidate_id"], sample["sample_id"])
        if not path.exists():
            raise RuntimeError(f"missing completed sample checkpoint: {path.name}")
        rows.append(read_json(path))
    write_csv(RESULTS, rows)
    write_json(STATE, {
        "status": "COMPLETED",
        "protocol_freeze_head": head,
        "manifest_sha256": sha256(MANIFEST),
        "result_sha256": sha256(RESULTS),
        "sample_count": 57,
        "controller_count": 4,
        "authoritative_runs": 228,
        "workers": args.workers,
        "retries": retries,
        "compromised": False,
        "scientific_results_inspected_during_execution": False,
    })
    resolved_cache = CACHE.resolve()
    if resolved_cache.parent != R4.resolve() or resolved_cache.name != ".holdout_case_cache":
        raise RuntimeError("unsafe temporary checkpoint path")
    shutil.rmtree(resolved_cache)
    print(json.dumps({"result": "V3_ONE_SHOT_HOLDOUT_EXECUTION_COMPLETE", "runs": 228, "retries": len(retries)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
