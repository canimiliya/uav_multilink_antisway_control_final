from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "reproducibility/v9/final/evidence_manifest.json"


def included_files() -> list[Path]:
    roots = [
        ROOT / "docs/v9",
        ROOT / "reproducibility/v9",
        ROOT / "src/uav_sway/v9",
        ROOT / "tests/v9",
    ]
    files = {
        path
        for base in roots
        for path in base.rglob("*")
        if path.is_file() and path != OUTPUT and "__pycache__" not in path.parts
    }
    files.update(
        {
            ROOT / "scripts/build_v9_contract.py",
            ROOT / "scripts/generate_v9_training_data.py",
            ROOT / "scripts/train_v9_neural_predictor.py",
            ROOT / "scripts/build_v9_evidence_manifest.py",
            ROOT / "tests/release/test_release_surface.py",
        }
    )
    return sorted(files)


def canonical_bytes(path: Path) -> bytes:
    content = path.read_bytes()
    try:
        text = content.decode("utf-8")
    except UnicodeDecodeError:
        return content
    return text.replace("\r\n", "\n").encode("utf-8")


def main() -> None:
    records: dict[str, dict[str, str | int]] = {}
    for path in included_files():
        relative = path.relative_to(ROOT).as_posix()
        content = canonical_bytes(path)
        records[relative] = {
            "sha256": hashlib.sha256(content).hexdigest(),
            "bytes": len(content),
        }

    payload = {
        "task": "V9-NEURAL-PREDICTOR-MPC-RECENT-PAPER-STRONG-BASELINE-AND-FINAL-CLOSURE-R1",
        "result": "BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED",
        "project_research_complete": False,
        "development_executed": False,
        "holdout_executed": False,
        "hash_mode": "UTF-8 text normalized to LF; binary files hashed as raw bytes",
        "files": records,
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
