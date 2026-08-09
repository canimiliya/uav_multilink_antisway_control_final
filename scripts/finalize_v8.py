"""Create the deterministic V8 final evidence manifest without simulations."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
V8 = ROOT / "reproducibility/v8"
FINAL = V8 / "final"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    paths = []
    for folder in (V8 / "r0", V8 / "development", V8 / "paper", V8 / "holdout", FINAL, ROOT / "docs/v8"):
        for path in folder.rglob("*"):
            if path.is_file() and "_cache" not in path.parts and path.name != "evidence_manifest.json":
                paths.append(path)
    payload = {
        "task": "V8-DOUBLE-PENDULUM-RECENT-PAPER-STRONG-BASELINE-AND-PROJECT-CLOSURE-R1",
        "result": "V8_NO_QUALIFIED_DOUBLE_PENDULUM_PAPER_BASELINE",
        "project_research_complete": False,
        "holdout_executed": False,
        "files": {
            str(path.relative_to(ROOT)).replace("\\", "/"): {
                "sha256": digest(path),
                "bytes": path.stat().st_size,
            }
            for path in sorted(paths)
        },
    }
    output = FINAL / "evidence_manifest.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"file_count": len(paths), "manifest": str(output.relative_to(ROOT))}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
