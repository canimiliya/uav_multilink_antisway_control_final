"""Byte-stable UTF-8 wind-file contract."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .wind_profiles import WindSeries


WIND_COLUMNS = ["time", "wind_x", "wind_y", "wind_z", "profile", "seed"]


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_wind_csv(path: str | Path, series: WindSeries) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        stream.write(",".join(WIND_COLUMNS) + "\n")
        writer = csv.writer(stream, lineterminator="\n")
        seed_text = "" if series.seed is None else str(series.seed)
        for time, wind_x in zip(series.time, series.wind_x):
            writer.writerow([format(float(time), ".17g"), format(float(wind_x), ".17g"), "0", "0", series.profile, seed_text])
    return path


def read_wind_csv(path: str | Path) -> dict[str, np.ndarray | str | int | None]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != WIND_COLUMNS:
            raise ValueError(f"unexpected wind columns: {reader.fieldnames}")
        rows = list(reader)
    if not rows:
        raise ValueError("empty wind CSV")
    seed = rows[0]["seed"]
    return {
        "time": np.asarray([float(row["time"]) for row in rows], dtype=float),
        "wind_x": np.asarray([float(row["wind_x"]) for row in rows], dtype=float),
        "wind_y": np.asarray([float(row["wind_y"]) for row in rows], dtype=float),
        "wind_z": np.asarray([float(row["wind_z"]) for row in rows], dtype=float),
        "profile": rows[0]["profile"],
        "seed": None if seed == "" else int(seed),
    }


def write_manifest(path: str | Path, entries: list[dict]) -> None:
    Path(path).write_text(json.dumps({"generator_version": "s2-wind-profiles-r1", "files": entries}, indent=2) + "\n", encoding="utf-8", newline="\n")
