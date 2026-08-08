"""Scenario configuration loading."""

from pathlib import Path

import yaml


def load_scenario_config(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)
