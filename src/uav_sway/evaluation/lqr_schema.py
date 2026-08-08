"""Frozen S4 LQR raw-CSV contract."""

from __future__ import annotations

from .controlled_schema import controlled_schema_columns


def lqr_schema_columns(n_links: int) -> list[str]:
    return controlled_schema_columns(n_links) + [
        "lqr_feedback_ax", "lqr_state_norm",
    ]


def lqr_schema_description(n_links: int) -> dict:
    return {"columns": lqr_schema_columns(n_links), "controller": "lqr", "protocol_mode": "free_flight_controlled", "anchor_active": False}
