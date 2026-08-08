"""V3 research-contract primitives.

This package defines the common three-axis outer-loop command contract.  It
does not implement or tune a controller.
"""

from .contracts import (
    V3AccelerationCommand,
    V3AccelerationLimiter,
    V3Controller,
    V3_INNER_LOOP_CONTRACT,
)

__all__ = [
    "V3AccelerationCommand",
    "V3AccelerationLimiter",
    "V3Controller",
    "V3_INNER_LOOP_CONTRACT",
]
