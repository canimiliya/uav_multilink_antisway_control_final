"""Frozen Native-Stack v1.1 physical case semantics."""

from .resolver import (
    NATIVE_CASE_SEMANTICS_VERSION,
    NativeCaseResolver,
    ResolvedNativeCase,
    ResolvedReference,
)
from .authoritative import AUTHORITATIVE_EXECUTION, AuthoritativeNativeCaseRunner

__all__ = [
    "NATIVE_CASE_SEMANTICS_VERSION",
    "AUTHORITATIVE_EXECUTION",
    "AuthoritativeNativeCaseRunner",
    "NativeCaseResolver",
    "ResolvedNativeCase",
    "ResolvedReference",
]
