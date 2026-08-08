"""OSQP wrapper with deterministic warm-start support."""

from __future__ import annotations

import numpy as np


class OSQPPreviewSolver:
    def __init__(self, eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_start=True):
        import osqp
        from scipy import sparse
        self.osqp = osqp
        self.sparse = sparse
        self.kwargs = dict(eps_abs=float(eps_abs), eps_rel=float(eps_rel), max_iter=int(max_iter),
                           warm_start=bool(warm_start), verbose=False, polish=False)
        self._problem = None
        self._shape = None
        self._last = None

    def solve(self, qp):
        P = self.sparse.triu(self.sparse.csc_matrix((qp.P + qp.P.T) / 2.0)).tocsc()
        A = self.sparse.csc_matrix(qp.A)
        if self._problem is None or self._shape != P.shape:
            self._problem = self.osqp.OSQP(); self._problem.setup(P=P, q=qp.q, A=A, l=qp.lower, u=qp.upper, **self.kwargs)
            self._shape = P.shape
        else:
            self._problem.update(q=qp.q, l=qp.lower, u=qp.upper, Px=P.data)
        if self._last is not None:
            self._problem.warm_start(x=self._last)
        result = self._problem.solve()
        if result.x is None or result.info.status not in ("solved", "solved inaccurate"):
            raise RuntimeError(f"OSQP failed: {result.info.status}")
        self._last = np.asarray(result.x, dtype=float).copy()
        return self._last, result.info
