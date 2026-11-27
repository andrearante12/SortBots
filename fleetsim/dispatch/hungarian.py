"""Minimum-cost assignment (Kuhn-Munkres with potentials), O(n^2 m).

No scipy on the system python3 (fleetsim's only dependencies are numpy and
PyYAML), so this is the shortest-augmenting-path form of the Hungarian
method: rows are added one at a time, each by a Dijkstra-like search over
reduced costs, and the dual potentials keep every reduced cost >= 0. The
inner loop over columns is one numpy expression, so a 20 x 20 solve takes a
few milliseconds.

Rectangular matrices are fine: with more columns than rows every row gets a
column; with more rows than columns the matrix is transposed and every
column gets a row. Costs must be finite (use a large number for "not
allowed" and filter those pairs out afterwards); negative costs are fine.
"""
from __future__ import annotations

import numpy as np


def solve(cost) -> list[int]:
    """-> col[i] for every row i: its assigned column, or -1 (only when there
    are more rows than columns). Minimises the sum of cost[i, col[i]]."""
    c = np.asarray(cost, dtype=float)
    if c.ndim != 2:
        raise ValueError(f"cost must be 2-D, got shape {c.shape}")
    n, m = c.shape
    if n == 0 or m == 0:
        return [-1] * n
    if not np.isfinite(c).all():
        raise ValueError("cost must be finite (use a large number for a forbidden pair)")
    if n > m:
        row_of_col = solve(c.T)
        out = [-1] * n
        for j, i in enumerate(row_of_col):
            out[i] = j
        return out
    INF = np.inf
    u = np.zeros(n + 1)            # row potentials, 1-based (0 is the virtual row)
    v = np.zeros(m + 1)            # column potentials, 1-based (0 is the virtual column)
    p = np.zeros(m + 1, dtype=np.int64)    # p[j]: row matched to column j, 0 = none
    way = np.zeros(m + 1, dtype=np.int64)  # previous column on the augmenting path
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(m + 1, INF)
        used = np.zeros(m + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            free = ~used[1:]
            cur = c[i0 - 1] - u[i0] - v[1:]
            better = free & (cur < minv[1:])
            minv[1:][better] = cur[better]
            way[1:][better] = j0
            masked = np.where(free, minv[1:], INF)
            j1 = int(np.argmin(masked)) + 1
            delta = masked[j1 - 1]
            # Matched rows of the used columns are distinct, so the
            # fancy-index += never hits one row twice.
            u[p[used]] += delta
            v[used] -= delta
            minv[1:][free] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:  # flip the augmenting path
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    out = [-1] * n
    for j in range(1, m + 1):
        if p[j]:
            out[p[j] - 1] = j - 1
    return out


def total(cost, cols) -> float:
    c = np.asarray(cost, dtype=float)
    return float(sum(c[i, j] for i, j in enumerate(cols) if j >= 0))
