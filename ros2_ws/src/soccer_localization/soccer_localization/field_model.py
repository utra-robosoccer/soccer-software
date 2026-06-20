"""Field model + likelihood-field map (localization report §3.1, §6).

Holds the exact field geometry (lines + centre circle + goalposts) and bakes a
**likelihood-field map**: a grid whose value at each cell is a Gaussian of the
distance to the nearest field line. Weighting an observed line point is then an
O(1) lookup — the efficient form of Chamfer matching the top RoboCup teams use
(Bit-Bots ``lines.png``). ``MinibotField`` is the single source of field truth,
reused by both the MCL and any visualization.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt

    _HAVE_SCIPY = True
except Exception:  # pragma: no cover
    _HAVE_SCIPY = False


@dataclass
class MinibotField:
    """A small symmetric soccer field, centred at the origin (metres)."""

    length: float = 6.0          # x extent (touchline to touchline)
    width: float = 4.0           # y extent
    centre_circle_r: float = 0.75
    resolution: float = 0.05     # likelihood-field grid cell size (m)
    line_sigma: float = 0.15     # Gaussian falloff of the likelihood field (m)
    goalposts: list = field(default_factory=list)

    def __post_init__(self) -> None:
        hl, hw = self.length / 2.0, self.width / 2.0
        # Goalposts at both goals (the asymmetry-breaking landmarks).
        self.goalposts = [
            (-hl, -1.0), (-hl, 1.0),  # own goal
            (hl, -1.0), (hl, 1.0),    # opponent goal
        ]
        self._segments = self._build_segments()
        self._grid, self._origin = self._bake_likelihood_field()

    # ── Geometry ──
    def _build_segments(self) -> list[tuple[float, float, float, float]]:
        hl, hw = self.length / 2.0, self.width / 2.0
        segs = [
            (-hl, -hw, hl, -hw), (-hl, hw, hl, hw),    # touchlines
            (-hl, -hw, -hl, hw), (hl, -hw, hl, hw),    # goal lines
            (0.0, -hw, 0.0, hw),                        # halfway line
        ]
        # Approximate the centre circle with chords.
        n = 24
        for i in range(n):
            a0 = 2 * np.pi * i / n
            a1 = 2 * np.pi * (i + 1) / n
            segs.append((
                self.centre_circle_r * np.cos(a0), self.centre_circle_r * np.sin(a0),
                self.centre_circle_r * np.cos(a1), self.centre_circle_r * np.sin(a1),
            ))
        return segs

    # ── Likelihood field ──
    def _bake_likelihood_field(self):
        margin = 0.5
        hl, hw = self.length / 2.0 + margin, self.width / 2.0 + margin
        nx = int(2 * hl / self.resolution)
        ny = int(2 * hw / self.resolution)
        occ = np.ones((ny, nx), dtype=np.uint8)  # 1 = free, 0 = on a line
        origin = (-hl, -hw)

        def to_cell(x, y):
            return (int((x - origin[0]) / self.resolution),
                    int((y - origin[1]) / self.resolution))

        for (x0, y0, x1, y1) in self._segments:
            steps = int(max(abs(x1 - x0), abs(y1 - y0)) / self.resolution) + 1
            for t in np.linspace(0.0, 1.0, steps):
                cx, cy = to_cell(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
                if 0 <= cx < nx and 0 <= cy < ny:
                    occ[cy, cx] = 0

        if _HAVE_SCIPY:
            dist = distance_transform_edt(occ) * self.resolution
        else:  # coarse fallback: 0 on lines, large elsewhere
            dist = np.where(occ == 0, 0.0, 1.0)
        likelihood = np.exp(-0.5 * (dist / self.line_sigma) ** 2)
        return likelihood.astype(np.float32), origin

    def line_likelihood(self, pts_xy: np.ndarray) -> np.ndarray:
        """Likelihood in [0,1] for world-frame points (N,2) via O(1) grid lookup."""
        ny, nx = self._grid.shape
        cx = ((pts_xy[:, 0] - self._origin[0]) / self.resolution).astype(int)
        cy = ((pts_xy[:, 1] - self._origin[1]) / self.resolution).astype(int)
        inside = (cx >= 0) & (cx < nx) & (cy >= 0) & (cy < ny)
        out = np.full(len(pts_xy), 1e-3, dtype=np.float32)
        out[inside] = self._grid[cy[inside], cx[inside]]
        return out

    def random_pose(self, rng: np.random.Generator) -> np.ndarray:
        """A uniformly random on-field pose [x, y, theta] (for explorer particles)."""
        hl, hw = self.length / 2.0, self.width / 2.0
        return np.array([
            rng.uniform(-hl, hl), rng.uniform(-hw, hw), rng.uniform(-np.pi, np.pi)
        ])
