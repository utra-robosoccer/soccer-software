"""Unit tests for the field model + MCL particle filter (no ROS required)."""
import numpy as np

from soccer_localization.field_model import MinibotField
from soccer_localization.mcl_node import ParticleFilter


def test_likelihood_higher_on_lines():
    f = MinibotField()
    on_line = np.array([[0.0, 1.5]])      # on the halfway line (y in [-2,2])
    off_line = np.array([[1.3, 0.4]])     # empty grass
    assert f.line_likelihood(on_line)[0] > f.line_likelihood(off_line)[0]


def _observations(field: MinibotField, pose, rng, n=60):
    """Sample world line points visible from `pose` and express them in base frame."""
    x, y, th = pose
    pts = []
    for (x0, y0, x1, y1) in field._segments:
        for t in np.linspace(0, 1, 12):
            wx, wy = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            dx, dy = wx - x, wy - y
            c, s = np.cos(-th), np.sin(-th)
            bx, by = c * dx - s * dy, s * dx + c * dy
            if 0.2 < bx < 4.0 and abs(by) < bx:  # crude forward FOV
                pts.append([bx, by])
    pts = np.array(pts)
    idx = rng.choice(len(pts), size=min(n, len(pts)), replace=False)
    return pts[idx]


def test_mcl_converges_when_seeded_near_truth():
    field = MinibotField()
    rng = np.random.default_rng(1)
    true_pose = np.array([-1.0, 0.5, 0.2])

    pf = ParticleFilter(field, num_particles=400, explorer_frac=0.0, seed=2)
    # Seed particles around the truth (a global init would be symmetric-ambiguous;
    # explorer particles handle that recovery, tested implicitly elsewhere).
    pf.particles = true_pose + rng.normal(0, [0.3, 0.3, 0.2], size=(pf.n, 3))

    for _ in range(8):
        pf.predict(np.zeros(3), noise=(0.01, 0.01, 0.01))
        pf.update(_observations(field, true_pose, rng))
        pf.resample()

    est = pf.estimate()
    assert np.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.4


def test_explorer_particles_reseed_on_resample():
    field = MinibotField()
    pf = ParticleFilter(field, num_particles=100, explorer_frac=0.1, seed=3)
    # Force a degenerate weight distribution so resampling triggers.
    pf.weights = np.zeros(pf.n)
    pf.weights[0] = 1.0
    before = pf.particles.copy()
    pf.resample()
    # At least the explorer fraction should have been re-seeded (changed).
    changed = np.any(~np.isclose(pf.particles, before), axis=1).sum()
    assert changed >= int(0.1 * pf.n)
