"""Tier-2 global localization: Monte-Carlo Localization (localization report §3.1).

A multi-hypothesis particle filter — the correct tool for a *symmetric* field
where the belief is inherently multi-modal and a single-hypothesis EKF/UKF can
lock onto the wrong half. Each step:

  1. **Drift**    — apply the odometry delta from Tier-1.
  2. **Diffuse**  — add process noise.
  3. **Measure**  — weight each particle by looking observed line points up in the
                    precomputed likelihood-field map (O(1) Chamfer matching).
  4. **Resample** — importance resampling + a fraction of **explorer particles**
                    re-seeded uniformly for kidnapped-robot / penalty-return recovery.
  5. **Estimate** — weighted mean + covariance -> the ``map -> odom`` correction.

The :class:`ParticleFilter` is ROS-free so it can be unit-tested directly.
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from soccer_msgs.msg import FieldFeature, FieldFeatureArray
from tf2_ros import TransformBroadcaster

from soccer_localization.field_model import SoccerbotField


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


class ParticleFilter:
    """Field-feature MCL over poses [x, y, theta]."""

    def __init__(self, field: SoccerbotField, num_particles: int = 300,
                 explorer_frac: float = 0.05, seed: int = 0) -> None:
        self.field = field
        self.n = num_particles
        self.explorer_frac = explorer_frac
        self.rng = np.random.default_rng(seed)
        self.particles = np.stack(
            [self.field.random_pose(self.rng) for _ in range(self.n)]
        )
        self.weights = np.full(self.n, 1.0 / self.n)

    # 1-2. Motion model: drift by odometry delta + diffuse with noise.
    def predict(self, d: np.ndarray, noise=(0.02, 0.02, 0.02)) -> None:
        c, s = np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])
        self.particles[:, 0] += c * d[0] - s * d[1]
        self.particles[:, 1] += s * d[0] + c * d[1]
        self.particles[:, 2] = _wrap(self.particles[:, 2] + d[2])
        self.particles += self.rng.normal(0.0, noise, self.particles.shape)
        self.particles[:, 2] = _wrap(self.particles[:, 2])

    # 3. Measurement model: likelihood-field lookup of transformed line points.
    def update(self, pts_base: np.ndarray) -> None:
        if pts_base.shape[0] == 0:
            return
        log_w = np.zeros(self.n)
        for i in range(self.n):
            x, y, th = self.particles[i]
            c, s = np.cos(th), np.sin(th)
            world = np.empty_like(pts_base)
            world[:, 0] = x + c * pts_base[:, 0] - s * pts_base[:, 1]
            world[:, 1] = y + s * pts_base[:, 0] + c * pts_base[:, 1]
            lik = self.field.line_likelihood(world)
            log_w[i] = np.sum(np.log(lik + 1e-6))
        log_w -= log_w.max()
        self.weights *= np.exp(log_w)
        total = self.weights.sum()
        self.weights = (np.full(self.n, 1.0 / self.n)
                        if total < 1e-12 else self.weights / total)

    # 4. Systematic resampling + explorer-particle injection.
    def resample(self) -> None:
        if 1.0 / np.sum(self.weights ** 2) > self.n / 2.0:
            return  # effective sample size healthy -> skip
        positions = (np.arange(self.n) + self.rng.random()) / self.n
        cumsum = np.cumsum(self.weights)
        idx = np.searchsorted(cumsum, positions)
        idx = np.clip(idx, 0, self.n - 1)
        self.particles = self.particles[idx]
        n_expl = int(self.explorer_frac * self.n)
        for k in range(n_expl):  # kidnapped-robot recovery
            self.particles[k] = self.field.random_pose(self.rng)
        self.weights = np.full(self.n, 1.0 / self.n)

    # 5. Weighted estimate (circular mean for theta).
    def estimate(self) -> np.ndarray:
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        th = np.arctan2(
            np.average(np.sin(self.particles[:, 2]), weights=self.weights),
            np.average(np.cos(self.particles[:, 2]), weights=self.weights),
        )
        return np.array([x, y, th])


class MclNode(Node):
    """ROS wrapper: consumes field features + odom, publishes map->odom."""

    def __init__(self) -> None:
        super().__init__("mcl_node")
        self.declare_parameter("num_particles", 300)
        self.declare_parameter("explorer_frac", 0.05)
        self._field = SoccerbotField()
        self._pf = ParticleFilter(
            self._field,
            int(self.get_parameter("num_particles").value),
            float(self.get_parameter("explorer_frac").value),
        )
        self._last_odom = None

        self._tf = TransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "mcl_pose", 10)
        self.create_subscription(FieldFeatureArray, "field_features", self._on_feats, 5)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_timer(0.1, self._publish)  # 10 Hz map->odom
        self.get_logger().info("mcl_node up (%d particles)." % self._pf.n)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cur = np.array([p.x, p.y, yaw])
        if self._last_odom is not None:
            d = cur - self._last_odom
            d[2] = _wrap(np.array([d[2]]))[0]
            self._pf.predict(d)
        self._last_odom = cur

    def _on_feats(self, msg: FieldFeatureArray) -> None:
        pts = np.array(
            [[f.position.x, f.position.y] for f in msg.features
             if f.type == FieldFeature.TYPE_LINE_POINT]
        )
        self._pf.update(pts if pts.ndim == 2 else pts.reshape(0, 2))
        self._pf.resample()

    def _publish(self) -> None:
        est = self._pf.estimate()
        # In a full system the map->odom transform is est composed with the inverse
        # of the latest odom; for soccerbot we publish the estimate as map->base_link.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = float(est[0])
        t.transform.translation.y = float(est[1])
        t.transform.rotation.z = float(np.sin(est[2] / 2))
        t.transform.rotation.w = float(np.cos(est[2] / 2))
        self._tf.sendTransform(t)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(est[0])
        msg.pose.pose.position.y = float(est[1])
        msg.pose.pose.orientation.z = t.transform.rotation.z
        msg.pose.pose.orientation.w = t.transform.rotation.w
        self._pose_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MclNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
