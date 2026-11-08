"""Constrained numerical IK: TCP position (3) + tip direction (2)."""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares

@dataclass(frozen=True)
class IKResult:
    success: bool
    q: np.ndarray
    position_error_m: float
    direction_error_deg: float
    message: str

class Kinematics:
    def __init__(self, model):
        self.model = model
        self.robot = model.robot
        axis = np.asarray(model.tcp.tip_axis, dtype=float)
        self.tip_axis = axis / np.linalg.norm(axis)

    def fk(self, q) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.shape != (self.model.n,):
            raise ValueError(f"q must have shape ({self.model.n},)")
        return np.asarray(self.robot.fkine(q, end=self.model.tcp.link_name).A, dtype=float)

    def tip_direction(self, q):
        return self.fk(q)[:3, :3] @ self.tip_axis

    @staticmethod
    def _unit(vec):
        v = np.asarray(vec, dtype=float)
        if v.shape != (3,) or not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-12:
            raise ValueError("Expected a finite, non-zero 3-vector")
        return v / np.linalg.norm(v)

    def evaluate(self, q, xyz, direction=(0, 0, -1)):
        T = self.fk(q)
        pos_err = float(np.linalg.norm(T[:3, 3] - np.asarray(xyz)))
        dp = np.clip(np.dot(T[:3, :3] @ self.tip_axis, self._unit(direction)), -1., 1.)
        return pos_err, float(np.rad2deg(np.arccos(dp)))

    def ik(self, xyz, direction=(0, 0, -1), q0=None, *,
           position_tolerance_m=0.002, angle_tolerance_deg=2.0,
           direction_weight_m=0.10, max_nfev=250):
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError("xyz must be a finite 3-vector in metres")
        d = self._unit(direction)
        lower, upper = self.model.qlim
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower >= upper):
            raise ValueError("Finite and ordered joint limits required")
        seed = (lower + upper)/2 if q0 is None else np.asarray(q0, dtype=float)
        if seed.shape != (self.model.n,) or not np.all(np.isfinite(seed)):
            raise ValueError("q0 has incorrect shape or invalid numbers")
        seed = np.clip(seed, lower, upper)

        def residual(q):
            T = self.fk(q)
            return np.r_[T[:3, 3]-xyz, direction_weight_m*(T[:3, :3]@self.tip_axis-d)]

        result = least_squares(residual, seed, bounds=(lower, upper),
                               max_nfev=max_nfev, xtol=1e-10, ftol=1e-10, gtol=1e-10)
        position_error, angle_error = self.evaluate(result.x, xyz, d)
        ok = bool(result.success and position_error <= position_tolerance_m
                  and angle_error <= angle_tolerance_deg)
        return IKResult(ok, result.x.copy(), position_error, angle_error, str(result.message))
