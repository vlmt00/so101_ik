"""Coordinate IK, validation and collision; reject unverified motion."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class MotionResult:
    accepted: bool
    q: np.ndarray
    reason: str
    position_error_m: float | None = None
    direction_error_deg: float | None = None

class SimulationController:
    def __init__(self, model, kinematics, collision_checker, *, allow_unchecked_collision=False):
        self.model = model
        self.kin = kinematics
        self.collision = collision_checker
        self.allow_unchecked_collision = allow_unchecked_collision
        self.q = np.asarray(model.robot.q, dtype=float).copy()

    def move_tcp(self, xyz, direction=(0, 0, -1)):
        solution = self.kin.ik(xyz, direction=direction, q0=self.q)
        if not solution.success:
            return MotionResult(False, self.q.copy(), "IK infeasible: "+solution.message,
                                solution.position_error_m, solution.direction_error_deg)
        lower, upper = self.model.qlim
        if np.any(solution.q < lower) or np.any(solution.q > upper):
            return MotionResult(False, self.q.copy(), "Joint limits violated")
        check = self.collision.check(solution.q)
        if check.collision is True:
            return MotionResult(False, self.q.copy(), check.message)
        if not check.checked and not self.allow_unchecked_collision:
            return MotionResult(False, self.q.copy(), check.message+"; collision unverified")
        self.q = solution.q.copy()
        self.model.robot.q = self.q
        message = "accepted" if check.checked else "accepted WITHOUT collision verification (debug only)"
        return MotionResult(True, self.q.copy(), message, solution.position_error_m, solution.direction_error_deg)
