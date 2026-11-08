"""Explicit, fail-closed collision check with RTB-P collision APIs."""
from dataclasses import dataclass

@dataclass(frozen=True)
class CollisionResult:
    checked: bool
    collision: bool | None
    message: str

class CollisionChecker:
    def __init__(self, model, obstacles=()):
        self.robot = model.robot
        self.obstacles = list(obstacles)

    def check(self, q):
        if not callable(getattr(self.robot, "isselfcollided", None)):
            return CollisionResult(False, None, "RTB-P self-collision API unavailable")
        try:
            if self.robot.isselfcollided(q):
                return CollisionResult(True, True, "Self collision")
            if self.obstacles:
                if not callable(getattr(self.robot, "iscollided", None)):
                    return CollisionResult(False, None, "Obstacle collision API unavailable")
                for obstacle in self.obstacles:
                    if self.robot.iscollided(q, obstacle):
                        return CollisionResult(True, True, "Obstacle collision")
            return CollisionResult(True, False, "No collision detected")
        except (RuntimeError, ValueError, ImportError, AttributeError, TypeError) as exc:
            return CollisionResult(False, None, f"Collision checking failed: {exc}")
