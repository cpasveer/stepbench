"""What a check is, and the shared context every check reads from."""
from __future__ import annotations

from dataclasses import dataclass, field

import build123d as bd

from ..task import Task


@dataclass
class CheckResult:
    name: str
    ok: bool | None  # None = skipped because an earlier check failed
    reason: str
    value: float | None = None  # the one number the check turns on
    unit: str = ""
    details: dict = field(default_factory=dict)
    # False for a check that is reported but does not decide the verdict
    required: bool = True
    # the requirement this check belongs to: fits, clean, makeable, sturdy
    group: str = "fits"
    # True when the grader could not measure the part (a geometry-kernel
    # failure, not a property of the part); ok is then None
    error: bool = False

    def to_json(self) -> dict:
        return {
            "name": self.name,
            "status": "error" if self.error else ("skipped" if self.ok is None else ("pass" if self.ok else "fail")),
            "required": self.required,
            "group": self.group,
            "reason": self.reason,
            "value": self.value,
            "unit": self.unit,
            "details": self.details,
        }


@dataclass
class Context:
    """Everything the checks share. Checks may leave measurements here for
    later checks (HOLES leaves the measured grip of each bolt, which
    NO_INTERFERENCE uses to place the bolt heads)."""

    task: Task
    step_path: str
    shape: bd.Shape | None = None
    solid: bd.Solid | None = None
    _fixed: dict = field(default_factory=dict)
    grips: dict = field(default_factory=dict)  # (pose, bolt id) -> grip in mm
    # motion: the answer's placement at each pose (None = it stays put)
    answer_poses: list | None = None
    _moved: dict = field(default_factory=dict)
    # motion: a family may check more poses than task.json stores
    # (check_poses) and place the fixed parts itself at each (pose_scene)
    n_check_poses: int | None = None
    scene: object = None
    # grading for the verdict alone (generators): a check may stop at the
    # first failure it finds instead of measuring everything
    verdict_only: bool = False

    def poses(self) -> range:
        """The poses every check runs at."""
        return range(self.n_check_poses or len(self.task.poses))

    def stored(self, k: int) -> int:
        """The stored pose nearest to check pose ``k`` (for bolts, faces and
        obstacles, which task.json gives per stored pose)."""
        n, m = self.n_check_poses or len(self.task.poses), len(self.task.poses)
        if n == m or m == 1:
            return min(k, m - 1)
        return round(k * (m - 1) / (n - 1))

    def solid_at(self, pose: int):
        """The answer as placed at ``pose``. Static families: the answer itself."""
        if not self.answer_poses or self.answer_poses[pose] is None:
            return self.solid
        if pose not in self._moved:
            self._moved[pose] = self.solid.moved(self.answer_poses[pose].location())
        return self._moved[pose]

    def fixed(self, pose: int) -> dict[str, bd.Part]:
        """The fixed parts at check pose ``pose``: as the family's
        ``pose_scene`` places them (the scene the answer actually makes),
        else as task.json stores them."""
        if pose not in self._fixed:
            if self.scene is not None:
                from ..catalogue import fixed_part_solid

                places = self.scene(self.task, self, pose)
                self._fixed[pose] = {n: fixed_part_solid(itf, self.task.root).moved(places[n].location())
                                     for n, itf in self.task.parts.items()}
            else:
                self._fixed[pose] = self.task.fixed_solids(self.stored(pose))
        return self._fixed[pose]

    def obstacles(self, pose: int) -> dict[str, bd.Part]:
        key = ("obstacles", pose)
        if key not in self._fixed:
            self._fixed[key] = self.task.obstacle_solids(self.stored(pose))
        return self._fixed[key]


def volume(shape) -> float:
    if shape is None:
        return 0.0
    try:
        return float(shape.volume)
    except Exception:  # empty result of a boolean
        return 0.0


def area(shape) -> float:
    if shape is None:
        return 0.0
    try:
        return float(shape.area)
    except Exception:
        return 0.0
