from dataclasses import dataclass
from pathlib import Path
import numpy as np

try:
    from omegaconf import MISSING
except ImportError:
    MISSING = "???"

from .trajectory_base import BaseTrajectory, BaseTrajectoryConfig

@dataclass
class SplineTrajectoryConfig(BaseTrajectoryConfig):
    config: Path | None = None  # Path to YAML configuration file
    type: str = MISSING  # "quintic" or "septic"
    start_pos: list[float] = MISSING
    end_pos: list[float] = MISSING
    start_vel: list[float] | None = None
    end_vel: list[float] | None = None
    start_acc: list[float] | None = None
    end_acc: list[float] | None = None
    start_jerk: list[float] | None = None
    end_jerk: list[float] | None = None
    target_class: str = "SplineTrajectory"


class SplineTrajectory(BaseTrajectory):
    """Generates a polynomial trajectory (quintic or septic) for multiple joints."""

    def __init__(self, cfg: SplineTrajectoryConfig, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        self.start_pos = np.array(cfg.start_pos)
        self.end_pos = np.array(cfg.end_pos)
        self.num_joints = len(cfg.start_pos)
        self.type = cfg.type

        if len(cfg.end_pos) != self.num_joints:
            raise ValueError("Start and end positions must have the same length.")

        self.start_vel = np.array(cfg.start_vel) if cfg.start_vel is not None else np.zeros(self.num_joints)
        self.end_vel = np.array(cfg.end_vel) if cfg.end_vel is not None else np.zeros(self.num_joints)
        self.start_acc = np.array(cfg.start_acc) if cfg.start_acc is not None else np.zeros(self.num_joints)
        self.end_acc = np.array(cfg.end_acc) if cfg.end_acc is not None else np.zeros(self.num_joints)
        self.start_jerk = np.array(cfg.start_jerk) if cfg.start_jerk is not None else np.zeros(self.num_joints)
        self.end_jerk = np.array(cfg.end_jerk) if cfg.end_jerk is not None else np.zeros(self.num_joints)

        # Pre-calculate coefficients based on type
        if self.type == "quintic":
            self.coeffs = self._calculate_quintic_coefficients()
        elif self.type == "septic":
            self.coeffs = self._calculate_septic_coefficients()
        else:
            raise ValueError(f"Unknown spline type: {self.type}")

    def _calculate_quintic_coefficients(self) -> np.ndarray:
        T = self.duration
        T2, T3, T4, T5 = T**2, T**3, T**4, T**5

        coeffs = np.zeros((self.num_joints, 6))
        for i in range(self.num_joints):
            q0, v0, acc0 = self.start_pos[i], self.start_vel[i], self.start_acc[i]
            q1, v1, acc1 = self.end_pos[i], self.end_vel[i], self.end_acc[i]

            a0, a1, a2 = q0, v0, acc0 / 2.0
            A = np.array([
                [T3, T4, T5],
                [3 * T2, 4 * T3, 5 * T4],
                [6 * T, 12 * T2, 20 * T3],
            ])
            B = np.array([
                q1 - (a0 + a1 * T + a2 * T2),
                v1 - (a1 + 2 * a2 * T),
                acc1 - (2 * a2),
            ])
            x = np.linalg.solve(A, B)
            coeffs[i] = [a0, a1, a2, x[0], x[1], x[2]]
        return coeffs

    def _calculate_septic_coefficients(self) -> np.ndarray:
        T = self.duration
        T2, T3, T4, T5, T6, T7 = T**2, T**3, T**4, T**5, T**6, T**7

        coeffs = np.zeros((self.num_joints, 8))
        for i in range(self.num_joints):
            q0, v0, acc0, j0 = self.start_pos[i], self.start_vel[i], self.start_acc[i], self.start_jerk[i]
            q1, v1, acc1, j1 = self.end_pos[i], self.end_vel[i], self.end_acc[i], self.end_jerk[i]

            a0, a1, a2, a3 = q0, v0, acc0 / 2.0, j0 / 6.0
            A = np.array([
                [T4, T5, T6, T7],
                [4 * T3, 5 * T4, 6 * T5, 7 * T6],
                [12 * T2, 20 * T3, 30 * T4, 42 * T5],
                [24 * T, 60 * T2, 120 * T3, 210 * T4],
            ])
            B = np.array([
                q1 - (a0 + a1 * T + a2 * T2 + a3 * T3),
                v1 - (a1 + 2 * a2 * T + 3 * a3 * T2),
                acc1 - (2 * a2 + 6 * a3 * T),
                j1 - (6 * a3),
            ])
            x = np.linalg.solve(A, B)
            coeffs[i] = [a0, a1, a2, a3, x[0], x[1], x[2], x[3]]
        return coeffs

    def _generate(self, *args, **kwargs):
        num_steps = len(self.time_array)
        pos = np.zeros((num_steps, self.num_joints))
        vel = np.zeros((num_steps, self.num_joints))
        acc = np.zeros((num_steps, self.num_joints))

        for t_idx, t in enumerate(self.time_array):
            t2, t3, t4, t5 = t**2, t**3, t**4, t**5
            for j in range(self.num_joints):
                if self.type == "quintic":
                    a0, a1, a2, a3, a4, a5 = self.coeffs[j]
                    pos[t_idx, j] = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5
                    vel[t_idx, j] = a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4
                    acc[t_idx, j] = 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3
                elif self.type == "septic":
                    a0, a1, a2, a3, a4, a5, a6, a7 = self.coeffs[j]
                    t6, t7 = t5 * t, t6 * t
                    pos[t_idx, j] = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5 + a6 * t6 + a7 * t7
                    vel[t_idx, j] = a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4 + 6 * a6 * t5 + 7 * a7 * t6
                    acc[t_idx, j] = 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3 + 30 * a6 * t4 + 42 * a7 * t5

        return pos, vel, acc
