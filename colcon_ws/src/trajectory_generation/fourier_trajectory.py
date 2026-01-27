from dataclasses import dataclass, field
from pathlib import Path
import numpy as np

try:
    from omegaconf import MISSING
except ImportError:
    MISSING = "???"

from .trajectory_base import BaseTrajectory, BaseTrajectoryConfig

@dataclass
class FourierTrajectoryConfig(BaseTrajectoryConfig):
    config: Path | None = None  # Path to YAML configuration file
    num_joints: int = MISSING
    num_harmonics: int = MISSING
    base_freq: float = MISSING
    coefficients: dict | None = field(default_factory=lambda: MISSING)
    q0: list[float] | None = field(default_factory=lambda: MISSING)
    target_class: str = "FourierTrajectory"


class FourierTrajectory(BaseTrajectory):
    """
    Finite Fourier Series Trajectory.
    q_i(t) = q_{i,0} + sum_{k=1}^{N} ( a_{i,k} sin(2*pi*k*f*t) + b_{i,k} cos(2*pi*k*f*t) )
    """

    def __init__(self, cfg: FourierTrajectoryConfig, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        self.num_joints = cfg.num_joints
        self.num_harmonics = cfg.num_harmonics
        self.base_freq = cfg.base_freq
        self.omega_b = 2 * np.pi * cfg.base_freq

        if cfg.coefficients is None or cfg.coefficients == MISSING or isinstance(cfg.coefficients, str):
            self.a = np.zeros((self.num_joints, self.num_harmonics))
            self.b = np.zeros((self.num_joints, self.num_harmonics))
            self.q0 = np.zeros(self.num_joints)
        else:
            self.a = np.array(cfg.coefficients.get("a", np.zeros((self.num_joints, self.num_harmonics))))
            self.b = np.array(cfg.coefficients.get("b", np.zeros((self.num_joints, self.num_harmonics))))
            if "q0" in cfg.coefficients:
                self.q0 = np.array(cfg.coefficients["q0"])
            elif cfg.q0 is not None and cfg.q0 != MISSING:
                self.q0 = np.array(cfg.q0)
            else:
                self.q0 = np.zeros(self.num_joints)

    def get_value(self):
        """
        Calculate q, dq, ddq at time t.
        Returns:
            q (N, num_joints), dq (N, num_joints), ddq (N, num_joints)
        """
        t_arr = np.array(self.time_array)
        num_steps = len(t_arr)
        
        q = np.tile(self.q0, (num_steps, 1))
        dq = np.zeros((num_steps, self.num_joints))
        ddq = np.zeros((num_steps, self.num_joints))

        for k in range(1, self.num_harmonics + 1):
            omega_k = k * self.omega_b

            idx = k - 1
            a_k = self.a[:, idx].reshape(1, -1)  # (1, num_joints)
            b_k = self.b[:, idx].reshape(1, -1)  # (1, num_joints)

            wkt = (omega_k * t_arr).reshape(-1, 1) # (N, 1)
            sin_wkt = np.sin(wkt)
            cos_wkt = np.cos(wkt)

            # Position: q += a*sin + b*cos
            q += a_k * sin_wkt + b_k * cos_wkt

            # Velocity: dq/dt = a*w*cos - b*w*sin
            dq += omega_k * (a_k * cos_wkt - b_k * sin_wkt)

            # Acceleration: d2q/dt2 = -w^2 * (a*sin + b*cos)
            ddq += -(omega_k**2) * (a_k * sin_wkt + b_k * cos_wkt)

        return q, dq, ddq

    def _generate(self, *args, **kwargs):
        # remove last element if it exceeds duration significantly
        if self.time_array[-1] > self.duration + 1e-9:
            self.time_array = self.time_array[:-1]

        pos, vel, acc = self.get_value()
        return pos, vel, acc
