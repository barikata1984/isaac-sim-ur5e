from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np
from .base import TrajectoryBase, TrajectoryConfig

@dataclass
class FourierConfig(TrajectoryConfig):
    """Configuration for Fourier-series based trajectories."""
    num_harmonics: int = 5
    base_frequency: float = 0.2  # rad/s
    coefficients: Optional[List[List[float]]] = None

    def __post_init__(self):
        if self.coefficients is None:
            num_dof = 6
            size = 2 * self.num_harmonics + 1
            self.coefficients = (np.random.rand(num_dof, size) * 0.1).tolist()

class FourierTrajectory(TrajectoryBase):
    """Fourier-series based trajectory generator."""

    def __init__(self, config: FourierConfig):
        super().__init__(config)
        self.coeffs = np.array(config.coefficients)

    def _generate(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = self.times
        num_dof = self.coeffs.shape[0]
        q = np.zeros((self.num_points, num_dof))
        v = np.zeros((self.num_points, num_dof))
        a = np.zeros((self.num_points, num_dof))
        
        w = self.config.base_frequency
        
        for i in range(num_dof):
            c = self.coeffs[i]
            a0 = c[0]
            # q(t) = a0 + sum_{n=1}^N (an cos(nwt) + bn sin(nwt))
            # v(t) = sum_{n=1}^N (-an*nw sin(nwt) + bn*nw cos(nwt))
            # a(t) = sum_{n=1}^N (-an*(nw)^2 cos(nwt) - bn*(nw)^2 sin(nwt))
            q_val = np.full_like(t, a0)
            v_val = np.zeros_like(t)
            a_val = np.zeros_like(t)
            
            for n in range(1, self.config.num_harmonics + 1):
                an = c[2*n - 1]
                bn = c[2*n]
                nw = n * w
                
                cos_nwt = np.cos(nw * t)
                sin_nwt = np.sin(nw * t)
                
                q_val += an * cos_nwt + bn * sin_nwt
                v_val += -an * nw * sin_nwt + bn * nw * cos_nwt
                a_val += -an * (nw**2) * cos_nwt - bn * (nw**2) * sin_nwt
                
            q[:, i] = q_val
            v[:, i] = v_val
            a[:, i] = a_val
            
        return q, v, a
