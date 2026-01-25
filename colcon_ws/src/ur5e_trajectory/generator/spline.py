from dataclasses import dataclass
from typing import Literal, List, Optional, Tuple
import numpy as np
from .base import TrajectoryBase, TrajectoryConfig

@dataclass
class SplineConfig(TrajectoryConfig):
    """Configuration for spline-based trajectories."""
    type: Literal["5th", "7th"] = "5th"
    start_pos: Optional[List[float]] = None
    end_pos: Optional[List[float]] = None
    # Boundary conditions: velocity, acceleration, jerk (for 7th)
    start_vel: Optional[List[float]] = None
    end_vel: Optional[List[float]] = None
    start_acc: Optional[List[float]] = None
    end_acc: Optional[List[float]] = None
    start_jerk: Optional[List[float]] = None
    end_jerk: Optional[List[float]] = None

    def __post_init__(self):
        # Default to 6-DOF (UR5e) if not provided
        if self.start_pos is None: self.start_pos = [0.0] * 6
        if self.end_pos is None: self.end_pos = [1.0] * 6
        if self.start_vel is None: self.start_vel = [0.0] * 6
        if self.end_vel is None: self.end_vel = [0.0] * 6
        if self.start_acc is None: self.start_acc = [0.0] * 6
        if self.end_acc is None: self.end_acc = [0.0] * 6
        if self.start_jerk is None: self.start_jerk = [0.0] * 6
        if self.end_jerk is None: self.end_jerk = [0.0] * 6

class SplineTrajectory(TrajectoryBase):
    """Spline-based trajectory generator supporting 5th and 7th order polynomials."""

    def __init__(self, config: SplineConfig):
        super().__init__(config)
        self.coeffs = self._calculate_coefficients()

    def _calculate_coefficients(self):
        T = self.duration
        coeffs = []
        
        for i in range(len(self.config.start_pos)):
            q0, qf = self.config.start_pos[i], self.config.end_pos[i]
            v0, vf = self.config.start_vel[i], self.config.end_vel[i]
            a0, af = self.config.start_acc[i], self.config.end_acc[i]
            
            if self.config.type == "5th":
                A = np.array([
                    [T**3, T**4, T**5],
                    [3*T**2, 4*T**3, 5*T**4],
                    [6*T, 12*T**2, 20*T**3]
                ])
                B = np.array([
                    [qf - q0 - v0*T - 0.5*a0*T**2],
                    [vf - v0 - a0*T],
                    [af - a0]
                ])
                X = np.linalg.solve(A, B)
                coeffs.append([q0, v0, 0.5*a0, X[0,0], X[1,0], X[2,0]])
                
            elif self.config.type == "7th":
                j0, jf = self.config.start_jerk[i], self.config.end_jerk[i]
                A = np.array([
                    [T**4, T**5, T**6, T**7],
                    [4*T**3, 5*T**4, 6*T**5, 7*T**6],
                    [12*T**2, 20*T**3, 30*T**4, 42*T**5],
                    [24*T, 60*T**2, 120*T**3, 210*T**4]
                ])
                B = np.array([
                    [qf - q0 - v0*T - 0.5*a0*T**2 - (1/6)*j0*T**3],
                    [vf - v0 - a0*T - 0.5*j0*T**2],
                    [af - a0 - j0*T],
                    [jf - j0]
                ])
                X = np.linalg.solve(A, B)
                coeffs.append([q0, v0, 0.5*a0, (1/6)*j0, X[0,0], X[1,0], X[2,0], X[3,0]])
                
        return np.array(coeffs)

    def _generate(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = self.times
        num_dof = self.coeffs.shape[0]
        q = np.zeros((self.num_points, num_dof))
        v = np.zeros((self.num_points, num_dof))
        a = np.zeros((self.num_points, num_dof))
        
        for i in range(num_dof):
            c = self.coeffs[i]
            # Position: q(t) = sum(c_j * t^j)
            # Velocity: v(t) = sum(j * c_j * t^(j-1))
            # Acceleration: a(t) = sum(j * (j-1) * c_j * t^(j-2))
            for j in range(len(c)):
                q[:, i] += c[j] * (t**j)
                if j > 0:
                    v[:, i] += j * c[j] * (t**(j-1))
                if j > 1:
                    a[:, i] += j * (j-1) * c[j] * (t**(j-2))
            
        return q, v, a
