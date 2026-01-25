import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

@dataclass
class TrajectoryConfig:
    """Base configuration for trajectories."""
    fps: int = 30
    duration: float = 5.0
    output_dir: str = "results"
    show_plot: bool = False
    plot_path: str | None = "plot.png"
    json_path: str | None = "trajectory.json"

class TrajectoryBase(ABC):
    """Base class for all trajectory generators."""

    def __init__(self, config: TrajectoryConfig):
        self.fps = config.fps
        self.duration = config.duration
        self.dt = 1.0 / self.fps
        self.num_points = int(self.duration * self.fps) + 1
        self.times = np.linspace(0, self.duration, self.num_points)
        self.config = config
        
        # Determine package root in the source workspace
        # This assumes the package is in 'colcon_ws/src/ur5e_trajectory'
        self.package_root = Path("/workspaces/isaac-sim-ur5e/colcon_ws/src/ur5e_trajectory")

    @abstractmethod
    def _generate(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Implementation-specific trajectory generation.
        To be implemented in each child class.
        Returns:
            Tuple of (position, velocity, acceleration) arrays.
        """
        pass

    def generate(self, **kwargs) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Public interface for trajectory generation.
        Handles calculations, JSON export, and plotting.
        """
        # Allow keyword overrides or fall back to config
        show_plot = kwargs.get("show_plot", self.config.show_plot)
        plot_path = kwargs.get("plot_path", self.config.plot_path)
        json_path = kwargs.get("json_path", self.config.json_path)

        # Implementation specific generation
        q, v, a = self._generate()

        # Handle outputs
        if json_path:
            self.write_to_json(q, v, a, json_path=json_path)

        if show_plot or plot_path:
            self.plot(q, v, a, show=show_plot, plot_path=plot_path)

        return q, v, a

    def get_times(self):
        """Returns the time array for the trajectory."""
        return self.times

    def write_to_json(self, q: np.ndarray, v: np.ndarray, a: np.ndarray, json_path: Optional[str] = None):
        """Save the trajectory to a JSON file."""
        json_path_str = json_path or self.config.json_path
        if json_path_str is None:
            return

        final_json_path = Path(json_path_str)
        if not final_json_path.is_absolute():
            final_json_path = self.package_root / self.config.output_dir / final_json_path

        frames = []
        for i in range(len(self.times)):
            # frame format: [pos_list, vel_list, acc_list]
            frame = [q[i].tolist(), v[i].tolist(), a[i].tolist()]
            frames.append(frame)

        data = {
            "duration": self.duration,
            "fps": float(self.fps),
            "frames": frames
        }

        final_json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(final_json_path, "w") as f:
            json.dump(data, f, indent=4)
        print(f"Trajectory JSON saved to {final_json_path}")

    def plot(self, q: np.ndarray, v: np.ndarray, a: np.ndarray, show: bool = False, plot_path: Optional[str] = None):
        """Plot the trajectory."""
        fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        
        self._plot_single_ax(axes[0], q, "Joint Positions", "Time [s]", "Position [rad]")
        self._plot_single_ax(axes[1], v, "Joint Velocities", "Time [s]", "Velocity [rad/s]")
        self._plot_single_ax(axes[2], a, "Joint Accelerations", "Time [s]", "Acceleration [rad/s^2]")
        
        plt.tight_layout()

        final_plot_path_str = plot_path or self.config.plot_path
        if final_plot_path_str:
            final_plot_path = Path(final_plot_path_str)
            if not final_plot_path.is_absolute():
                final_plot_path = self.package_root / self.config.output_dir / final_plot_path
            
            final_plot_path.parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(final_plot_path)
            print(f"Plot saved to {final_plot_path}")

        if show:
            plt.show()

    def _plot_single_ax(self, ax: Axes, data: np.ndarray, title: str, xlabel: str, ylabel: str):
        for j in range(data.shape[1]):
            ax.plot(self.times, data[:, j], label=f"Joint {j}")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small', ncol=2)
