import argparse
import sys
from pathlib import Path
from trajectory_generation import (
    FourierTrajectory, FourierTrajectoryConfig,
    SplineTrajectory, SplineTrajectoryConfig
)

def main():
    parser = argparse.ArgumentParser(description="Trajectory Generation CLI")
    parser.add_argument("--type", choices=["fourier", "spline"], required=True, help="Type of trajectory")
    parser.add_argument("--duration", type=float, default=2.0, help="Duration in seconds")
    parser.add_argument("--fps", type=float, default=30.0, help="Frames per second")
    parser.add_argument("--show-plot", action="store_true", help="Show the plot")
    parser.add_argument("--plot-path", type=str, help="Path to save the plot image")
    parser.add_argument("--json-path", type=str, help="Path to save the trajectory JSON")
    
    # Simple defaults for demo
    args = parser.parse_args()

    if args.type == "fourier":
        cfg = FourierTrajectoryConfig(
            duration=args.duration,
            fps=args.fps,
            num_joints=2,
            num_harmonics=2,
            base_freq=0.5,
            coefficients={
                "a": [[1.0, 0.5], [0.5, 0.2]],
                "b": [[0.1, 0.0], [0.1, 0.0]],
                "q0": [0.0, 0.0]
            }
        )
        traj = FourierTrajectory(cfg)
    else: # spline
        cfg = SplineTrajectoryConfig(
            duration=args.duration,
            fps=args.fps,
            type="quintic",
            start_pos=[0.0, 0.0],
            end_pos=[1.0, 2.0]
        )
        traj = SplineTrajectory(cfg)

    print(f"Generating {args.type} trajectory...")
    traj.generate(show_plot=args.show_plot, plot_path=args.plot_path, json_path=args.json_path)

if __name__ == "__main__":
    main()
