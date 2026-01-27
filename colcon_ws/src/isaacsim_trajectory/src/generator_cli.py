import argparse
import numpy as np
import os
from isaacsim_trajectory import SplineTrajectory, SplineTrajectoryConfig
from isaacsim_trajectory import FourierTrajectory, FourierTrajectoryConfig
from isaacsim_trajectory import WindowTrajectory, WindowTrajectoryConfig

def main():
    parser = argparse.ArgumentParser(description='Generate trajectories for Isaac Sim.')
    parser.add_argument('--type', type=str, choices=['spline', 'fourier', 'window'], required=True, help='Type of trajectory')
    parser.add_argument('--duration', type=float, default=5.0, help='Duration in seconds')
    parser.add_argument('--fps', type=float, default=60.0, help='Frames per second')
    parser.add_argument('--output', type=str, default='trajectory.json', help='Output JSON path')
    parser.add_argument('--plot', type=str, default=None, help='Output plot path (e.g., plot.png)')
    parser.add_argument('--show-plot', action='store_true', help='Show the plot')

    # Spline specific
    parser.add_argument('--start', type=float, nargs=6, help='Start joint positions (6 values)')
    parser.add_argument('--end', type=float, nargs=6, help='End joint positions (6 values)')
    
    # Fourier specific
    parser.add_argument('--joints', type=int, default=6, help='Number of joints')
    parser.add_argument('--harmonics', type=int, default=5, help='Number of harmonics')
    parser.add_argument('--base-freq', type=float, default=0.1, help='Base frequency')

    args = parser.parse_args()

    # Ensure output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    if args.type == 'spline':
        if not args.start or not args.end:
            print("Error: --start and --end (6 values each) are required for spline trajectory")
            return
        cfg = SplineTrajectoryConfig(
            duration=args.duration,
            fps=args.fps,
            start_pos=args.start,
            end_pos=args.end,
            type="quintic"
        )
        traj = SplineTrajectory(cfg)
    
    elif args.type == 'fourier':
        cfg = FourierTrajectoryConfig(
            duration=args.duration,
            fps=args.fps,
            num_joints=args.joints,
            num_harmonics=args.harmonics,
            base_freq=args.base_freq
        )
        # Using random coefficients for demonstration if none provided
        traj = FourierTrajectory(cfg)
        # Initialize with some random coefficients for demo
        traj.a = np.random.uniform(-0.1, 0.1, (args.joints, args.harmonics))
        traj.b = np.random.uniform(-0.1, 0.1, (args.joints, args.harmonics))

    elif args.type == 'window':
        cfg = WindowTrajectoryConfig(
            duration=args.duration,
            fps=args.fps,
            num_joints=args.joints
        )
        traj = WindowTrajectory(cfg)

    print(f"Generating {args.type} trajectory...")
    traj.generate(show_plot=args.show_plot, plot_path=args.plot, json_path=args.output)
    print(f"Trajectory saved to {args.output}")

if __name__ == '__main__':
    main()
