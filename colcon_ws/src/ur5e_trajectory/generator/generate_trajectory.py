import tyro
import numpy as np
from .factory import create_trajectory, TrajectoryConfigUnion

def main():
    # tyro will automatically generate a CLI based on the Union type
    config = tyro.cli(TrajectoryConfigUnion)
    
    generator = create_trajectory(config)
    
    # The new generate() handles plotting and JSON export internally
    # based on the config parameters (show_plot, plot_path, json_path).
    q, v, a = generator.generate()
    times = generator.get_times()
    
    print(f"Generated trajectory with {len(times)} points.")
    print(f"Shape: {q.shape}")
    
    # Print first few points as example
    print("\nFirst 5 points (Joint 0):")
    for i in range(min(5, len(times))):
        print(f"t={times[i]:.3f}: pos={q[i, 0]:.4f}, vel={v[i, 0]:.4f}, acc={a[i, 0]:.4f}")

if __name__ == "__main__":
    main()
