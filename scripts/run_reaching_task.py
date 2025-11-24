#!/usr/bin/env python3
"""
Example script for humanoid reaching task with ZMP constraints.

This script demonstrates how to use the motion planner to plan
and execute a reaching task while maintaining balance.
"""

import numpy as np
import argparse
import yaml
from humanoid_planner import (
    MotionPlanner,
    ZMPConstraint,
    SupportPolygonCalculator,
    TrajectoryOptimizer
)


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Run humanoid reaching task')
    parser.add_argument('--config', type=str, default='config/planner_params.yaml',
                       help='Path to configuration file')
    parser.add_argument('--target', type=float, nargs=3, default=[0.5, 0.3, 1.2],
                       help='Target position (x y z)')
    parser.add_argument('--visualize', action='store_true',
                       help='Visualize trajectory and ZMP')
    return parser.parse_args()


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def main():
    """Main function."""
    args = parse_args()
    
    print("=" * 60)
    print("Humanoid Reaching Task with ZMP Constraints")
    print("=" * 60)
    
    # Load configuration
    print(f"\nLoading configuration from {args.config}...")
    try:
        config = load_config(args.config)
    except FileNotFoundError:
        print(f"Config file not found, using defaults")
        config = {
            'planner': {
                'zmp_margin': 0.02,
                'max_velocity': 1.0,
                'max_acceleration': 2.0,
                'planning_time': 5.0
            }
        }
    
    # Initialize components
    print("\nInitializing motion planner...")
    
    # ZMP constraint
    zmp_constraint = ZMPConstraint(margin=config['planner']['zmp_margin'])
    print(f"  ZMP margin: {config['planner']['zmp_margin']} m")
    
    # Support polygon calculator
    support_calc = SupportPolygonCalculator(foot_size=(0.2, 0.1))
    
    # Trajectory optimizer
    optimizer = TrajectoryOptimizer(
        num_joints=12,  # Example: 12 DOF robot
        dt=0.1
    )
    
    # Target pose
    target_pos = np.array(args.target)
    print(f"\nTarget position: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
    
    # Example: Define foot poses for double support
    left_foot_pose = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])  # x, y, z, roll, pitch, yaw
    right_foot_pose = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
    
    # Compute support polygon
    print("\nComputing support polygon...")
    support_polygon = support_calc.compute_double_support_polygon(
        left_foot_pose,
        right_foot_pose
    )
    print(f"  Support polygon area: {support_calc.compute_polygon_area(support_polygon):.4f} m²")
    
    # Plan trajectory
    print("\nPlanning trajectory...")
    
    # Starting configuration (all joints at zero)
    q_start = np.zeros(12)
    
    # Goal configuration (simplified - in practice would use IK)
    q_goal = np.array([0.1, 0.2, -0.4, 0.5, 0.3, -0.2,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Joint limits
    q_min = -np.ones(12) * np.pi
    q_max = np.ones(12) * np.pi
    
    # Optimize trajectory
    trajectory = optimizer.optimize_trajectory(
        q_start=q_start,
        q_goal=q_goal,
        num_waypoints=20,
        q_limits=(q_min, q_max),
        velocity_limit=config['planner']['max_velocity']
    )
    
    print(f"  Trajectory length: {len(trajectory)} waypoints")
    print(f"  Duration: {len(trajectory) * 0.1:.2f} seconds")
    
    # Verify ZMP constraints
    print("\nVerifying ZMP constraints...")
    
    # Simplified ZMP check (in practice would use full dynamics)
    num_violations = 0
    zmp_positions = []
    
    for i, q in enumerate(trajectory):
        # Estimate CoM position (simplified)
        com_pos = np.array([0.0, 0.0, 0.8])  # Example fixed CoM height
        com_acc = np.array([0.0, 0.0, 0.0])  # Simplified
        
        # Compute ZMP
        zmp = zmp_constraint.compute_zmp_from_com(com_pos, com_acc, height=0.8)
        zmp_positions.append(zmp)
        
        # Check stability
        if not zmp_constraint.is_stable(zmp, support_polygon):
            num_violations += 1
    
    print(f"  ZMP violations: {num_violations}/{len(trajectory)}")
    print(f"  Success rate: {100 * (1 - num_violations/len(trajectory)):.1f}%")
    
    # Compute trajectory statistics
    print("\nTrajectory statistics:")
    velocities = optimizer.compute_velocity_profile(trajectory)
    max_vel = np.max(np.abs(velocities))
    print(f"  Maximum velocity: {max_vel:.3f} rad/s")
    
    if len(trajectory) >= 3:
        accelerations = optimizer.compute_acceleration_profile(trajectory)
        max_acc = np.max(np.abs(accelerations))
        print(f"  Maximum acceleration: {max_acc:.3f} rad/s²")
    
    # Visualization
    if args.visualize:
        print("\nGenerating visualization...")
        try:
            import matplotlib.pyplot as plt
            
            # Plot support polygon and ZMP trajectory
            zmp_positions = np.array(zmp_positions)
            support_calc.visualize_polygon(support_polygon, zmp_positions[0])
            
            # Plot joint trajectories
            fig, axes = plt.subplots(3, 1, figsize=(10, 8))
            
            time = np.arange(len(trajectory)) * 0.1
            
            # Position
            axes[0].plot(time, trajectory)
            axes[0].set_ylabel('Position (rad)')
            axes[0].set_title('Joint Trajectories')
            axes[0].grid(True)
            axes[0].legend([f'Joint {i+1}' for i in range(min(6, 12))], 
                          loc='right', fontsize=8)
            
            # Velocity
            time_vel = np.arange(len(velocities)) * 0.1
            axes[1].plot(time_vel, velocities)
            axes[1].set_ylabel('Velocity (rad/s)')
            axes[1].axhline(y=config['planner']['max_velocity'], 
                           color='r', linestyle='--', label='Limit')
            axes[1].axhline(y=-config['planner']['max_velocity'], 
                           color='r', linestyle='--')
            axes[1].grid(True)
            axes[1].legend()
            
            # Acceleration
            if len(trajectory) >= 3:
                time_acc = np.arange(len(accelerations)) * 0.1
                axes[2].plot(time_acc, accelerations)
                axes[2].set_ylabel('Acceleration (rad/s²)')
                axes[2].set_xlabel('Time (s)')
                axes[2].axhline(y=config['planner']['max_acceleration'], 
                               color='r', linestyle='--', label='Limit')
                axes[2].axhline(y=-config['planner']['max_acceleration'], 
                               color='r', linestyle='--')
                axes[2].grid(True)
                axes[2].legend()
            
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            print("  Matplotlib not available, skipping visualization")
    
    print("\n" + "=" * 60)
    print("Task completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
