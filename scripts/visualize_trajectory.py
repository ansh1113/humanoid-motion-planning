#!/usr/bin/env python3
"""
Trajectory Visualization Script

This script provides comprehensive visualization of planned trajectories,
including robot motion, ZMP trajectory, joint angle profiles, and stability metrics.
"""

import argparse
import pickle
import sys
from pathlib import Path
from typing import Dict, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Visualize humanoid robot trajectories",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--trajectory',
        type=str,
        required=True,
        help='Path to trajectory file (.pkl)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Save visualization to file (e.g., output.png)'
    )
    
    parser.add_argument(
        '--show-zmp',
        action='store_true',
        help='Show ZMP trajectory plot'
    )
    
    parser.add_argument(
        '--show-joints',
        action='store_true',
        help='Show joint angle profiles'
    )
    
    parser.add_argument(
        '--show-velocity',
        action='store_true',
        help='Show joint velocity profiles'
    )
    
    parser.add_argument(
        '--dpi',
        type=int,
        default=150,
        help='DPI for output figure'
    )
    
    return parser.parse_args()


def load_trajectory(filepath: str) -> Optional[Dict]:
    """
    Load trajectory from pickle file.
    
    Args:
        filepath: Path to trajectory file
        
    Returns:
        Trajectory dictionary or None if loading fails
    """
    try:
        with open(filepath, 'rb') as f:
            trajectory = pickle.load(f)
        print(f"✓ Loaded trajectory from {filepath}")
        return trajectory
    except FileNotFoundError:
        print(f"✗ Error: Trajectory file not found: {filepath}")
        return None
    except Exception as e:
        print(f"✗ Error loading trajectory: {e}")
        return None


def print_trajectory_info(trajectory: Dict):
    """Print trajectory information."""
    print("\n" + "="*60)
    print("Trajectory Information")
    print("="*60)
    
    if 'waypoints' in trajectory:
        print(f"Number of waypoints: {len(trajectory['waypoints'])}")
    
    if 'duration' in trajectory:
        print(f"Duration: {trajectory['duration']:.2f} seconds")
    
    if 'timestamps' in trajectory:
        dt = np.diff(trajectory['timestamps'])
        print(f"Average timestep: {np.mean(dt):.3f} seconds")
        print(f"Min timestep: {np.min(dt):.3f} seconds")
        print(f"Max timestep: {np.max(dt):.3f} seconds")
    
    if 'joint_angles' in trajectory:
        print(f"Number of joints: {len(trajectory['joint_angles'])}")
        print(f"Joint names: {', '.join(trajectory['joint_angles'].keys())}")
    
    if 'success' in trajectory:
        status = "✓ SUCCESS" if trajectory['success'] else "✗ FAILED"
        print(f"Planning status: {status}")
    
    print("="*60 + "\n")


def plot_zmp_trajectory(ax, trajectory: Dict):
    """
    Plot ZMP trajectory on support polygon.
    
    Args:
        ax: Matplotlib axis
        trajectory: Trajectory dictionary
    """
    if 'zmp_positions' not in trajectory:
        ax.text(0.5, 0.5, 'No ZMP data available', 
                ha='center', va='center', transform=ax.transAxes)
        return
    
    zmp_positions = np.array(trajectory['zmp_positions'])
    
    # Plot ZMP trajectory
    ax.plot(zmp_positions[:, 0], zmp_positions[:, 1], 
            'b-', linewidth=2, label='ZMP Trajectory', alpha=0.7)
    
    # Mark start and end
    ax.scatter(zmp_positions[0, 0], zmp_positions[0, 1], 
              c='green', s=150, marker='o', label='Start', zorder=5, edgecolors='black')
    ax.scatter(zmp_positions[-1, 0], zmp_positions[-1, 1], 
              c='red', s=150, marker='s', label='End', zorder=5, edgecolors='black')
    
    # Plot support polygon (simple rectangle as example)
    # In practice, this would come from actual foot contacts
    support_polygon = np.array([
        [0.15, 0.08],
        [0.15, -0.08],
        [-0.15, -0.08],
        [-0.15, 0.08],
        [0.15, 0.08]
    ])
    
    ax.plot(support_polygon[:, 0], support_polygon[:, 1], 
            'r--', linewidth=2, label='Support Polygon', alpha=0.5)
    ax.fill(support_polygon[:, 0], support_polygon[:, 1], 
            'red', alpha=0.1)
    
    ax.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
    ax.set_title('ZMP Trajectory', fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.axis('equal')
    
    # Add stability indicator
    zmp_in_polygon = np.all(np.abs(zmp_positions) < [0.15, 0.08], axis=1)
    stability_pct = 100 * np.sum(zmp_in_polygon) / len(zmp_in_polygon)
    color = 'green' if stability_pct > 95 else 'orange' if stability_pct > 80 else 'red'
    ax.text(0.02, 0.98, f'Stability: {stability_pct:.1f}%', 
            transform=ax.transAxes, fontsize=10, fontweight='bold',
            verticalalignment='top', bbox=dict(boxstyle='round', 
            facecolor=color, alpha=0.3))


def plot_joint_angles(ax, trajectory: Dict):
    """
    Plot joint angle profiles over time.
    
    Args:
        ax: Matplotlib axis
        trajectory: Trajectory dictionary
    """
    if 'joint_angles' not in trajectory or 'timestamps' not in trajectory:
        ax.text(0.5, 0.5, 'No joint angle data available', 
                ha='center', va='center', transform=ax.transAxes)
        return
    
    timestamps = trajectory['timestamps']
    joint_angles = trajectory['joint_angles']
    
    # Plot each joint
    colors = plt.cm.tab10(np.linspace(0, 1, len(joint_angles)))
    for i, (joint_name, angles) in enumerate(joint_angles.items()):
        ax.plot(timestamps, np.rad2deg(angles), 
                label=joint_name, linewidth=2, alpha=0.8, color=colors[i])
    
    ax.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Joint Angle (degrees)', fontsize=11, fontweight='bold')
    ax.set_title('Joint Angle Profiles', fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3, linestyle='--')


def plot_joint_velocities(ax, trajectory: Dict):
    """
    Plot joint velocity profiles over time.
    
    Args:
        ax: Matplotlib axis
        trajectory: Trajectory dictionary
    """
    if 'joint_angles' not in trajectory or 'timestamps' not in trajectory:
        ax.text(0.5, 0.5, 'No joint angle data available', 
                ha='center', va='center', transform=ax.transAxes)
        return
    
    timestamps = trajectory['timestamps']
    joint_angles = trajectory['joint_angles']
    
    # Compute velocities using finite differences
    dt = np.diff(timestamps)
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(joint_angles)))
    for i, (joint_name, angles) in enumerate(joint_angles.items()):
        velocities = np.diff(angles) / dt
        ax.plot(timestamps[:-1], np.rad2deg(velocities), 
                label=joint_name, linewidth=2, alpha=0.8, color=colors[i])
    
    # Add velocity limits (typical humanoid joint velocity limit)
    max_vel = 90  # degrees per second
    ax.axhline(y=max_vel, color='r', linestyle='--', alpha=0.5, linewidth=1.5)
    ax.axhline(y=-max_vel, color='r', linestyle='--', alpha=0.5, linewidth=1.5)
    ax.text(timestamps[-1], max_vel, ' Limit', va='center', color='red', fontweight='bold')
    
    ax.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Joint Velocity (deg/s)', fontsize=11, fontweight='bold')
    ax.set_title('Joint Velocity Profiles', fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3, linestyle='--')


def plot_trajectory_3d(ax, trajectory: Dict):
    """
    Plot 3D trajectory of end-effector.
    
    Args:
        ax: Matplotlib 3D axis
        trajectory: Trajectory dictionary
    """
    # For now, plot a placeholder
    # In a full implementation, this would compute FK for each waypoint
    ax.text(0.5, 0.5, 0.5, 'End-effector 3D trajectory\n(Requires FK computation)', 
            ha='center', va='center')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('End-Effector Trajectory (3D)', fontsize=13, fontweight='bold')


def visualize_trajectory(trajectory: Dict, args):
    """
    Create comprehensive trajectory visualization.
    
    Args:
        trajectory: Trajectory dictionary
        args: Command line arguments
    """
    # Determine number of subplots - default to showing ZMP and joints if none specified
    if not args.show_zmp and not args.show_joints and not args.show_velocity:
        print("Note: No plots explicitly selected. Showing ZMP and joints by default.")
        args.show_zmp = True
        args.show_joints = True
    
    n_plots = sum([args.show_zmp, args.show_joints, args.show_velocity])
    
    # Create figure with GridSpec for flexible layout
    fig = plt.figure(figsize=(15, 5 * ((n_plots + 1) // 2)))
    fig.suptitle('Humanoid Robot Trajectory Visualization', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.3)
    
    plot_idx = 0
    
    # Plot ZMP trajectory
    if args.show_zmp:
        ax = fig.add_subplot(gs[plot_idx // 2, plot_idx % 2])
        plot_zmp_trajectory(ax, trajectory)
        plot_idx += 1
    
    # Plot joint angles
    if args.show_joints:
        ax = fig.add_subplot(gs[plot_idx // 2, plot_idx % 2])
        plot_joint_angles(ax, trajectory)
        plot_idx += 1
    
    # Plot joint velocities
    if args.show_velocity:
        ax = fig.add_subplot(gs[plot_idx // 2, plot_idx % 2])
        plot_joint_velocities(ax, trajectory)
        plot_idx += 1
    
    # Add trajectory metadata as text
    if plot_idx < 4:
        ax = fig.add_subplot(gs[plot_idx // 2, plot_idx % 2])
        ax.axis('off')
        
        info_text = "Trajectory Metadata\n" + "="*30 + "\n\n"
        if 'waypoints' in trajectory:
            info_text += f"Waypoints: {len(trajectory['waypoints'])}\n"
        if 'duration' in trajectory:
            info_text += f"Duration: {trajectory['duration']:.2f} s\n"
        if 'success' in trajectory:
            status = "SUCCESS ✓" if trajectory['success'] else "FAILED ✗"
            info_text += f"Status: {status}\n"
        if 'timestamps' in trajectory:
            dt = np.diff(trajectory['timestamps'])
            info_text += f"\nTiming Statistics:\n"
            info_text += f"  Avg timestep: {np.mean(dt):.3f} s\n"
            info_text += f"  Min timestep: {np.min(dt):.3f} s\n"
            info_text += f"  Max timestep: {np.max(dt):.3f} s\n"
        
        ax.text(0.1, 0.9, info_text, transform=ax.transAxes, 
                fontsize=11, verticalalignment='top', 
                fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    # Save or show
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(args.output, dpi=args.dpi, bbox_inches='tight')
        print(f"✓ Saved visualization to {args.output}")
    else:
        plt.show()


def main():
    """Main function."""
    args = parse_args()
    
    print("="*60)
    print("Humanoid Robot Trajectory Visualization")
    print("="*60)
    
    # Load trajectory
    trajectory = load_trajectory(args.trajectory)
    if trajectory is None:
        sys.exit(1)
    
    # Print trajectory information
    print_trajectory_info(trajectory)
    
    # Visualize
    try:
        visualize_trajectory(trajectory, args)
    except Exception as e:
        print(f"✗ Error during visualization: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print("\n✓ Visualization complete!")


if __name__ == '__main__':
    main()
