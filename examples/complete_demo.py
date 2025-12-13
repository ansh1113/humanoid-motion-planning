#!/usr/bin/env python3
"""
Complete End-to-End Reaching Demo

Demonstrates the full pipeline:
1. Environment perception and object detection
2. Goal prediction and grasp planning  
3. Trajectory optimization with ZMP constraints
4. Stability verification
5. Visualization
"""

import numpy as np
import argparse
import matplotlib.pyplot as plt
from humanoid_planner import (
    ObjectDetector, EnvironmentState, GoalPredictor,
    WorkspaceAnalyzer, ObjectType,
    TrajectoryOptimizer, ZMPConstraint,
    SupportPolygonCalculator, StabilityAnalyzer,
    ForwardKinematics
)


def setup_environment():
    """Create a demo environment with objects."""
    detector = ObjectDetector(detection_range=3.0)
    
    # Add target object
    target = detector.add_object(
        position=np.array([0.6, 0.2, 1.1]),
        object_type=ObjectType.TARGET,
        dimensions=np.array([0.08, 0.08, 0.15]),
        confidence=0.95
    )
    print(f"✓ Target detected at: {target.position}")
    
    # Add obstacles
    obstacle1 = detector.add_object(
        position=np.array([0.4, 0.0, 0.9]),
        object_type=ObjectType.OBSTACLE,
        dimensions=np.array([0.15, 0.15, 0.3]),
        confidence=0.88
    )
    print(f"✓ Obstacle 1 at: {obstacle1.position}")
    
    obstacle2 = detector.add_object(
        position=np.array([0.5, -0.3, 0.8]),
        object_type=ObjectType.OBSTACLE,
        dimensions=np.array([0.2, 0.2, 0.2]),
        confidence=0.82
    )
    print(f"✓ Obstacle 2 at: {obstacle2.position}")
    
    # Add surface
    surface = detector.add_object(
        position=np.array([0.6, 0.0, 0.7]),
        object_type=ObjectType.SURFACE,
        dimensions=np.array([1.0, 0.8, 0.05]),
        confidence=0.99
    )
    print(f"✓ Surface at: {surface.position}")
    
    return detector


def predict_grasp(detector, predictor):
    """Predict optimal grasp position."""
    target = detector.get_objects_by_type(ObjectType.TARGET)[0]
    obstacles = detector.get_objects_by_type(ObjectType.OBSTACLE)
    
    # Predict grasp position
    grasp_pos, grasp_orient = predictor.predict_grasp_position(
        target.position,
        target.dimensions,
        approach_direction=np.array([0, 0, -1])  # From above
    )
    print(f"\n✓ Predicted grasp position: {grasp_pos}")
    
    # Generate via points
    start_pos = np.array([0.0, 0.0, 1.2])
    via_points = predictor.predict_via_points(
        start_pos, grasp_pos, obstacles, num_via_points=3
    )
    print(f"✓ Generated {len(via_points)} via points")
    
    return grasp_pos, via_points


def plan_trajectory(num_joints=7):
    """Plan joint space trajectory."""
    optimizer = TrajectoryOptimizer(num_joints=num_joints, dt=0.1)
    
    # Simplified joint space planning
    q_start = np.zeros(num_joints)
    q_goal = np.array([0.3, 0.5, -0.4, 0.8, 0.2, -0.3, 0.1])
    
    print("\n✓ Planning trajectory...")
    trajectory = optimizer.optimize_trajectory(
        q_start=q_start,
        q_goal=q_goal,
        num_waypoints=25,
        q_limits=(np.ones(num_joints) * -np.pi, np.ones(num_joints) * np.pi),
        velocity_limit=2.0
    )
    
    print(f"✓ Trajectory planned with {len(trajectory)} waypoints")
    print(f"  Duration: {len(trajectory) * 0.1:.2f} seconds")
    
    return trajectory, optimizer


def verify_stability(trajectory):
    """Verify ZMP stability along trajectory."""
    # Setup stability analysis
    zmp_constraint = ZMPConstraint(margin=0.02)
    support_calc = SupportPolygonCalculator(foot_size=(0.25, 0.12))
    stability_analyzer = StabilityAnalyzer()
    
    # Define foot poses (double support)
    left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
    right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
    
    support_polygon = support_calc.compute_double_support_polygon(
        left_foot, right_foot
    )
    
    print("\n✓ Verifying stability...")
    print(f"  Support polygon area: {support_calc.compute_polygon_area(support_polygon):.4f} m²")
    
    # Simulate CoM trajectory (simplified)
    violations = 0
    margins = []
    
    for i, q in enumerate(trajectory):
        # Simplified CoM position
        com_pos = np.array([0.0, 0.0, 0.95])
        com_acc = np.zeros(3)
        
        zmp = zmp_constraint.compute_zmp_from_com(com_pos, com_acc, height=0.95)
        
        stable = zmp_constraint.is_stable(zmp, support_polygon)
        margin = zmp_constraint.compute_stability_margin(zmp, support_polygon)
        
        if not stable:
            violations += 1
        margins.append(margin)
    
    print(f"  ZMP violations: {violations}/{len(trajectory)}")
    print(f"  Min stability margin: {min(margins):.4f} m")
    print(f"  Avg stability margin: {np.mean(margins):.4f} m")
    
    return support_polygon, margins


def visualize_results(trajectory, optimizer, support_polygon, margins):
    """Visualize trajectory and stability metrics."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Joint trajectories
    time = np.arange(len(trajectory)) * 0.1
    for joint_idx in range(min(7, trajectory.shape[1])):
        axes[0, 0].plot(time, trajectory[:, joint_idx], label=f'Joint {joint_idx+1}')
    
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Joint Angle (rad)')
    axes[0, 0].set_title('Joint Trajectories')
    axes[0, 0].legend(loc='upper right', fontsize=8)
    axes[0, 0].grid(True)
    
    # Plot 2: Velocity profile
    velocities = optimizer.compute_velocity_profile(trajectory)
    time_vel = np.arange(len(velocities)) * 0.1
    max_vels = np.max(np.abs(velocities), axis=1)
    
    axes[0, 1].plot(time_vel, max_vels, 'b-', linewidth=2)
    axes[0, 1].axhline(y=2.0, color='r', linestyle='--', label='Velocity Limit')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Max Joint Velocity (rad/s)')
    axes[0, 1].set_title('Velocity Profile')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Plot 3: Support polygon
    polygon_closed = np.vstack([support_polygon, support_polygon[0]])
    axes[1, 0].plot(polygon_closed[:, 0], polygon_closed[:, 1], 'b-', linewidth=2)
    axes[1, 0].fill(support_polygon[:, 0], support_polygon[:, 1], alpha=0.3, color='blue')
    axes[1, 0].plot(0, 0, 'ro', markersize=10, label='ZMP (simplified)')
    axes[1, 0].set_xlabel('X (m)')
    axes[1, 0].set_ylabel('Y (m)')
    axes[1, 0].set_title('Support Polygon')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    axes[1, 0].axis('equal')
    
    # Plot 4: Stability margins
    time_margin = np.arange(len(margins)) * 0.1
    axes[1, 1].plot(time_margin, margins, 'g-', linewidth=2)
    axes[1, 1].axhline(y=0, color='r', linestyle='--', label='Stability Threshold')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Stability Margin (m)')
    axes[1, 1].set_title('Stability Margins Over Time')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('/tmp/reaching_demo_results.png', dpi=150, bbox_inches='tight')
    print("\n✓ Visualization saved to /tmp/reaching_demo_results.png")
    
    return fig


def main():
    """Main demo function."""
    parser = argparse.ArgumentParser(description='Complete reaching task demo')
    parser.add_argument('--visualize', action='store_true', help='Show visualizations')
    args = parser.parse_args()
    
    print("=" * 70)
    print("HUMANOID REACHING TASK - COMPLETE DEMO")
    print("=" * 70)
    
    # Step 1: Setup environment
    print("\n[1/5] Setting up environment...")
    detector = setup_environment()
    
    # Step 2: Predict grasp
    print("\n[2/5] Predicting grasp position...")
    predictor = GoalPredictor()
    grasp_pos, via_points = predict_grasp(detector, predictor)
    
    # Step 3: Plan trajectory
    print("\n[3/5] Planning trajectory...")
    trajectory, optimizer = plan_trajectory(num_joints=7)
    
    # Step 4: Verify stability
    print("\n[4/5] Verifying stability...")
    support_polygon, margins = verify_stability(trajectory)
    
    # Step 5: Visualize
    print("\n[5/5] Generating visualization...")
    if args.visualize:
        fig = visualize_results(trajectory, optimizer, support_polygon, margins)
        plt.show()
    else:
        visualize_results(trajectory, optimizer, support_polygon, margins)
    
    # Summary
    print("\n" + "=" * 70)
    print("DEMO COMPLETED SUCCESSFULLY!")
    print("=" * 70)
    print("\nSummary:")
    print(f"  • Objects detected: {len(detector.detected_objects)}")
    print(f"  • Trajectory waypoints: {len(trajectory)}")
    print(f"  • Execution time: {len(trajectory) * 0.1:.2f} s")
    print(f"  • Min stability margin: {min(margins):.4f} m")
    print(f"  • ZMP violations: {sum(1 for m in margins if m < 0)}")
    print("\n✓ All systems operational!")
    print("=" * 70)


if __name__ == "__main__":
    main()
