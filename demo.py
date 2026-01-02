#!/usr/bin/env python3
"""
Humanoid Motion Planning Demo

Complete demonstration of ZMP-constrained whole-body motion planning
for the Unitree G1 humanoid robot.

Usage:
    python demo.py                    # Run full demo with visualization
    python demo.py --no-viz           # Run without visualization
    python demo.py --interactive      # Interactive mode
"""

import sys
import numpy as np
import time

# Add src to path
sys.path.insert(0, 'src')

from g1_model import G1Model
from zmp_calculator import ZMPCalculator
from inverse_kinematics import InverseKinematics
from motion_planner import WholeBodyPlanner, visualize_plan


def print_header(title):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def run_demo(visualize=True):
    """Run the complete motion planning demonstration."""
    
    print_header("HUMANOID MOTION PLANNING WITH ZMP STABILITY")
    print("""
    This demo shows a complete whole-body motion planning system for
    the Unitree G1 humanoid robot. The planner:
    
    1. Solves inverse kinematics for arm reaching targets
    2. Monitors Zero Moment Point (ZMP) stability throughout motion
    3. Applies waist compensation when stability margin gets tight
    4. Generates smooth, executable trajectories
    """)
    
    # Initialize
    print_header("Initializing Robot Model")
    robot = G1Model()
    planner = WholeBodyPlanner(robot, safety_margin=0.015)
    
    print(f"  Robot: Unitree G1")
    print(f"  Total mass: {robot.total_mass:.2f} kg")
    print(f"  Joints: {robot.model.njnt}")
    print(f"  Actuators: {robot.model.nu}")
    
    # Show initial state
    robot.print_state_summary()
    
    support_poly = robot.get_support_polygon('double')
    print(f"\n  Support polygon bounds:")
    print(f"    X: [{support_poly[:, 0].min():.3f}, {support_poly[:, 0].max():.3f}] m")
    print(f"    Y: [{support_poly[:, 1].min():.3f}, {support_poly[:, 1].max():.3f}] m")
    
    # Plan a sequence of motions
    print_header("Planning Motion Sequence")
    
    right_hand = robot.get_end_effector_position('right_hand')
    left_hand = robot.get_end_effector_position('left_hand')
    
    motions = [
        ("Right arm forward (10cm)", 'right', right_hand + np.array([0.10, 0.0, 0.0])),
        ("Right arm lateral (8cm)", 'right', right_hand + np.array([0.0, -0.08, 0.0])),
        ("Right arm up (6cm)", 'right', right_hand + np.array([0.03, 0.0, 0.06])),
        ("Left arm forward (10cm)", 'left', left_hand + np.array([0.10, 0.0, 0.0])),
    ]
    
    plans = []
    for name, arm, target in motions:
        robot.reset()
        plan = planner.plan_reach(arm, target, duration=1.5)
        status = "✓" if plan.success else "✗"
        margin = f"{plan.min_stability_margin*1000:.1f}mm" if plan.success else "N/A"
        print(f"  {status} {name:30s} | margin: {margin}")
        plans.append((name, arm, plan))
    
    # Visualize results
    if visualize:
        print_header("Generating Visualization Plots")
        for i, (name, arm, plan) in enumerate(plans):
            if plan.success:
                filename = f"demo_motion_{i+1}.png"
                visualize_plan(plan, support_poly, filename)
    
    # Show detailed analysis of one motion
    print_header("Detailed Analysis: Right Arm Forward Reach")
    
    name, arm, plan = plans[0]
    if plan.success:
        print(f"\n  Trajectory duration: {plan.duration:.1f}s")
        print(f"  Trajectory points: {len(plan.trajectory)}")
        print(f"  Minimum stability margin: {plan.min_stability_margin*1000:.1f}mm")
        
        print("\n  Trajectory samples:")
        print("  " + "-" * 60)
        print(f"  {'Time':>6s} | {'CoM X':>8s} | {'CoM Z':>8s} | {'ZMP X':>8s} | {'Margin':>8s}")
        print("  " + "-" * 60)
        
        indices = [0, len(plan.trajectory)//4, len(plan.trajectory)//2, 
                   3*len(plan.trajectory)//4, -1]
        for idx in indices:
            p = plan.trajectory[idx]
            print(f"  {p.time:6.2f}s | {p.com[0]:8.4f} | {p.com[2]:8.4f} | "
                  f"{p.zmp[0]:8.4f} | {p.stability_margin*1000:6.1f}mm")
        
        # Check if waist compensation was used
        waist_used = any(np.abs(p.waist_joints).max() > 0.001 for p in plan.trajectory)
        print(f"\n  Waist compensation: {'Yes' if waist_used else 'No (not needed)'}")
    
    # 3D Visualization
    if visualize:
        print_header("3D Visualization")
        print("\n  Launching MuJoCo viewer to play back trajectories...")
        print("  Close the viewer window to continue.\n")
        
        try:
            from visualizer import MotionVisualizer
            viz = MotionVisualizer(robot)
            
            for name, arm, plan in plans:
                if plan.success:
                    print(f"  Playing: {name}")
                    robot.reset()
                    viz.playback_trajectory(plan, arm, speed=1.0)
                    robot.reset()
        except Exception as e:
            print(f"  Visualization error: {e}")
            print("  (This may happen in headless environments)")
    
    print_header("Demo Complete")
    print("""
    The motion planning system successfully:
    
    ✓ Loaded the Unitree G1 model with accurate kinematics
    ✓ Solved inverse kinematics for multiple reaching targets  
    ✓ Computed ZMP and verified stability throughout trajectories
    ✓ Generated smooth, executable motion plans
    
    Key files created:
    - src/g1_model.py          : Robot model wrapper
    - src/zmp_calculator.py    : ZMP stability computation
    - src/inverse_kinematics.py: Jacobian-based IK solver
    - src/motion_planner.py    : Whole-body planner with ZMP constraints
    - src/visualizer.py        : MuJoCo visualization
    
    Next steps for a production system:
    - Add trajectory optimization (time-optimal, minimum jerk)
    - Implement dynamic ZMP tracking (for faster motions)
    - Add obstacle avoidance
    - Integrate with perception for target detection
    - Deploy to real hardware via unitree_sdk2
    """)


def interactive_mode():
    """Interactive mode for testing custom targets."""
    print_header("Interactive Motion Planning")
    
    robot = G1Model()
    planner = WholeBodyPlanner(robot, safety_margin=0.015)
    
    print("\n  Enter target positions to plan reaching motions.")
    print("  Format: arm x y z (e.g., 'right 0.3 -0.15 0.85')")
    print("  Type 'quit' to exit.\n")
    
    while True:
        try:
            cmd = input("  > ").strip()
            if cmd.lower() == 'quit':
                break
            
            parts = cmd.split()
            if len(parts) != 4:
                print("    Invalid format. Use: arm x y z")
                continue
            
            arm = parts[0]
            if arm not in ['left', 'right']:
                print("    Arm must be 'left' or 'right'")
                continue
            
            target = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            
            robot.reset()
            plan = planner.plan_reach(arm, target, duration=2.0)
            
            if plan.success:
                print(f"    ✓ Plan found! Margin: {plan.min_stability_margin*1000:.1f}mm")
                
                # Visualize
                try:
                    from visualizer import MotionVisualizer
                    viz = MotionVisualizer(robot)
                    viz.playback_trajectory(plan, arm)
                except:
                    pass
            else:
                print(f"    ✗ Planning failed: {plan.message}")
            
            robot.reset()
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"    Error: {e}")
    
    print("\n  Exiting interactive mode.")


if __name__ == '__main__':
    if '--no-viz' in sys.argv:
        run_demo(visualize=False)
    elif '--interactive' in sys.argv:
        interactive_mode()
    else:
        run_demo(visualize=True)
