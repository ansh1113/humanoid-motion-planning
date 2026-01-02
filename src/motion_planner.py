"""
Whole-Body Motion Planner with ZMP Stability Constraints

This is the main planning module that:
1. Takes a target hand position
2. Solves IK for the arm
3. Checks ZMP stability
4. If unstable, compensates with waist/torso adjustment
5. Generates a smooth, stable trajectory
"""

import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass
import time

from g1_model import G1Model, RobotState
from zmp_calculator import ZMPCalculator, ZMPState
from inverse_kinematics import InverseKinematics, IKResult


@dataclass
class TrajectoryPoint:
    """Single point in a trajectory"""
    time: float
    arm_joints: np.ndarray
    waist_joints: np.ndarray
    com: np.ndarray
    zmp: np.ndarray
    stability_margin: float


@dataclass 
class MotionPlan:
    """Complete motion plan"""
    success: bool
    trajectory: List[TrajectoryPoint]
    duration: float
    min_stability_margin: float
    message: str


class WholeBodyPlanner:
    """
    Plans whole-body motions that maintain ZMP stability.
    """
    
    def __init__(
        self,
        robot: G1Model,
        safety_margin: float = 0.015,  # 1.5cm from polygon edge
        trajectory_dt: float = 0.02,   # 50Hz trajectory
        max_waist_compensation: float = 0.4  # Max waist angle for compensation (rad)
    ):
        self.robot = robot
        # More aggressive IK settings
        self.ik = InverseKinematics(
            robot, 
            max_iterations=200, 
            damping=0.01,
            step_size=0.5,
            position_tolerance=0.002  # 2mm tolerance
        )
        self.zmp = ZMPCalculator(safety_margin=safety_margin)
        
        self.safety_margin = safety_margin
        self.dt = trajectory_dt
        self.max_waist = max_waist_compensation
        
    def _compute_com_for_config(
        self,
        arm: str,
        arm_joints: np.ndarray,
        waist_joints: np.ndarray = None
    ) -> np.ndarray:
        """Compute CoM for a given arm and waist configuration."""
        original_qpos = self.robot.get_qpos().copy()
        
        self.robot.set_joint_positions(f'{arm}_arm', arm_joints)
        
        if waist_joints is not None:
            self.robot.set_joint_positions('waist', waist_joints)
        
        com = self.robot.get_com()
        self.robot.set_qpos(original_qpos)
        
        return com
    
    def _find_waist_compensation(
        self,
        arm: str,
        arm_joints: np.ndarray,
        support_polygon: np.ndarray,
        initial_waist: np.ndarray = None
    ) -> Tuple[bool, np.ndarray, ZMPState]:
        """
        Find waist angles that restore ZMP stability.
        
        Uses gradient descent on waist angles to push ZMP toward polygon center.
        """
        if initial_waist is None:
            initial_waist = np.zeros(3)
        
        waist = initial_waist.copy()
        waist_lower, waist_upper = self.robot.get_joint_limits('waist')
        poly_center = np.mean(support_polygon, axis=0)
        
        learning_rate = 0.15
        max_iter = 100
        
        best_waist = waist.copy()
        best_margin = -np.inf
        
        for i in range(max_iter):
            com = self._compute_com_for_config(arm, arm_joints, waist)
            zmp_state = self.zmp.compute_stability(com, support_polygon)
            
            # Track best solution
            if zmp_state.stability_margin > best_margin:
                best_margin = zmp_state.stability_margin
                best_waist = waist.copy()
            
            if zmp_state.stability_margin >= self.safety_margin:
                return True, waist, zmp_state
            
            # Numerical gradient
            eps = 0.02
            grad = np.zeros(3)
            
            for j in range(3):
                waist_plus = waist.copy()
                waist_plus[j] = np.clip(waist[j] + eps, waist_lower[j], waist_upper[j])
                
                com_plus = self._compute_com_for_config(arm, arm_joints, waist_plus)
                zmp_plus = self.zmp.compute_zmp(com_plus)
                
                dist_current = np.linalg.norm(zmp_state.zmp - poly_center)
                dist_plus = np.linalg.norm(zmp_plus - poly_center)
                
                grad[j] = (dist_plus - dist_current) / eps
            
            # Update
            waist = waist - learning_rate * grad
            waist = np.clip(waist, waist_lower, waist_upper)
            waist = np.clip(waist, -self.max_waist, self.max_waist)
        
        # Return best found
        com = self._compute_com_for_config(arm, arm_joints, best_waist)
        zmp_state = self.zmp.compute_stability(com, support_polygon)
        
        return zmp_state.is_stable, best_waist, zmp_state
    
    def plan_reach(
        self,
        arm: str,
        target_position: np.ndarray,
        target_orientation: np.ndarray = None,
        duration: float = 2.0,
        support_mode: str = 'double'
    ) -> MotionPlan:
        """
        Plan a reaching motion with ZMP stability.
        """
        start_time = time.time()
        
        current_arm_joints = self.robot.get_joint_positions(f'{arm}_arm')
        current_waist_joints = self.robot.get_joint_positions('waist')
        support_polygon = self.robot.get_support_polygon(support_mode)
        
        # Check workspace bounds (from our analysis)
        shoulder_pos = self.robot.get_body_position(f'{arm}_shoulder_pitch_link')
        rel_target = target_position - shoulder_pos
        reach_dist = np.linalg.norm(rel_target)
        
        # G1 arm reach is ~0.4m max
        if reach_dist > 0.42:
            return MotionPlan(
                success=False,
                trajectory=[],
                duration=0,
                min_stability_margin=0,
                message=f"Target likely unreachable. Distance from shoulder: {reach_dist:.3f}m (max ~0.40m)"
            )
        
        # Solve IK for final pose using current joints as initial guess
        ik_result = self.ik.solve_arm(
            arm,
            target_position,
            target_orientation,
            initial_guess=current_arm_joints
        )
        
        # Accept if error is small enough
        if ik_result.position_error > 0.005:  # 5mm
            return MotionPlan(
                success=False,
                trajectory=[],
                duration=0,
                min_stability_margin=0,
                message=f"IK failed: position error {ik_result.position_error*1000:.1f}mm after {ik_result.iterations} iterations"
            )
        
        target_arm_joints = ik_result.joint_positions
        
        # Generate trajectory with stability checks
        num_points = int(duration / self.dt)
        trajectory = []
        min_margin = float('inf')
        
        # Progressive IK: use previous solution as starting point
        prev_arm_joints = current_arm_joints.copy()
        prev_waist_joints = current_waist_joints.copy()
        
        for i in range(num_points + 1):
            t = i * self.dt
            alpha = i / num_points
            
            # Smooth interpolation (quintic for smoother motion)
            smooth_alpha = 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5
            
            # Interpolate arm joints
            arm_joints = (1 - smooth_alpha) * current_arm_joints + smooth_alpha * target_arm_joints
            
            # Compute CoM and check stability
            com = self._compute_com_for_config(arm, arm_joints, prev_waist_joints)
            zmp_state = self.zmp.compute_stability(com, support_polygon)
            
            if not zmp_state.is_stable or zmp_state.stability_margin < self.safety_margin:
                # Need waist compensation
                success, waist_joints, zmp_state = self._find_waist_compensation(
                    arm, arm_joints, support_polygon, prev_waist_joints
                )
                
                if not success and zmp_state.stability_margin < 0:
                    return MotionPlan(
                        success=False,
                        trajectory=trajectory,
                        duration=t,
                        min_stability_margin=zmp_state.stability_margin,
                        message=f"Cannot stabilize at t={t:.2f}s. ZMP margin: {zmp_state.stability_margin*1000:.1f}mm"
                    )
            else:
                waist_joints = prev_waist_joints.copy()
            
            point = TrajectoryPoint(
                time=t,
                arm_joints=arm_joints.copy(),
                waist_joints=waist_joints.copy(),
                com=zmp_state.com.copy(),
                zmp=zmp_state.zmp.copy(),
                stability_margin=zmp_state.stability_margin
            )
            trajectory.append(point)
            
            min_margin = min(min_margin, zmp_state.stability_margin)
            prev_waist_joints = waist_joints
        
        planning_time = time.time() - start_time
        
        return MotionPlan(
            success=True,
            trajectory=trajectory,
            duration=duration,
            min_stability_margin=min_margin,
            message=f"Plan found in {planning_time:.2f}s. Min margin: {min_margin*1000:.1f}mm"
        )
    
    def execute_trajectory(
        self,
        plan: MotionPlan,
        arm: str
    ) -> bool:
        """Execute a planned trajectory on the robot model."""
        if not plan.success:
            print(f"Cannot execute failed plan: {plan.message}")
            return False
        
        for point in plan.trajectory:
            self.robot.set_joint_positions(f'{arm}_arm', point.arm_joints)
            self.robot.set_joint_positions('waist', point.waist_joints)
        
        return True


def visualize_plan(plan: MotionPlan, support_polygon: np.ndarray, save_path: str = None):
    """Visualize a motion plan"""
    import matplotlib.pyplot as plt
    
    if not plan.trajectory:
        print("No trajectory to visualize")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    times = [p.time for p in plan.trajectory]
    margins = [p.stability_margin for p in plan.trajectory]
    coms = np.array([p.com for p in plan.trajectory])
    zmps = np.array([p.zmp for p in plan.trajectory])
    
    # Plot 1: ZMP trajectory on support polygon
    ax1 = axes[0, 0]
    poly_closed = np.vstack([support_polygon, support_polygon[0]])
    ax1.plot(poly_closed[:, 0], poly_closed[:, 1], 'b-', linewidth=2, label='Support Polygon')
    ax1.fill(support_polygon[:, 0], support_polygon[:, 1], 'b', alpha=0.1)
    
    # ZMP path with color gradient
    colors = plt.cm.viridis(np.linspace(0, 1, len(zmps)))
    for i in range(len(zmps) - 1):
        ax1.plot(zmps[i:i+2, 0], zmps[i:i+2, 1], color=colors[i], linewidth=2)
    ax1.scatter(zmps[0, 0], zmps[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(zmps[-1, 0], zmps[-1, 1], c='red', s=100, marker='s', label='End', zorder=5)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('ZMP Trajectory on Support Polygon')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Stability margin over time
    ax2 = axes[0, 1]
    ax2.plot(times, np.array(margins)*1000, 'b-', linewidth=2)
    ax2.axhline(y=15, color='orange', linestyle='--', label='Safety margin (15mm)')
    ax2.axhline(y=0, color='r', linestyle='-', label='Polygon edge')
    ax2.fill_between(times, np.array(margins)*1000, 15, 
                     where=np.array(margins)*1000 < 15, 
                     color='orange', alpha=0.3)
    ax2.fill_between(times, np.array(margins)*1000, 0,
                     where=np.array(margins)*1000 < 0,
                     color='red', alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Stability Margin (mm)')
    ax2.set_title('ZMP Stability Margin Over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: CoM trajectory (side view)
    ax3 = axes[1, 0]
    ax3.plot(coms[:, 0], coms[:, 2], 'g-', linewidth=2)
    ax3.scatter(coms[0, 0], coms[0, 2], c='green', s=100, marker='o', label='Start')
    ax3.scatter(coms[-1, 0], coms[-1, 2], c='red', s=100, marker='s', label='End')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('CoM Trajectory (Side View)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Waist joint angles over time
    ax4 = axes[1, 1]
    waist_joints = np.array([p.waist_joints for p in plan.trajectory])
    waist_names = ['Yaw', 'Roll', 'Pitch']
    for i in range(waist_joints.shape[1]):
        ax4.plot(times, np.rad2deg(waist_joints[:, i]), label=f'Waist {waist_names[i]}', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Joint Angle (deg)')
    ax4.set_title('Waist Compensation')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    else:
        plt.show()


# Test with realistic targets
if __name__ == '__main__':
    print("Testing Whole-Body Motion Planner with Realistic Targets...")
    print("=" * 70)
    
    robot = G1Model()
    planner = WholeBodyPlanner(robot, safety_margin=0.015)
    
    support_poly = robot.get_support_polygon('double')
    
    # Get current hand position as reference
    current_hand = robot.get_end_effector_position('right_hand')
    print(f"Current right hand: [{current_hand[0]:.3f}, {current_hand[1]:.3f}, {current_hand[2]:.3f}]")
    
    # Test 1: Moderate forward reach (10cm forward, reachable)
    print("\n" + "=" * 70)
    print("Test 1: Moderate forward reach (10cm forward from current)")
    print("=" * 70)
    
    target1 = current_hand + np.array([0.10, 0.0, 0.0])
    print(f"Target: [{target1[0]:.3f}, {target1[1]:.3f}, {target1[2]:.3f}]")
    
    plan1 = planner.plan_reach('right', target1, duration=1.5)
    print(f"Success: {plan1.success}")
    print(f"Message: {plan1.message}")
    
    if plan1.success:
        print(f"Trajectory points: {len(plan1.trajectory)}")
        # Show waist compensation at end
        final = plan1.trajectory[-1]
        print(f"Final waist angles (deg): [{np.rad2deg(final.waist_joints[0]):.1f}, "
              f"{np.rad2deg(final.waist_joints[1]):.1f}, {np.rad2deg(final.waist_joints[2]):.1f}]")
        
        visualize_plan(plan1, support_poly, 'motion_plan_forward.png')
    
    robot.reset()
    
    # Test 2: Reach down and to the side
    print("\n" + "=" * 70)
    print("Test 2: Reach down and to the side")
    print("=" * 70)
    
    target2 = current_hand + np.array([0.05, -0.10, -0.15])
    print(f"Target: [{target2[0]:.3f}, {target2[1]:.3f}, {target2[2]:.3f}]")
    
    plan2 = planner.plan_reach('right', target2, duration=1.5)
    print(f"Success: {plan2.success}")
    print(f"Message: {plan2.message}")
    
    if plan2.success:
        visualize_plan(plan2, support_poly, 'motion_plan_side.png')
    
    robot.reset()
    
    # Test 3: Reach forward and up (challenging for stability)
    print("\n" + "=" * 70)
    print("Test 3: Reach forward and up")
    print("=" * 70)
    
    target3 = current_hand + np.array([0.12, 0.0, 0.05])
    print(f"Target: [{target3[0]:.3f}, {target3[1]:.3f}, {target3[2]:.3f}]")
    
    plan3 = planner.plan_reach('right', target3, duration=2.0)
    print(f"Success: {plan3.success}")
    print(f"Message: {plan3.message}")
    
    if plan3.success:
        visualize_plan(plan3, support_poly, 'motion_plan_forward_up.png')
    
    robot.reset()
    
    # Test 4: Left arm reach (test symmetry)
    print("\n" + "=" * 70)
    print("Test 4: Left arm reach")
    print("=" * 70)
    
    left_hand = robot.get_end_effector_position('left_hand')
    target4 = left_hand + np.array([0.08, 0.05, -0.05])
    print(f"Left hand current: [{left_hand[0]:.3f}, {left_hand[1]:.3f}, {left_hand[2]:.3f}]")
    print(f"Target: [{target4[0]:.3f}, {target4[1]:.3f}, {target4[2]:.3f}]")
    
    plan4 = planner.plan_reach('left', target4, duration=1.5)
    print(f"Success: {plan4.success}")
    print(f"Message: {plan4.message}")
    
    if plan4.success:
        visualize_plan(plan4, support_poly, 'motion_plan_left_arm.png')
    
    print("\n" + "=" * 70)
    print("Motion planner tests complete!")
    print("=" * 70)
