"""
Inverse Kinematics Solver for Unitree G1

Uses damped least-squares (Levenberg-Marquardt) Jacobian-based IK
with null-space optimization for secondary objectives.

The solver finds joint angles q such that:
    f(q) = target_pose
    
Where f is the forward kinematics function.

Key features:
- Position and orientation tracking
- Joint limit enforcement
- Null-space posture optimization
- Singularity-robust damping
"""

import numpy as np
from typing import Tuple, Optional, Dict
from dataclasses import dataclass
import mujoco

from g1_model import G1Model


@dataclass
class IKResult:
    """Result of IK computation"""
    success: bool
    joint_positions: np.ndarray
    position_error: float       # meters
    orientation_error: float    # radians
    iterations: int
    message: str


class InverseKinematics:
    """
    Jacobian-based Inverse Kinematics solver.
    
    Uses Damped Least Squares (DLS) method:
        Δq = J^T (J J^T + λ²I)^{-1} Δx
        
    Where:
        J: Jacobian matrix
        λ: Damping factor (increases near singularities)
        Δx: Task-space error
        Δq: Joint-space correction
    """
    
    def __init__(
        self,
        robot: G1Model,
        position_tolerance: float = 0.001,   # 1mm
        orientation_tolerance: float = 0.01,  # ~0.6 degrees
        max_iterations: int = 100,
        damping: float = 0.05,
        step_size: float = 0.5
    ):
        """
        Args:
            robot: G1Model instance
            position_tolerance: Acceptable position error (m)
            orientation_tolerance: Acceptable orientation error (rad)
            max_iterations: Maximum IK iterations
            damping: Base damping factor for singularity robustness
            step_size: Fraction of computed step to take (for stability)
        """
        self.robot = robot
        self.pos_tol = position_tolerance
        self.ori_tol = orientation_tolerance
        self.max_iter = max_iterations
        self.damping = damping
        self.step_size = step_size
        
    def _rotation_error(
        self,
        current_rot: np.ndarray,
        target_rot: np.ndarray
    ) -> np.ndarray:
        """
        Compute rotation error as axis-angle vector.
        
        Args:
            current_rot: Current rotation matrix (3x3)
            target_rot: Target rotation matrix (3x3)
            
        Returns:
            error: (3,) axis-angle error vector
        """
        # Error rotation: R_error = R_target @ R_current^T
        R_error = target_rot @ current_rot.T
        
        # Convert to axis-angle
        # Using Rodrigues formula inverse
        trace = np.trace(R_error)
        
        if trace >= 3.0 - 1e-6:
            # No rotation needed
            return np.zeros(3)
        elif trace <= -1.0 + 1e-6:
            # 180 degree rotation - find axis
            # Axis is eigenvector with eigenvalue 1
            eigenvalues, eigenvectors = np.linalg.eig(R_error)
            idx = np.argmin(np.abs(eigenvalues - 1.0))
            axis = np.real(eigenvectors[:, idx])
            return np.pi * axis
        else:
            angle = np.arccos((trace - 1) / 2)
            axis = np.array([
                R_error[2, 1] - R_error[1, 2],
                R_error[0, 2] - R_error[2, 0],
                R_error[1, 0] - R_error[0, 1]
            ]) / (2 * np.sin(angle))
            return angle * axis
    
    def solve_arm(
        self,
        arm: str,
        target_position: np.ndarray,
        target_orientation: np.ndarray = None,
        initial_guess: np.ndarray = None,
        position_only: bool = False
    ) -> IKResult:
        """
        Solve IK for one arm.
        
        Args:
            arm: 'left' or 'right'
            target_position: Target end-effector position [x, y, z]
            target_orientation: Target rotation matrix (3x3). If None, ignores orientation.
            initial_guess: Initial joint angles. If None, uses current.
            position_only: If True, only track position (ignore orientation)
            
        Returns:
            IKResult with solution
        """
        if target_orientation is None:
            position_only = True
        
        # Get arm joint info
        group = f'{arm}_arm'
        effector = f'{arm}_hand'
        
        qpos_indices = self.robot.joint_groups[group]['qpos_indices']
        joint_ids = self.robot.joint_groups[group]['joint_ids']
        
        # Get joint limits
        lower, upper = self.robot.get_joint_limits(group)
        
        # Save current full state
        original_qpos = self.robot.get_qpos().copy()
        
        # Initialize arm joints
        if initial_guess is not None:
            q = initial_guess.copy()
        else:
            q = self.robot.get_joint_positions(group)
        
        # IK iteration
        for iteration in range(self.max_iter):
            # Set joints and compute FK
            self.robot.set_joint_positions(group, q)
            
            current_pos, current_rot = self.robot.get_end_effector_pose(effector)
            
            # Compute errors
            pos_error = target_position - current_pos
            pos_error_norm = np.linalg.norm(pos_error)
            
            if position_only:
                ori_error = np.zeros(3)
                ori_error_norm = 0.0
            else:
                ori_error = self._rotation_error(current_rot, target_orientation)
                ori_error_norm = np.linalg.norm(ori_error)
            
            # Check convergence
            if pos_error_norm < self.pos_tol and ori_error_norm < self.ori_tol:
                # Restore robot state but return solution
                self.robot.set_qpos(original_qpos)
                return IKResult(
                    success=True,
                    joint_positions=q,
                    position_error=pos_error_norm,
                    orientation_error=ori_error_norm,
                    iterations=iteration + 1,
                    message="Converged"
                )
            
            # Get Jacobians
            jacp, jacr = self.robot.get_arm_jacobian(arm)
            
            # Build task-space error and Jacobian
            if position_only:
                dx = pos_error
                J = jacp
            else:
                dx = np.concatenate([pos_error, ori_error])
                J = np.vstack([jacp, jacr])
            
            # Damped least squares
            # dq = J^T (J J^T + λ²I)^{-1} dx
            JJT = J @ J.T
            
            # Adaptive damping based on manipulability
            manipulability = np.sqrt(np.linalg.det(JJT))
            lambda_sq = self.damping ** 2
            if manipulability < 0.01:
                # Near singularity - increase damping
                lambda_sq = (self.damping * 10) ** 2
            
            # Solve
            A = JJT + lambda_sq * np.eye(JJT.shape[0])
            dq = J.T @ np.linalg.solve(A, dx)
            
            # Apply step with scaling
            q = q + self.step_size * dq
            
            # Enforce joint limits
            q = np.clip(q, lower, upper)
        
        # Failed to converge
        self.robot.set_qpos(original_qpos)
        
        return IKResult(
            success=False,
            joint_positions=q,
            position_error=pos_error_norm,
            orientation_error=ori_error_norm,
            iterations=self.max_iter,
            message=f"Max iterations reached. Pos error: {pos_error_norm:.4f}m"
        )
    
    def solve_arm_with_nullspace(
        self,
        arm: str,
        target_position: np.ndarray,
        target_orientation: np.ndarray = None,
        preferred_posture: np.ndarray = None,
        nullspace_gain: float = 0.1,
        initial_guess: np.ndarray = None
    ) -> IKResult:
        """
        Solve IK with null-space posture optimization.
        
        The null-space of the Jacobian allows secondary optimization
        without affecting the end-effector task:
        
            dq = J^+ dx + (I - J^+ J) dq_null
            
        Where dq_null pushes toward a preferred posture.
        
        Args:
            arm: 'left' or 'right'
            target_position: Target position
            target_orientation: Target rotation (optional)
            preferred_posture: Preferred joint angles to optimize toward
            nullspace_gain: Weight of null-space optimization
            initial_guess: Initial joint angles
        """
        position_only = target_orientation is None
        
        group = f'{arm}_arm'
        effector = f'{arm}_hand'
        
        qpos_indices = self.robot.joint_groups[group]['qpos_indices']
        lower, upper = self.robot.get_joint_limits(group)
        
        # Default preferred posture: middle of joint ranges
        if preferred_posture is None:
            preferred_posture = (lower + upper) / 2
        
        original_qpos = self.robot.get_qpos().copy()
        
        if initial_guess is not None:
            q = initial_guess.copy()
        else:
            q = self.robot.get_joint_positions(group)
        
        for iteration in range(self.max_iter):
            self.robot.set_joint_positions(group, q)
            
            current_pos, current_rot = self.robot.get_end_effector_pose(effector)
            
            pos_error = target_position - current_pos
            pos_error_norm = np.linalg.norm(pos_error)
            
            if position_only:
                ori_error = np.zeros(3)
                ori_error_norm = 0.0
            else:
                ori_error = self._rotation_error(current_rot, target_orientation)
                ori_error_norm = np.linalg.norm(ori_error)
            
            if pos_error_norm < self.pos_tol and ori_error_norm < self.ori_tol:
                self.robot.set_qpos(original_qpos)
                return IKResult(
                    success=True,
                    joint_positions=q,
                    position_error=pos_error_norm,
                    orientation_error=ori_error_norm,
                    iterations=iteration + 1,
                    message="Converged with null-space optimization"
                )
            
            jacp, jacr = self.robot.get_arm_jacobian(arm)
            
            if position_only:
                dx = pos_error
                J = jacp
            else:
                dx = np.concatenate([pos_error, ori_error])
                J = np.vstack([jacp, jacr])
            
            # Damped pseudo-inverse
            JJT = J @ J.T
            lambda_sq = self.damping ** 2
            A = JJT + lambda_sq * np.eye(JJT.shape[0])
            J_pinv = J.T @ np.linalg.inv(A)
            
            # Primary task: reach target
            dq_primary = J_pinv @ dx
            
            # Null-space projector: (I - J^+ J)
            n = J.shape[1]  # number of joints
            N = np.eye(n) - J_pinv @ J
            
            # Secondary task: move toward preferred posture
            dq_null = nullspace_gain * (preferred_posture - q)
            dq_secondary = N @ dq_null
            
            # Combined update
            dq = dq_primary + dq_secondary
            
            q = q + self.step_size * dq
            q = np.clip(q, lower, upper)
        
        self.robot.set_qpos(original_qpos)
        return IKResult(
            success=False,
            joint_positions=q,
            position_error=pos_error_norm,
            orientation_error=ori_error_norm,
            iterations=self.max_iter,
            message=f"Max iterations reached. Pos error: {pos_error_norm:.4f}m"
        )
    
    def check_reachability(
        self,
        arm: str,
        target_position: np.ndarray
    ) -> Tuple[bool, float]:
        """
        Quick check if a position is potentially reachable.
        
        Args:
            arm: 'left' or 'right'
            target_position: Target position
            
        Returns:
            reachable: True if position might be reachable
            distance_to_shoulder: Distance from shoulder to target
        """
        # Get shoulder position
        shoulder_body = f'{arm}_shoulder_pitch_link'
        shoulder_pos = self.robot.get_body_position(shoulder_body)
        
        # Approximate arm length
        arm_length = 0.45  # Approximate max reach for G1
        
        distance = np.linalg.norm(target_position - shoulder_pos)
        reachable = distance < arm_length
        
        return reachable, distance


# Test
if __name__ == '__main__':
    print("Testing Inverse Kinematics Solver...")
    
    robot = G1Model()
    ik = InverseKinematics(robot)
    
    # Get current hand position
    current_pos = robot.get_end_effector_position('right_hand')
    print(f"\nCurrent right hand position: [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}]")
    
    # Test 1: Reach slightly forward
    print("\n" + "=" * 50)
    print("Test 1: Reach 10cm forward from current position")
    print("=" * 50)
    
    target1 = current_pos + np.array([0.1, 0.0, 0.0])
    print(f"Target: [{target1[0]:.4f}, {target1[1]:.4f}, {target1[2]:.4f}]")
    
    result1 = ik.solve_arm('right', target1)
    print(f"Success: {result1.success}")
    print(f"Iterations: {result1.iterations}")
    print(f"Position error: {result1.position_error:.6f} m")
    print(f"Joint angles (deg): {np.rad2deg(result1.joint_positions)}")
    
    # Verify by applying solution
    robot.set_joint_positions('right_arm', result1.joint_positions)
    achieved_pos = robot.get_end_effector_position('right_hand')
    print(f"Achieved position: [{achieved_pos[0]:.4f}, {achieved_pos[1]:.4f}, {achieved_pos[2]:.4f}]")
    
    robot.reset()
    
    # Test 2: Reach to the side
    print("\n" + "=" * 50)
    print("Test 2: Reach to the side (y = -0.3)")
    print("=" * 50)
    
    target2 = np.array([0.2, -0.3, 0.9])
    print(f"Target: [{target2[0]:.4f}, {target2[1]:.4f}, {target2[2]:.4f}]")
    
    result2 = ik.solve_arm('right', target2)
    print(f"Success: {result2.success}")
    print(f"Iterations: {result2.iterations}")
    print(f"Position error: {result2.position_error:.6f} m")
    
    robot.set_joint_positions('right_arm', result2.joint_positions)
    achieved_pos2 = robot.get_end_effector_position('right_hand')
    print(f"Achieved position: [{achieved_pos2[0]:.4f}, {achieved_pos2[1]:.4f}, {achieved_pos2[2]:.4f}]")
    
    robot.reset()
    
    # Test 3: With null-space optimization
    print("\n" + "=" * 50)
    print("Test 3: Same target with null-space posture optimization")
    print("=" * 50)
    
    # Preferred posture: slightly bent elbow
    preferred = np.array([0.0, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0])
    
    result3 = ik.solve_arm_with_nullspace('right', target2, preferred_posture=preferred)
    print(f"Success: {result3.success}")
    print(f"Iterations: {result3.iterations}")
    print(f"Position error: {result3.position_error:.6f} m")
    print(f"Joint angles (deg): {np.rad2deg(result3.joint_positions)}")
    
    # Test 4: Unreachable target
    print("\n" + "=" * 50)
    print("Test 4: Unreachable target (too far)")
    print("=" * 50)
    
    target4 = np.array([1.0, -0.5, 1.0])  # Way too far
    reachable, dist = ik.check_reachability('right', target4)
    print(f"Target: [{target4[0]:.4f}, {target4[1]:.4f}, {target4[2]:.4f}]")
    print(f"Distance to shoulder: {dist:.4f} m")
    print(f"Potentially reachable: {reachable}")
    
    if not reachable:
        print("Skipping IK solve - target is out of reach")
    
    print("\nIK tests complete!")
