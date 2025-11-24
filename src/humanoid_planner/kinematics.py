"""
Robot Kinematics Module

Forward and inverse kinematics for humanoid robots.
"""

import numpy as np
from typing import Optional, Tuple


class ForwardKinematics:
    """Forward kinematics for serial chain manipulators."""
    
    def __init__(self, dh_params: np.ndarray):
        """
        Initialize forward kinematics with DH parameters.
        
        Args:
            dh_params: Denavit-Hartenberg parameters (N x 4)
                      Each row is [a, alpha, d, theta_offset]
        """
        self.dh_params = dh_params
        self.num_joints = len(dh_params)
    
    def compute_fk(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics.
        
        Args:
            joint_angles: Joint angles in radians (num_joints,)
            
        Returns:
            position: End-effector position (3,)
            orientation: End-effector orientation as rotation matrix (3x3)
        """
        T = np.eye(4)
        
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            
            # DH transformation matrix
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            
            Ti = np.array([
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,   sa,     ca,    d   ],
                [0,   0,      0,     1   ]
            ])
            
            T = T @ Ti
        
        position = T[:3, 3]
        orientation = T[:3, :3]
        
        return position, orientation
    
    def compute_jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Compute geometric Jacobian matrix.
        
        Args:
            joint_angles: Joint angles (num_joints,)
            
        Returns:
            jacobian: Jacobian matrix (6 x num_joints)
                     First 3 rows are linear velocity
                     Last 3 rows are angular velocity
        """
        J = np.zeros((6, self.num_joints))
        
        # Compute transforms to each joint
        transforms = []
        T = np.eye(4)
        transforms.append(T.copy())
        
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            
            Ti = np.array([
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,   sa,     ca,    d   ],
                [0,   0,      0,     1   ]
            ])
            
            T = T @ Ti
            transforms.append(T.copy())
        
        # End-effector position
        p_end = transforms[-1][:3, 3]
        
        # For each joint
        for i in range(self.num_joints):
            # Joint position and z-axis
            p_i = transforms[i][:3, 3]
            z_i = transforms[i][:3, 2]
            
            # Linear velocity contribution
            J[:3, i] = np.cross(z_i, p_end - p_i)
            
            # Angular velocity contribution
            J[3:, i] = z_i
        
        return J


class InverseKinematics:
    """Inverse kinematics solver using numerical methods."""
    
    def __init__(self, fk: ForwardKinematics):
        """
        Initialize IK solver.
        
        Args:
            fk: Forward kinematics object
        """
        self.fk = fk
        self.num_joints = fk.num_joints
    
    def solve_ik(self,
                 target_pos: np.ndarray,
                 target_orient: Optional[np.ndarray] = None,
                 q_init: Optional[np.ndarray] = None,
                 max_iter: int = 100,
                 tol: float = 1e-4) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics using iterative method.
        
        Args:
            target_pos: Desired end-effector position (3,)
            target_orient: Desired orientation as rotation matrix (3x3), optional
            q_init: Initial joint configuration (num_joints,)
            max_iter: Maximum iterations
            tol: Convergence tolerance
            
        Returns:
            q_solution: Joint angles achieving target pose, or None if failed
        """
        if q_init is None:
            q = np.zeros(self.num_joints)
        else:
            q = q_init.copy()
        
        for iteration in range(max_iter):
            # Compute current end-effector pose
            pos, orient = self.fk.compute_fk(q)
            
            # Position error
            pos_error = target_pos - pos
            pos_error_norm = np.linalg.norm(pos_error)
            
            # Check convergence (position only for now)
            if pos_error_norm < tol:
                return q
            
            # Compute Jacobian
            J = self.fk.compute_jacobian(q)
            
            # Use only position part for now
            J_pos = J[:3, :]
            
            # Compute pseudo-inverse
            J_pinv = np.linalg.pinv(J_pos)
            
            # Update joint angles
            alpha = 0.5  # Step size
            dq = alpha * J_pinv @ pos_error
            q = q + dq
        
        # Failed to converge
        print(f"IK failed to converge after {max_iter} iterations")
        print(f"Final error: {np.linalg.norm(target_pos - self.fk.compute_fk(q)[0]):.6f}")
        return None
    
    def solve_ik_with_constraints(self,
                                  target_pos: np.ndarray,
                                  q_limits: Tuple[np.ndarray, np.ndarray],
                                  q_init: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Solve IK with joint limit constraints using optimization.
        
        Args:
            target_pos: Desired position (3,)
            q_limits: Joint limits as (q_min, q_max)
            q_init: Initial guess
            
        Returns:
            q_solution: Joint configuration or None
        """
        from scipy.optimize import minimize
        
        q_min, q_max = q_limits
        
        if q_init is None:
            q0 = (q_min + q_max) / 2
        else:
            q0 = q_init.copy()
        
        # Cost function: distance to target
        def cost(q):
            pos, _ = self.fk.compute_fk(q)
            return np.linalg.norm(pos - target_pos) ** 2
        
        # Bounds
        bounds = [(q_min[i], q_max[i]) for i in range(self.num_joints)]
        
        # Optimize
        result = minimize(
            cost,
            q0,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-6, 'maxiter': 200}
        )
        
        if result.success and cost(result.x) < 1e-3:
            return result.x
        else:
            return None


def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles (ZYX convention) to rotation matrix.
    
    Args:
        roll: Rotation around X axis (radians)
        pitch: Rotation around Y axis (radians)
        yaw: Rotation around Z axis (radians)
        
    Returns:
        R: Rotation matrix (3x3)
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])
    
    return R


def euler_from_rotation_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    """
    Extract Euler angles (ZYX convention) from rotation matrix.
    
    Args:
        R: Rotation matrix (3x3)
        
    Returns:
        roll, pitch, yaw: Euler angles in radians
    """
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        # Gimbal lock
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    
    return roll, pitch, yaw
