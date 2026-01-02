"""
Unitree G1 Robot Model Wrapper for MuJoCo

Provides:
- Forward/Inverse Kinematics
- Jacobian computation
- Center of Mass calculation
- Support polygon computation
"""

import mujoco
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, Dict, List
from pathlib import Path


@dataclass
class JointGroup:
    """Defines a group of joints (e.g., left arm, right leg)"""
    name: str
    joint_names: List[str]
    joint_ids: List[int] = None
    actuator_ids: List[int] = None


@dataclass
class RobotState:
    """Complete robot state"""
    qpos: np.ndarray          # Joint positions (including floating base)
    qvel: np.ndarray          # Joint velocities
    com: np.ndarray           # Center of mass position [x, y, z]
    com_vel: np.ndarray       # Center of mass velocity
    left_foot_pos: np.ndarray
    right_foot_pos: np.ndarray
    left_hand_pos: np.ndarray
    right_hand_pos: np.ndarray


class G1Model:
    """
    Unitree G1 Robot Model
    
    Handles all kinematics and dynamics computations using MuJoCo.
    """
    
    # Joint group definitions based on actual G1 structure
    JOINT_GROUPS = {
        'left_arm': [
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint', 
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint'
        ],
        'right_arm': [
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint'
        ],
        'left_leg': [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_hip_yaw_joint',
            'left_knee_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint'
        ],
        'right_leg': [
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_hip_yaw_joint',
            'right_knee_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint'
        ],
        'waist': [
            'waist_yaw_joint',
            'waist_roll_joint',
            'waist_pitch_joint'
        ]
    }
    
    # End-effector body names
    END_EFFECTORS = {
        'left_hand': 'left_wrist_yaw_link',
        'right_hand': 'right_wrist_yaw_link',
        'left_foot': 'left_ankle_roll_link',
        'right_foot': 'right_ankle_roll_link'
    }
    
    # Foot dimensions (approximate, for support polygon)
    FOOT_LENGTH = 0.10  # m, front to back
    FOOT_WIDTH = 0.05   # m, side to side
    
    def __init__(self, model_path: str = None):
        """
        Initialize G1 model.
        
        Args:
            model_path: Path to scene.xml. If None, uses default location.
        """
        if model_path is None:
            # Default path relative to project root
            model_path = Path(__file__).parent.parent / 'mujoco_menagerie' / 'unitree_g1' / 'scene.xml'
        
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model not found at {self.model_path}")
        
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        
        # Cache joint and body IDs for fast lookup
        self._build_id_cache()
        
        # Total robot mass
        self.total_mass = sum(self.model.body_mass)
        
        # Initialize to standing pose
        self.reset()
        
    def _build_id_cache(self):
        """Build caches for joint, actuator, and body IDs"""
        
        # Joint name to ID mapping
        self.joint_ids = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_ids[name] = i
        
        # Actuator name to ID mapping
        self.actuator_ids = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.actuator_ids[name] = i
        
        # Body name to ID mapping
        self.body_ids = {}
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name:
                self.body_ids[name] = i
        
        # Site name to ID mapping
        self.site_ids = {}
        for i in range(self.model.nsite):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            if name:
                self.site_ids[name] = i
        
        # Build joint group ID lists
        self.joint_groups = {}
        for group_name, joint_names in self.JOINT_GROUPS.items():
            joint_ids = [self.joint_ids[jn] for jn in joint_names]
            # Get qpos indices (accounting for floating base which uses 7 qpos values)
            qpos_indices = []
            for jid in joint_ids:
                qpos_idx = self.model.jnt_qposadr[jid]
                qpos_indices.append(qpos_idx)
            
            actuator_ids = [self.actuator_ids.get(jn) for jn in joint_names]
            
            self.joint_groups[group_name] = {
                'joint_names': joint_names,
                'joint_ids': joint_ids,
                'qpos_indices': qpos_indices,
                'actuator_ids': [a for a in actuator_ids if a is not None]
            }
    
    def reset(self):
        """Reset robot to default standing pose"""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
    
    def set_qpos(self, qpos: np.ndarray):
        """Set joint positions and update kinematics"""
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)
    
    def get_qpos(self) -> np.ndarray:
        """Get current joint positions"""
        return self.data.qpos.copy()
    
    def get_joint_positions(self, group: str) -> np.ndarray:
        """Get joint positions for a specific group (e.g., 'left_arm')"""
        indices = self.joint_groups[group]['qpos_indices']
        return self.data.qpos[indices].copy()
    
    def set_joint_positions(self, group: str, positions: np.ndarray):
        """Set joint positions for a specific group"""
        indices = self.joint_groups[group]['qpos_indices']
        self.data.qpos[indices] = positions
        mujoco.mj_forward(self.model, self.data)
    
    def get_joint_limits(self, group: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get joint limits (lower, upper) for a group"""
        joint_ids = self.joint_groups[group]['joint_ids']
        lower = np.array([self.model.jnt_range[jid, 0] for jid in joint_ids])
        upper = np.array([self.model.jnt_range[jid, 1] for jid in joint_ids])
        return lower, upper
    
    # ==================== FORWARD KINEMATICS ====================
    
    def get_body_position(self, body_name: str) -> np.ndarray:
        """Get world position of a body"""
        body_id = self.body_ids[body_name]
        return self.data.xpos[body_id].copy()
    
    def get_body_rotation(self, body_name: str) -> np.ndarray:
        """Get rotation matrix (3x3) of a body in world frame"""
        body_id = self.body_ids[body_name]
        return self.data.xmat[body_id].reshape(3, 3).copy()
    
    def get_body_pose(self, body_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get position and rotation of a body"""
        return self.get_body_position(body_name), self.get_body_rotation(body_name)
    
    def get_end_effector_position(self, effector: str) -> np.ndarray:
        """
        Get end-effector position.
        
        Args:
            effector: One of 'left_hand', 'right_hand', 'left_foot', 'right_foot'
        """
        body_name = self.END_EFFECTORS[effector]
        return self.get_body_position(body_name)
    
    def get_end_effector_pose(self, effector: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get end-effector position and rotation"""
        body_name = self.END_EFFECTORS[effector]
        return self.get_body_pose(body_name)
    
    # ==================== JACOBIANS ====================
    
    def get_jacobian(self, body_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute Jacobian for a body.
        
        Returns:
            jacp: Position Jacobian (3 x nv)
            jacr: Rotation Jacobian (3 x nv)
        """
        body_id = self.body_ids[body_name]
        
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, body_id)
        
        return jacp, jacr
    
    def get_end_effector_jacobian(self, effector: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get Jacobian for an end-effector"""
        body_name = self.END_EFFECTORS[effector]
        return self.get_jacobian(body_name)
    
    def get_arm_jacobian(self, arm: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get Jacobian columns corresponding to arm joints only.
        
        Args:
            arm: 'left' or 'right'
            
        Returns:
            jacp: Position Jacobian for arm joints (3 x 7)
            jacr: Rotation Jacobian for arm joints (3 x 7)
        """
        effector = f'{arm}_hand'
        full_jacp, full_jacr = self.get_end_effector_jacobian(effector)
        
        # Get velocity indices for arm joints
        group = f'{arm}_arm'
        joint_ids = self.joint_groups[group]['joint_ids']
        
        # MuJoCo velocity indices
        dof_indices = []
        for jid in joint_ids:
            dof_idx = self.model.jnt_dofadr[jid]
            dof_indices.append(dof_idx)
        
        jacp = full_jacp[:, dof_indices]
        jacr = full_jacr[:, dof_indices]
        
        return jacp, jacr
    
    # ==================== CENTER OF MASS ====================
    
    def get_com(self) -> np.ndarray:
        """Get center of mass position in world frame"""
        # subtree_com[0] is world, [1] is typically the robot base
        # We compute it manually for accuracy
        com = np.zeros(3)
        total_mass = 0.0
        
        for i in range(1, self.model.nbody):  # Skip world body
            mass = self.model.body_mass[i]
            pos = self.data.xipos[i]  # Body CoM in world frame
            com += mass * pos
            total_mass += mass
        
        return com / total_mass
    
    def get_com_jacobian(self) -> np.ndarray:
        """
        Compute Jacobian of center of mass.
        
        Returns:
            jac_com: (3 x nv) Jacobian mapping joint velocities to CoM velocity
        """
        jac_com = np.zeros((3, self.model.nv))
        
        for i in range(1, self.model.nbody):
            mass = self.model.body_mass[i]
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacBodyCom(self.model, self.data, jacp, jacr, i)
            jac_com += mass * jacp
        
        jac_com /= self.total_mass
        return jac_com
    
    # ==================== SUPPORT POLYGON ====================
    
    def get_foot_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get left and right foot positions"""
        left_id = self.site_ids['left_foot']
        right_id = self.site_ids['right_foot']
        return self.data.site_xpos[left_id].copy(), self.data.site_xpos[right_id].copy()
    
    def get_support_polygon(self, mode: str = 'double') -> np.ndarray:
        """
        Get support polygon vertices.
        
        Args:
            mode: 'double' (both feet), 'left' (left foot only), 'right' (right foot only)
            
        Returns:
            vertices: (N x 2) array of polygon vertices in world XY plane
        """
        left_pos, right_pos = self.get_foot_positions()
        
        # Foot rectangle corners (in foot frame, then transformed)
        half_l = self.FOOT_LENGTH / 2
        half_w = self.FOOT_WIDTH / 2
        
        if mode == 'double':
            # Combine both feet into one polygon
            vertices = np.array([
                [left_pos[0] + half_l, left_pos[1] + half_w],    # Left foot front-left
                [left_pos[0] + half_l, left_pos[1] - half_w],    # Left foot front-right
                [left_pos[0] - half_l, left_pos[1] - half_w],    # Left foot back-right
                [left_pos[0] - half_l, left_pos[1] + half_w],    # Left foot back-left
                [right_pos[0] + half_l, right_pos[1] + half_w],  # Right foot front-left
                [right_pos[0] + half_l, right_pos[1] - half_w],  # Right foot front-right
                [right_pos[0] - half_l, right_pos[1] - half_w],  # Right foot back-right
                [right_pos[0] - half_l, right_pos[1] + half_w],  # Right foot back-left
            ])
            # Compute convex hull
            from scipy.spatial import ConvexHull
            hull = ConvexHull(vertices)
            return vertices[hull.vertices]
        
        elif mode == 'left':
            return np.array([
                [left_pos[0] + half_l, left_pos[1] + half_w],
                [left_pos[0] + half_l, left_pos[1] - half_w],
                [left_pos[0] - half_l, left_pos[1] - half_w],
                [left_pos[0] - half_l, left_pos[1] + half_w],
            ])
        
        elif mode == 'right':
            return np.array([
                [right_pos[0] + half_l, right_pos[1] + half_w],
                [right_pos[0] + half_l, right_pos[1] - half_w],
                [right_pos[0] - half_l, right_pos[1] - half_w],
                [right_pos[0] - half_l, right_pos[1] + half_w],
            ])
    
    # ==================== FULL STATE ====================
    
    def get_state(self) -> RobotState:
        """Get complete robot state"""
        left_foot, right_foot = self.get_foot_positions()
        
        return RobotState(
            qpos=self.data.qpos.copy(),
            qvel=self.data.qvel.copy(),
            com=self.get_com(),
            com_vel=self.data.subtree_linvel[1].copy(),  # Approximate
            left_foot_pos=left_foot,
            right_foot_pos=right_foot,
            left_hand_pos=self.get_end_effector_position('left_hand'),
            right_hand_pos=self.get_end_effector_position('right_hand')
        )
    
    def print_state_summary(self):
        """Print a summary of current robot state"""
        state = self.get_state()
        print("=" * 50)
        print("G1 Robot State Summary")
        print("=" * 50)
        print(f"CoM Position:    [{state.com[0]:.4f}, {state.com[1]:.4f}, {state.com[2]:.4f}] m")
        print(f"Left Hand:       [{state.left_hand_pos[0]:.4f}, {state.left_hand_pos[1]:.4f}, {state.left_hand_pos[2]:.4f}] m")
        print(f"Right Hand:      [{state.right_hand_pos[0]:.4f}, {state.right_hand_pos[1]:.4f}, {state.right_hand_pos[2]:.4f}] m")
        print(f"Left Foot:       [{state.left_foot_pos[0]:.4f}, {state.left_foot_pos[1]:.4f}, {state.left_foot_pos[2]:.4f}] m")
        print(f"Right Foot:      [{state.right_foot_pos[0]:.4f}, {state.right_foot_pos[1]:.4f}, {state.right_foot_pos[2]:.4f}] m")


# Quick test
if __name__ == '__main__':
    print("Testing G1Model...")
    model = G1Model()
    model.print_state_summary()
    
    print("\nJoint groups:")
    for name, group in model.joint_groups.items():
        print(f"  {name}: {len(group['joint_ids'])} joints")
    
    print("\nTesting Jacobian computation...")
    jacp, jacr = model.get_arm_jacobian('left')
    print(f"  Left arm position Jacobian shape: {jacp.shape}")
    print(f"  Left arm rotation Jacobian shape: {jacr.shape}")
    
    print("\nSupport polygon (double support):")
    poly = model.get_support_polygon('double')
    print(f"  Vertices: {len(poly)}")
    for i, v in enumerate(poly):
        print(f"    [{i}]: ({v[0]:.4f}, {v[1]:.4f})")
    
    print("\nTest complete!")
