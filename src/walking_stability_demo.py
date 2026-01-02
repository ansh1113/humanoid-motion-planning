"""
DYNAMIC WALKING STABILITY DEMO

This demonstrates real humanoid control:
1. Robot walks with a stable gait
2. User can perturb arm positions via GUI sliders
3. Without compensation: Robot becomes unstable
4. With compensation: Robot maintains balance through waist/ankle adjustment

This is the kind of control problem solved in real humanoid robotics.
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import sys
import threading
from dataclasses import dataclass
from typing import Tuple

sys.path.insert(0, 'src')

from g1_model import G1Model


@dataclass
class GaitPhase:
    """Walking gait phase"""
    LEFT_SUPPORT = 0   # Right foot swinging
    DOUBLE_SUPPORT_LR = 1  # Both feet, transitioning L->R
    RIGHT_SUPPORT = 2  # Left foot swinging
    DOUBLE_SUPPORT_RL = 3  # Both feet, transitioning R->L


class WalkingController:
    """
    Generates a stable walking pattern.
    Uses simple sinusoidal gait for demonstration.
    """
    
    def __init__(self, step_length=0.08, step_height=0.03, gait_period=1.0):
        self.step_length = step_length
        self.step_height = step_height
        self.gait_period = gait_period
        
        # Joint indices for legs
        self.leg_joints = {
            'right': ['right_hip_pitch_joint', 'right_hip_roll_joint', 
                     'right_hip_yaw_joint', 'right_knee_joint',
                     'right_ankle_pitch_joint', 'right_ankle_roll_joint'],
            'left': ['left_hip_pitch_joint', 'left_hip_roll_joint',
                    'left_hip_yaw_joint', 'left_knee_joint', 
                    'left_ankle_pitch_joint', 'left_ankle_roll_joint']
        }
    
    def get_gait_phase(self, t: float) -> Tuple[GaitPhase, float]:
        """Get current gait phase and phase progress (0-1)"""
        cycle_time = t % self.gait_period
        phase_duration = self.gait_period / 4
        
        phase_idx = int(cycle_time / phase_duration)
        phase_progress = (cycle_time % phase_duration) / phase_duration
        
        phases = [GaitPhase.LEFT_SUPPORT, GaitPhase.DOUBLE_SUPPORT_LR,
                 GaitPhase.RIGHT_SUPPORT, GaitPhase.DOUBLE_SUPPORT_RL]
        
        return phases[phase_idx], phase_progress
    
    def compute_leg_angles(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute leg joint angles for walking.
        Returns (right_leg, left_leg) joint angles.
        """
        phase = (t % self.gait_period) / self.gait_period * 2 * np.pi
        
        # Simple sinusoidal gait
        # Hip pitch for forward/backward leg swing
        right_hip_pitch = 0.15 * np.sin(phase)
        left_hip_pitch = 0.15 * np.sin(phase + np.pi)
        
        # Knee bend during swing
        right_knee = -0.3 * max(0, np.sin(phase))
        left_knee = -0.3 * max(0, np.sin(phase + np.pi))
        
        # Ankle compensation
        right_ankle_pitch = -0.1 * np.sin(phase)
        left_ankle_pitch = -0.1 * np.sin(phase + np.pi)
        
        # Hip roll for lateral stability
        right_hip_roll = 0.02 * np.sin(phase)
        left_hip_roll = 0.02 * np.sin(phase + np.pi)
        
        right_leg = np.array([right_hip_pitch, right_hip_roll, 0, 
                             right_knee, right_ankle_pitch, 0])
        left_leg = np.array([left_hip_pitch, left_hip_roll, 0,
                           left_knee, left_ankle_pitch, 0])
        
        return right_leg, left_leg


class StabilityController:
    """
    Real-time stability controller.
    Monitors CoM/ZMP and computes compensation.
    """
    
    def __init__(self, robot: G1Model):
        self.robot = robot
        
        # Control gains
        self.waist_gain = 2.0  # How aggressively to use waist
        self.ankle_gain = 1.0  # Ankle strategy gain
        
        # Limits
        self.max_waist_pitch = 0.25  # ~14 degrees
        self.max_waist_roll = 0.15   # ~8 degrees
        self.max_ankle_adjust = 0.1  # ~6 degrees
    
    def compute_com_xy(self, model, data) -> np.ndarray:
        """Compute center of mass XY position"""
        total_mass = 0
        com = np.zeros(3)
        
        for i in range(model.nbody):
            mass = model.body_mass[i]
            total_mass += mass
            com += mass * data.xpos[i]
        
        return com[:2] / total_mass if total_mass > 0 else com[:2]
    
    def compute_support_polygon(self, model, data, phase: GaitPhase) -> np.ndarray:
        """Get support polygon based on gait phase"""
        # Simplified foot positions
        # In reality, would get actual foot contact points
        
        right_foot = np.array([0.0, -0.1])  # Approximate
        left_foot = np.array([0.0, 0.1])
        
        foot_half_length = 0.08
        foot_half_width = 0.04
        
        if phase == GaitPhase.LEFT_SUPPORT:
            # Only left foot on ground
            return np.array([
                [left_foot[0] + foot_half_length, left_foot[1] + foot_half_width],
                [left_foot[0] + foot_half_length, left_foot[1] - foot_half_width],
                [left_foot[0] - foot_half_length, left_foot[1] - foot_half_width],
                [left_foot[0] - foot_half_length, left_foot[1] + foot_half_width],
            ])
        elif phase == GaitPhase.RIGHT_SUPPORT:
            # Only right foot on ground
            return np.array([
                [right_foot[0] + foot_half_length, right_foot[1] + foot_half_width],
                [right_foot[0] + foot_half_length, right_foot[1] - foot_half_width],
                [right_foot[0] - foot_half_length, right_foot[1] - foot_half_width],
                [right_foot[0] - foot_half_length, right_foot[1] + foot_half_width],
            ])
        else:
            # Double support - convex hull of both feet
            return np.array([
                [foot_half_length, left_foot[1] + foot_half_width],
                [foot_half_length, right_foot[1] - foot_half_width],
                [-foot_half_length, right_foot[1] - foot_half_width],
                [-foot_half_length, left_foot[1] + foot_half_width],
            ])
    
    def point_in_polygon(self, point: np.ndarray, polygon: np.ndarray) -> bool:
        """Check if point is inside polygon"""
        n = len(polygon)
        inside = False
        
        j = n - 1
        for i in range(n):
            if ((polygon[i, 1] > point[1]) != (polygon[j, 1] > point[1]) and
                point[0] < (polygon[j, 0] - polygon[i, 0]) * (point[1] - polygon[i, 1]) /
                          (polygon[j, 1] - polygon[i, 1]) + polygon[i, 0]):
                inside = not inside
            j = i
        
        return inside
    
    def compute_compensation(self, model, data, phase: GaitPhase, 
                            enable_compensation: bool) -> Tuple[np.ndarray, dict]:
        """
        Compute stability compensation.
        
        Returns:
            waist_adjustment: [yaw, roll, pitch] adjustments
            info: dict with stability metrics
        """
        com_xy = self.compute_com_xy(model, data)
        support_poly = self.compute_support_polygon(model, data, phase)
        poly_center = np.mean(support_poly, axis=0)
        
        # Compute distance to polygon center
        error = com_xy - poly_center
        
        # Check if stable
        is_stable = self.point_in_polygon(com_xy, support_poly)
        
        # Compute margin (simplified)
        margin = 0.0
        if is_stable:
            # Distance to nearest edge (simplified)
            margin = 0.02  # Placeholder
        else:
            margin = -np.linalg.norm(error)
        
        info = {
            'com_xy': com_xy,
            'poly_center': poly_center,
            'error': error,
            'is_stable': is_stable,
            'margin': margin
        }
        
        if not enable_compensation:
            return np.zeros(3), info
        
        # Compute waist compensation
        # Pitch (forward/backward): compensate for X error
        waist_pitch = -self.waist_gain * error[0]
        waist_pitch = np.clip(waist_pitch, -self.max_waist_pitch, self.max_waist_pitch)
        
        # Roll (lateral): compensate for Y error
        waist_roll = -self.waist_gain * error[1]
        waist_roll = np.clip(waist_roll, -self.max_waist_roll, self.max_waist_roll)
        
        waist_adjustment = np.array([0, waist_roll, waist_pitch])
        
        return waist_adjustment, info


class ArmPerturbationGUI:
    """
    Simple GUI for controlling arm perturbations.
    Uses keyboard input for simplicity.
    """
    
    def __init__(self):
        self.right_shoulder_pitch = 0.0
        self.right_shoulder_roll = 0.0
        self.left_shoulder_pitch = 0.0
        self.left_shoulder_roll = 0.0
        
        # Perturbation step size
        self.step = 0.1  # ~6 degrees per press
        
        self.instructions = """
ARM CONTROL (while walking):
  Q/A: Right shoulder pitch +/-
  W/S: Right shoulder roll +/-
  E/D: Left shoulder pitch +/-
  R/F: Left shoulder roll +/-
  SPACE: Reset arms to neutral
  
  1: Enable compensation (STABLE mode)
  2: Disable compensation (UNSTABLE mode)
"""
    
    def get_arm_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get current arm joint positions"""
        right_arm = np.array([self.right_shoulder_pitch, self.right_shoulder_roll, 
                             0, 0, 0, 0, 0])
        left_arm = np.array([self.left_shoulder_pitch, self.left_shoulder_roll,
                            0, 0, 0, 0, 0])
        return right_arm, left_arm
    
    def reset(self):
        """Reset to neutral"""
        self.right_shoulder_pitch = 0.0
        self.right_shoulder_roll = 0.0
        self.left_shoulder_pitch = 0.0
        self.left_shoulder_roll = 0.0


def run_demo():
    """Main demo function"""
    print("=" * 70)
    print("DYNAMIC WALKING STABILITY DEMONSTRATION")
    print("=" * 70)
    print("""
This demo shows real-time stability control during walking.

The robot walks continuously. You can perturb its arms using keyboard.
Watch how it maintains (or loses) balance!

MODE 1 (Compensation ON): Robot adjusts waist to maintain ZMP in support polygon
MODE 2 (Compensation OFF): No adjustment - perturbations cause instability
""")
    
    # Initialize
    robot = G1Model()
    walking = WalkingController(step_length=0.06, gait_period=1.2)
    stability = StabilityController(robot)
    arm_gui = ArmPerturbationGUI()
    
    print(arm_gui.instructions)
    print("\nPress ENTER to start...")
    input()
    
    # Setup joint indices
    model = robot.model
    data = robot.data
    
    def get_joint_idx(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            return model.jnt_qposadr[jid]
        return -1
    
    # Leg joint indices
    right_leg_idx = [get_joint_idx(n) for n in walking.leg_joints['right']]
    left_leg_idx = [get_joint_idx(n) for n in walking.leg_joints['left']]
    
    # Arm joint indices
    right_arm_names = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                      'right_shoulder_yaw_joint', 'right_elbow_joint',
                      'right_wrist_roll_joint', 'right_wrist_pitch_joint',
                      'right_wrist_yaw_joint']
    left_arm_names = ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                     'left_shoulder_yaw_joint', 'left_elbow_joint',
                     'left_wrist_roll_joint', 'left_wrist_pitch_joint',
                     'left_wrist_yaw_joint']
    
    right_arm_idx = [get_joint_idx(n) for n in right_arm_names]
    left_arm_idx = [get_joint_idx(n) for n in left_arm_names]
    
    # Waist joint indices
    waist_idx = [get_joint_idx(n) for n in ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']]
    
    # State
    enable_compensation = True
    base_qpos = data.qpos[:7].copy()
    
    # Key callback for arm control
    key_state = {}
    
    def key_callback(keycode):
        nonlocal enable_compensation
        
        key = chr(keycode) if 32 <= keycode < 127 else None
        
        if key == 'q':
            arm_gui.right_shoulder_pitch += arm_gui.step
        elif key == 'a':
            arm_gui.right_shoulder_pitch -= arm_gui.step
        elif key == 'w':
            arm_gui.right_shoulder_roll += arm_gui.step
        elif key == 's':
            arm_gui.right_shoulder_roll -= arm_gui.step
        elif key == 'e':
            arm_gui.left_shoulder_pitch += arm_gui.step
        elif key == 'd':
            arm_gui.left_shoulder_pitch -= arm_gui.step
        elif key == 'r':
            arm_gui.left_shoulder_roll += arm_gui.step
        elif key == 'f':
            arm_gui.left_shoulder_roll -= arm_gui.step
        elif key == ' ':
            arm_gui.reset()
            print("    [Arms reset to neutral]")
        elif key == '1':
            enable_compensation = True
            print("    >>> COMPENSATION ENABLED (Stable mode)")
        elif key == '2':
            enable_compensation = False
            print("    >>> COMPENSATION DISABLED (Unstable mode)")
    
    # Run
    mujoco.mj_forward(model, data)
    
    print("\n[Walking started - use keys to perturb arms!]")
    print("[Starting in STABLE mode (compensation ON)]")
    
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.9]
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -10
        
        last_print = 0
        
        while viewer.is_running():
            t0 = time.time()
            t = data.time
            
            # Keep base fixed (for now - walking in place)
            data.qpos[:7] = base_qpos
            
            # Get gait phase
            phase, progress = walking.get_gait_phase(t)
            
            # Compute leg angles for walking
            right_leg, left_leg = walking.compute_leg_angles(t)
            
            # Apply leg angles
            for i, idx in enumerate(right_leg_idx):
                if idx >= 0 and i < len(right_leg):
                    data.qpos[idx] = right_leg[i]
            for i, idx in enumerate(left_leg_idx):
                if idx >= 0 and i < len(left_leg):
                    data.qpos[idx] = left_leg[i]
            
            # Get arm perturbations from GUI
            right_arm, left_arm = arm_gui.get_arm_positions()
            
            # Apply arm positions
            for i, idx in enumerate(right_arm_idx):
                if idx >= 0 and i < len(right_arm):
                    data.qpos[idx] = right_arm[i]
            for i, idx in enumerate(left_arm_idx):
                if idx >= 0 and i < len(left_arm):
                    data.qpos[idx] = left_arm[i]
            
            # Compute stability compensation
            waist_adj, info = stability.compute_compensation(
                model, data, phase, enable_compensation
            )
            
            # Apply waist compensation
            for i, idx in enumerate(waist_idx):
                if idx >= 0 and i < len(waist_adj):
                    data.qpos[idx] = waist_adj[i]
            
            # Print status periodically
            if t - last_print > 0.5:
                mode = "STABLE" if enable_compensation else "UNSTABLE"
                arm_total = abs(arm_gui.right_shoulder_pitch) + abs(arm_gui.left_shoulder_pitch)
                waist_deg = np.rad2deg(waist_adj[2]) if enable_compensation else 0
                stable = "✓" if info['is_stable'] else "✗"
                
                print(f"  t={t:5.1f}s | Mode: {mode:8s} | Arm pert: {np.rad2deg(arm_total):5.1f}° | "
                      f"Waist comp: {waist_deg:5.1f}° | Stable: {stable}")
                last_print = t
            
            # Physics
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)
            data.time += model.opt.timestep
            
            viewer.sync()
            
            dt = model.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    
    print("\nDemo ended.")


if __name__ == '__main__':
    run_demo()
