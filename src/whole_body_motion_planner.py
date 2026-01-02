"""
Whole-Body Motion Planner for G1 Humanoid - FIXED IK

Key insight from testing:
- shoulder_pitch NEGATIVE = arm reaches FORWARD
- shoulder_pitch POSITIVE = arm goes BACKWARD
- shoulder_roll MORE NEGATIVE = arm reaches further out (for right arm)
- elbow MORE POSITIVE = more bend
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
from pathlib import Path
from typing import Dict, List, Tuple
from dataclasses import dataclass


@dataclass
class ReachingTask:
    target_position: np.ndarray
    arm: str = 'right'
    duration: float = 3.0
    maintain_balance: bool = True


@dataclass
class TaskResult:
    success: bool
    final_position_error: float
    min_stability_margin: float
    zmp_violations: int
    message: str


class WholeBodyMotionPlanner:
    
    def __init__(self):
        print("=" * 60)
        print("Whole-Body Motion Planner")
        print("=" * 60)
        
        self.base_path = Path(__file__).parent.parent
        xml_path = self.base_path / "mujoco_menagerie/unitree_g1/scene.xml"
        
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        
        # Actuator map
        self.actuators = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.actuators[name] = i
        
        # Find end effector bodies (use the last wrist link)
        self.ee_bodies = {'left': None, 'right': None}
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name:
                if 'left_wrist_yaw' in name.lower():
                    self.ee_bodies['left'] = i
                elif 'right_wrist_yaw' in name.lower():
                    self.ee_bodies['right'] = i
        
        print(f"Model: {self.model.nu} DOF")
        print(f"EE bodies: {self.ee_bodies}")
        
        # Standing pose
        self.standing_pose = {
            # Legs
            'left_hip_pitch_joint': -0.1,
            'left_hip_roll_joint': 0.0,
            'left_hip_yaw_joint': 0.0,
            'left_knee_joint': 0.25,
            'left_ankle_pitch_joint': -0.15,
            'left_ankle_roll_joint': 0.0,
            'right_hip_pitch_joint': -0.1,
            'right_hip_roll_joint': 0.0,
            'right_hip_yaw_joint': 0.0,
            'right_knee_joint': 0.25,
            'right_ankle_pitch_joint': -0.15,
            'right_ankle_roll_joint': 0.0,
            # Waist
            'waist_yaw_joint': 0.0,
            'waist_roll_joint': 0.0,
            'waist_pitch_joint': 0.0,
            # Arms - neutral position
            'left_shoulder_pitch_joint': 0.0,
            'left_shoulder_roll_joint': 0.2,
            'left_shoulder_yaw_joint': 0.0,
            'left_elbow_joint': 0.3,
            'left_wrist_roll_joint': 0.0,
            'left_wrist_pitch_joint': 0.0,
            'left_wrist_yaw_joint': 0.0,
            'right_shoulder_pitch_joint': 0.0,
            'right_shoulder_roll_joint': -0.2,
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 0.3,
            'right_wrist_roll_joint': 0.0,
            'right_wrist_pitch_joint': 0.0,
            'right_wrist_yaw_joint': 0.0,
        }
        
        self.current_pose = self.standing_pose.copy()
        self.task_counter = 0
        self.successful_tasks = 0
        
        print("Ready!")
    
    def _apply_pose(self):
        for name, target in self.current_pose.items():
            if name in self.actuators:
                self.data.ctrl[self.actuators[name]] = target
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.current_pose = self.standing_pose.copy()
        
        for _ in range(2000):
            self._apply_pose()
            mujoco.mj_step(self.model, self.data)
        
        print(f"Reset. Height: {self.data.qpos[2]:.3f}m")
    
    def get_state(self) -> Dict:
        pos = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        com = self._compute_com()
        zmp = com[:2].copy()
        margin = self._stability_margin(zmp)
        
        return {
            'height': pos[2],
            'pitch': pitch,
            'roll': roll,
            'com': com,
            'zmp': zmp,
            'stability_margin': margin,
            'is_stable': margin > 0,
            'is_fallen': pos[2] < 0.5 or abs(pitch) > 0.7 or abs(roll) > 0.7
        }
    
    def _compute_com(self) -> np.ndarray:
        total_mass = 0
        com = np.zeros(3)
        for i in range(self.model.nbody):
            m = self.model.body_mass[i]
            com += m * self.data.xipos[i]
            total_mass += m
        return com / total_mass
    
    def _stability_margin(self, zmp: np.ndarray) -> float:
        # Simple box support polygon
        foot_x, foot_y = 0.10, 0.15
        base = self.data.qpos[:2]
        return min(
            zmp[0] - (base[0] - foot_x),
            (base[0] + foot_x) - zmp[0],
            zmp[1] - (base[1] - foot_y),
            (base[1] + foot_y) - zmp[1]
        )
    
    def get_ee_position(self, arm: str) -> np.ndarray:
        if self.ee_bodies[arm] is not None:
            return self.data.xipos[self.ee_bodies[arm]].copy()
        return np.zeros(3)
    
    def solve_arm_ik(self, target: np.ndarray, arm: str) -> Dict:
        """
        Solve IK using iterative Jacobian method.
        
        Actually moves the arm in simulation to find the right angles.
        """
        # Store original state
        orig_qpos = self.data.qpos.copy()
        orig_ctrl = self.data.ctrl.copy()
        
        # Arm joint names
        if arm == 'right':
            joints = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 
                     'right_shoulder_yaw_joint', 'right_elbow_joint']
        else:
            joints = ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                     'left_shoulder_yaw_joint', 'left_elbow_joint']
        
        # Current angles
        angles = {j: self.current_pose.get(j, 0) for j in joints}
        
        # Iterative IK
        best_error = float('inf')
        best_angles = angles.copy()
        
        learning_rate = 0.5
        
        for iteration in range(50):
            # Apply current angles
            for j, a in angles.items():
                if j in self.actuators:
                    self.data.ctrl[self.actuators[j]] = a
            
            # Step simulation
            for _ in range(20):
                self._apply_pose()
                for j, a in angles.items():
                    if j in self.actuators:
                        self.data.ctrl[self.actuators[j]] = a
                mujoco.mj_step(self.model, self.data)
            
            # Get current EE position
            ee = self.get_ee_position(arm)
            error = target - ee
            error_norm = np.linalg.norm(error)
            
            if error_norm < best_error:
                best_error = error_norm
                best_angles = angles.copy()
            
            if error_norm < 0.02:  # 2cm tolerance
                break
            
            # Compute numerical Jacobian
            J = np.zeros((3, len(joints)))
            eps = 0.02
            
            for j_idx, joint_name in enumerate(joints):
                orig_angle = angles[joint_name]
                
                # Perturb +
                angles[joint_name] = orig_angle + eps
                for j, a in angles.items():
                    if j in self.actuators:
                        self.data.ctrl[self.actuators[j]] = a
                for _ in range(10):
                    mujoco.mj_step(self.model, self.data)
                ee_plus = self.get_ee_position(arm)
                
                # Perturb -
                angles[joint_name] = orig_angle - eps
                for j, a in angles.items():
                    if j in self.actuators:
                        self.data.ctrl[self.actuators[j]] = a
                for _ in range(10):
                    mujoco.mj_step(self.model, self.data)
                ee_minus = self.get_ee_position(arm)
                
                J[:, j_idx] = (ee_plus - ee_minus) / (2 * eps)
                angles[joint_name] = orig_angle
            
            # Damped least squares
            damping = 0.1
            JJT = J @ J.T + damping**2 * np.eye(3)
            delta_angles = J.T @ np.linalg.solve(JJT, error)
            
            # Update angles
            for j_idx, joint_name in enumerate(joints):
                angles[joint_name] += learning_rate * delta_angles[j_idx]
                # Clamp to reasonable range
                angles[joint_name] = np.clip(angles[joint_name], -2.0, 2.0)
        
        # Restore original state
        self.data.qpos[:] = orig_qpos
        self.data.ctrl[:] = orig_ctrl
        mujoco.mj_forward(self.model, self.data)
        
        # Return best found angles for all arm joints
        result = {}
        for j in joints:
            result[j] = best_angles.get(j, 0)
        
        # Keep wrist at zero
        wrist_joints = [f'{arm}_wrist_roll_joint', f'{arm}_wrist_pitch_joint', f'{arm}_wrist_yaw_joint']
        for j in wrist_joints:
            result[j] = 0.0
        
        return result, best_error
    
    def compute_waist_compensation(self, arm_pose: Dict, arm: str) -> Dict:
        """Adjust waist to maintain balance when arm moves."""
        sp = arm_pose.get(f'{arm}_shoulder_pitch_joint', 0)
        sr = arm_pose.get(f'{arm}_shoulder_roll_joint', 0)
        
        # Negative shoulder pitch = forward reach
        # Need to lean back slightly
        waist_pitch = 0.1 * sp  # If sp is negative (forward), lean forward slightly
        
        # Lateral compensation
        waist_roll = 0.05 * abs(sr)
        if arm == 'left':
            waist_roll = -waist_roll
        
        return {
            'waist_pitch_joint': np.clip(waist_pitch, -0.2, 0.2),
            'waist_roll_joint': np.clip(waist_roll, -0.1, 0.1),
            'waist_yaw_joint': 0.0,
        }
    
    def execute_task(self, task: ReachingTask, visualize: bool = True) -> TaskResult:
        self.task_counter += 1
        
        print(f"\n{'='*60}")
        print(f"Task {self.task_counter}: Reach to {task.target_position}")
        print(f"{'='*60}")
        
        start_ee = self.get_ee_position(task.arm)
        print(f"  Start EE: {start_ee}")
        print(f"  Target:   {task.target_position}")
        
        # Solve IK
        print("  Solving IK...")
        arm_pose, ik_error = self.solve_arm_ik(task.target_position, task.arm)
        print(f"  IK error: {ik_error:.4f}m")
        
        # Balance compensation
        waist_comp = self.compute_waist_compensation(arm_pose, task.arm) if task.maintain_balance else {}
        
        # Build target
        start_pose = self.current_pose.copy()
        end_pose = self.standing_pose.copy()
        end_pose.update(arm_pose)
        end_pose.update(waist_comp)
        
        # Visualize
        viewer = None
        if visualize:
            viewer = mujoco.viewer.launch_passive(self.model, self.data)
            viewer.cam.distance = 2.5
            viewer.cam.elevation = -15
            viewer.cam.azimuth = 135
        
        # Execute
        dt = self.model.opt.timestep
        n_steps = int(task.duration / dt)
        min_margin = float('inf')
        zmp_violations = 0
        fallen = False
        
        print(f"  Executing {n_steps} steps...")
        
        try:
            for step in range(n_steps):
                t = step / n_steps
                s = 10*t**3 - 15*t**4 + 6*t**5
                
                for name in end_pose:
                    if name in start_pose:
                        self.current_pose[name] = (1-s)*start_pose[name] + s*end_pose[name]
                
                self._apply_pose()
                mujoco.mj_step(self.model, self.data)
                
                if step % 200 == 0:
                    state = self.get_state()
                    min_margin = min(min_margin, state['stability_margin'])
                    if state['stability_margin'] < 0.01:
                        zmp_violations += 1
                    if state['is_fallen']:
                        fallen = True
                        print(f"  FALLEN at step {step}")
                        break
                
                if viewer and viewer.is_running() and step % 100 == 0:
                    viewer.sync()
        finally:
            if viewer:
                viewer.close()
        
        final_ee = self.get_ee_position(task.arm)
        error = np.linalg.norm(final_ee - task.target_position)
        
        success = error < 0.10 and not fallen
        if success:
            self.successful_tasks += 1
        
        result = TaskResult(
            success=success,
            final_position_error=error,
            min_stability_margin=min_margin,
            zmp_violations=zmp_violations,
            message="SUCCESS" if success else "FAILED"
        )
        
        print(f"\n  Result: {result.message}")
        print(f"  Final EE: {final_ee}")
        print(f"  Error: {error:.4f}m")
        print(f"  Min margin: {min_margin:.4f}m")
        print(f"  Success rate: {self.successful_tasks}/{self.task_counter}")
        
        return result


def main():
    planner = WholeBodyMotionPlanner()
    planner.reset()
    
    tasks = [
        ReachingTask(np.array([0.4, -0.15, 0.85]), 'right', 3.0, True),
        ReachingTask(np.array([0.35, -0.25, 0.75]), 'right', 3.0, True),
        ReachingTask(np.array([0.4, 0.15, 0.85]), 'left', 3.0, True),
        ReachingTask(np.array([0.35, 0.25, 0.75]), 'left', 3.0, True),
    ]
    
    print("\n" + "="*60)
    print("WHOLE-BODY MOTION PLANNING DEMO")
    print("="*60)
    input("Press ENTER to start...")
    
    for i, task in enumerate(tasks):
        print(f"\n--- Task {i+1}/{len(tasks)} ---")
        planner.execute_task(task)
        time.sleep(0.3)
        planner.reset()
    
    print("\n" + "="*60)
    print(f"Success: {planner.successful_tasks}/{planner.task_counter}")
    print("="*60)


if __name__ == "__main__":
    main()
