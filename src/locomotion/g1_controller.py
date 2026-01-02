"""
g1_controller.py
Open-Loop Walker with Dynamic Torso Bowing
"""
import numpy as np
import mujoco
import g1_config as cfg

class G1Walker:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.phase = 0.0
        
        self.joint_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) 
                          for name in cfg.JOINT_NAMES}
        self.q_targets = np.zeros(self.model.nu)

    def update(self, time):
        # 1. Warmup & Soft Start
        if time < cfg.WARMUP_TIME:
            self.phase = 0
            ramp = 0.0
        else:
            walk_time = time - cfg.WARMUP_TIME
            ramp = min(1.0, walk_time / 2.0)
            self.phase = (walk_time * 2 * np.pi * cfg.GAIT_FREQ)

        current_step_len = cfg.STEP_LENGTH * ramp
        current_step_height = cfg.STEP_HEIGHT * ramp
        
        # Ramp up the bowing action too
        current_bow_amp = cfg.HIP_PITCH_AMP * ramp

        ph_L = self.phase
        ph_R = self.phase + np.pi
        
        # 2. Compute Trajectories
        
        # --- Hip Pitch (Step + Bowing) ---
        # We add 'current_bow_amp' to lean forward during the step
        hip_pitch_L = cfg.HIP_PITCH_OFFSET + current_step_len * np.sin(ph_L)
        hip_pitch_R = cfg.HIP_PITCH_OFFSET + current_step_len * np.sin(ph_R)
        
        # --- Knee ---
        lift_L = max(0, -np.sin(ph_L + 0.5))
        lift_R = max(0, -np.sin(ph_R + 0.5))
        
        knee_L = cfg.KNEE_TARGET + lift_L * (current_step_height * 10)
        knee_R = cfg.KNEE_TARGET + lift_R * (current_step_height * 10)
        
        # --- Ankle Pitch ---
        # -0.05 bias ensures toes point down slightly to push off
        ankle_pitch_L = -(hip_pitch_L + knee_L) - 0.05 
        ankle_pitch_R = -(hip_pitch_R + knee_R) - 0.05
        
        # --- Lateral Control ---
        roll_L = cfg.HIP_ROLL_OFFSET 
        roll_R = -cfg.HIP_ROLL_OFFSET

        ankle_roll_L = -roll_L
        ankle_roll_R = -roll_R
        
        # 3. Apply Targets
        self._set_joint('left_hip_pitch_joint', hip_pitch_L)
        self._set_joint('right_hip_pitch_joint', hip_pitch_R)
        self._set_joint('left_hip_yaw_joint', 0.0)
        self._set_joint('right_hip_yaw_joint', 0.0)
        
        self._set_joint('left_knee_joint', knee_L)
        self._set_joint('right_knee_joint', knee_R)
        
        self._set_joint('left_ankle_pitch_joint', ankle_pitch_L)
        self._set_joint('right_ankle_pitch_joint', ankle_pitch_R)
        
        self._set_joint('left_hip_roll_joint', roll_L)
        self._set_joint('right_hip_roll_joint', roll_R)
        
        self._set_joint('left_ankle_roll_joint', ankle_roll_L)
        self._set_joint('right_ankle_roll_joint', ankle_roll_R)

        return self.q_targets

    def _set_joint(self, name, val):
        if name in self.joint_ids:
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator_id != -1:
                self.q_targets[actuator_id] = val