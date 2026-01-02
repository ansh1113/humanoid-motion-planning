"""
G1 Walking - Reduced amplitude for sustainability

The 0.64m version has pitch run away at ~8 degrees.
Let's reduce amplitude to keep pitch around 5-6 degrees - 
slightly slower but hopefully sustainable.
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import os


class WalkingController:
    def __init__(self):
        scene_path = os.path.expanduser(
            "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        
        self.act = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.act[name] = i
        
        self.step_time = 0.45
        self.cycle_time = 2 * self.step_time
        self.start_pos = None
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(2000):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        self.start_pos = self.data.qpos[:3].copy()
        return self.start_pos.copy()
    
    def get_body_state(self):
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch_rate = self.data.qvel[4]
        roll_rate = self.data.qvel[3]
        return pitch, roll, pitch_rate, roll_rate
    
    def control(self, t):
        ctrl = np.zeros(self.model.nu)
        
        pitch, roll, pitch_rate, roll_rate = self.get_body_state()
        com_pos = self.data.qpos[:3]
        
        phase = (t % self.cycle_time) / self.cycle_time
        right_swing = phase < 0.5
        swing_phase = (phase % 0.5) / 0.5
        
        s = swing_phase
        s_smooth = 3*s**2 - 2*s**3
        swing_height = 4 * s * (1 - s)
        
        # REDUCED from 0.10/0.06 to 0.08/0.05
        swing_forward = 0.08
        stance_push = 0.05
        knee_lift = 0.20
        base_lean = -0.043  # Slightly less lean
        
        target_pitch = 0.045  # Lower target (~2.5 degrees)
        Kp_pitch = 0.5
        Kd_pitch = 0.10
        ankle_pitch_fb = Kp_pitch * (pitch - target_pitch) + Kd_pitch * pitch_rate
        ankle_pitch_fb = np.clip(ankle_pitch_fb, -0.05, 0.07)
        
        Kp_roll = 0.8
        Kd_roll = 0.15
        ankle_roll_fb = Kp_roll * roll + Kd_roll * roll_rate
        ankle_roll_fb = np.clip(ankle_roll_fb, -0.07, 0.07)
        
        y_error = com_pos[1] - self.start_pos[1]
        y_correction = np.clip(0.3 * y_error, -0.04, 0.04)
        
        if right_swing:
            hip_roll = 0.025
            right_hip_pitch = stance_push - (swing_forward + stance_push) * s_smooth
            right_knee = knee_lift * swing_height
            left_hip_pitch = stance_push * (1 + s_smooth * 0.5)
            left_knee = 0.02
        else:
            hip_roll = -0.025
            left_hip_pitch = stance_push - (swing_forward + stance_push) * s_smooth
            left_knee = knee_lift * swing_height
            right_hip_pitch = stance_push * (1 + s_smooth * 0.5)
            right_knee = 0.02
        
        ctrl[self.act['left_hip_pitch_joint']] = left_hip_pitch
        ctrl[self.act['right_hip_pitch_joint']] = right_hip_pitch
        ctrl[self.act['left_knee_joint']] = left_knee
        ctrl[self.act['right_knee_joint']] = right_knee
        ctrl[self.act['left_hip_roll_joint']] = hip_roll
        ctrl[self.act['right_hip_roll_joint']] = hip_roll
        ctrl[self.act['left_ankle_pitch_joint']] = base_lean + ankle_pitch_fb
        ctrl[self.act['right_ankle_pitch_joint']] = base_lean + ankle_pitch_fb
        ctrl[self.act['left_ankle_roll_joint']] = -ankle_roll_fb - y_correction
        ctrl[self.act['right_ankle_roll_joint']] = ankle_roll_fb + y_correction
        ctrl[self.act['left_shoulder_roll_joint']] = 0.25
        ctrl[self.act['right_shoulder_roll_joint']] = -0.25
        
        return ctrl, pitch, roll
    
    def run(self, duration=30.0):
        print("=" * 50)
        print("G1 WALKING - Reduced Amplitude")
        print("=" * 50)
        
        start_pos = self.reset()
        input("ENTER...")
        
        start_time = self.data.time
        last_print = start_time
        max_dist = 0
        walk_time = 0
        
        with mujoco.viewer.launch_passive(self.model, self.data) as v:
            v.cam.distance = 3.0
            v.cam.elevation = -15
            v.cam.azimuth = 90
            
            while v.is_running():
                t = self.data.time
                elapsed = t - start_time
                
                ctrl, pitch, roll = self.control(elapsed)
                self.data.ctrl[:] = ctrl
                mujoco.mj_step(self.model, self.data)
                
                pos = self.data.qpos[:3]
                v.cam.lookat[:] = [pos[0], pos[1], 0.8]
                
                dx = pos[0] - start_pos[0]
                height = pos[2]
                
                if height > 0.6 and dx > max_dist:
                    max_dist = dx
                    walk_time = elapsed
                
                if t - last_print > 1.0:
                    status = "WALK" if height > 0.6 else "FALL"
                    speed = dx / elapsed if elapsed > 0.5 else 0
                    print(f"t={elapsed:5.1f}s | X={pos[0]:+.3f} | pitch={np.rad2deg(pitch):+5.1f}° | d={dx:+.2f}m | v={speed:.2f}m/s | {status}")
                    last_print = t
                    
                    if elapsed > duration or (height < 0.4 and elapsed > 3):
                        break
                
                v.sync()
                time.sleep(max(0, self.model.opt.timestep))
        
        print("-" * 50)
        if walk_time > 0.5:
            print(f"BEST: {max_dist:.2f}m in {walk_time:.1f}s = {max_dist/walk_time:.2f} m/s")


if __name__ == '__main__':
    controller = WalkingController()
    controller.run()
