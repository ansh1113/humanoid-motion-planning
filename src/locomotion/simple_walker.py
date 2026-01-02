"""
G1 Walker with PD feedback on pitch
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import os


class PushWalker:
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
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(1000):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        return self.data.qpos[:3].copy()
    
    def get_pitch(self):
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        return pitch
    
    def get_pitch_rate(self):
        # Angular velocity around Y axis
        return self.data.qvel[4]
    
    def walk_control(self, t):
        ctrl = np.zeros(self.model.nu)
        
        cycle = 1.0
        phase = (t % cycle) / cycle
        theta = 2 * np.pi * phase
        
        pitch = self.get_pitch()
        pitch_rate = self.get_pitch_rate()
        
        # PD control on pitch
        # Target pitch is slightly forward (~3 degrees = 0.05 rad)
        target_pitch = 0.05
        pitch_error = pitch - target_pitch
        
        # Gains
        Kp = 0.15  # Proportional
        Kd = 0.02  # Derivative
        
        base_lean = -0.048
        lean_adjustment = -Kp * pitch_error - Kd * pitch_rate
        lean_adjustment = np.clip(lean_adjustment, -0.02, 0.02)
        
        forward_lean = base_lean + lean_adjustment
        
        ctrl[self.act['left_ankle_pitch_joint']] = forward_lean
        ctrl[self.act['right_ankle_pitch_joint']] = forward_lean
        
        hip_amp = 0.1
        ctrl[self.act['right_hip_pitch_joint']] = hip_amp * np.sin(theta)
        ctrl[self.act['left_hip_pitch_joint']] = -hip_amp * np.sin(theta)
        
        right_swing = max(0, np.sin(theta))
        left_swing = max(0, -np.sin(theta))
        
        ctrl[self.act['right_knee_joint']] = 0.2 * right_swing
        ctrl[self.act['left_knee_joint']] = 0.2 * left_swing
        
        ctrl[self.act['right_ankle_pitch_joint']] += 0.08 * right_swing
        ctrl[self.act['left_ankle_pitch_joint']] += 0.08 * left_swing
        
        roll_amp = 0.03
        ctrl[self.act['left_hip_roll_joint']] = roll_amp * np.sin(theta)
        ctrl[self.act['right_hip_roll_joint']] = roll_amp * np.sin(theta)
        
        return ctrl, pitch, forward_lean
    
    def run(self):
        print("G1 WALKER - PD Feedback")
        
        start_pos = self.reset()
        input("ENTER...")
        
        last_print = 0
        max_dist = 0
        walk_dur = 0
        
        with mujoco.viewer.launch_passive(self.model, self.data) as v:
            v.cam.distance = 3.0
            v.cam.elevation = -15
            v.cam.azimuth = 90
            
            while v.is_running():
                t = self.data.time
                
                ctrl, pitch, lean = self.walk_control(t)
                self.data.ctrl[:] = ctrl
                mujoco.mj_step(self.model, self.data)
                
                pos = self.data.qpos[:3]
                v.cam.lookat[:] = [pos[0], pos[1], 0.8]
                
                dx = pos[0] - start_pos[0]
                if pos[2] > 0.6:
                    if dx > max_dist:
                        max_dist = dx
                    walk_dur = t - 2.0
                
                if t - last_print > 1.0:
                    s = "WALK" if pos[2] > 0.6 else "FALL"
                    print(f"t={t:5.1f} | X={pos[0]:+6.2f} | d={dx:+5.2f} | pitch={np.rad2deg(pitch):+5.1f}° | lean={lean:.3f} | {s}")
                    last_print = t
                    
                    if t > 30:
                        break
                
                v.sync()
                time.sleep(max(0, self.model.opt.timestep))
        
        print(f"\nWalked: {max_dist:.2f}m in {walk_dur:.1f}s")
        if walk_dur > 0.5:
            print(f"Speed: {max_dist/walk_dur:.2f} m/s")


if __name__ == '__main__':
    PushWalker().run()
