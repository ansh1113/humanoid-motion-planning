"""Evaluate trained RL walking policy - with full position logging"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import os
from stable_baselines3 import PPO


class G1WalkEnv:
    def __init__(self):
        scene_path = os.path.expanduser(
            "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        
        self.act_ids = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.act_ids[name] = i
        
        self.controlled = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
        ]
        self.ctrl_indices = [self.act_ids[n] for n in self.controlled if n in self.act_ids]
        
        self.phase = 0.0
        self.phase_freq = 1.5
        self.start_pos = np.zeros(3)
        
    def get_obs(self):
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        return np.concatenate([
            [pitch, roll],
            self.data.qvel[3:5],
            self.data.qvel[0:2],
            [self.data.ctrl[i] for i in self.ctrl_indices],
            [np.sin(2*np.pi*self.phase), np.cos(2*np.pi*self.phase)]
        ]).astype(np.float32)
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        for _ in range(100):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        self.start_pos = self.data.qpos[:3].copy()
        self.phase = 0.0
        return self.get_obs()
    
    def step(self, action):
        action = np.clip(action, -1, 1)
        
        base_ctrl = np.zeros(self.model.nu)
        if 'left_shoulder_roll_joint' in self.act_ids:
            base_ctrl[self.act_ids['left_shoulder_roll_joint']] = 0.3
        if 'right_shoulder_roll_joint' in self.act_ids:
            base_ctrl[self.act_ids['right_shoulder_roll_joint']] = -0.3
        
        for i, idx in enumerate(self.ctrl_indices):
            base_ctrl[idx] = action[i] * 0.3
        
        for _ in range(4):
            self.data.ctrl[:] = base_ctrl
            mujoco.mj_step(self.model, self.data)
        
        self.phase += self.phase_freq * 4 * self.model.opt.timestep
        self.phase = self.phase % 1.0
        
        pos = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        terminated = pos[2] < 0.5 or abs(pitch) > 1.2 or abs(roll) > 1.2
        
        return self.get_obs(), terminated, pos, pitch, roll


def evaluate(model_path="models/g1_walk_final", duration=30.0):
    print("=" * 70)
    print("G1 Walking - RL Policy Evaluation")
    print("=" * 70)
    
    env = G1WalkEnv()
    model = PPO.load(model_path)
    
    print(f"Loaded: {model_path}")
    input("ENTER to start...")
    
    obs = env.reset()
    start_time = env.data.time
    start_pos = env.start_pos.copy()
    max_dist = 0
    walk_time = 0
    
    with mujoco.viewer.launch_passive(env.model, env.data) as v:
        v.cam.distance = 4.0
        v.cam.elevation = -20
        v.cam.azimuth = 90
        
        last_print = 0
        
        while v.is_running():
            action, _ = model.predict(obs, deterministic=True)
            obs, terminated, pos, pitch, roll = env.step(action)
            
            elapsed = env.data.time - start_time
            
            v.cam.lookat[:] = [pos[0], pos[1], 0.8]
            
            dx = pos[0] - start_pos[0]
            dy = pos[1] - start_pos[1]
            
            if pos[2] > 0.5 and dx > max_dist:
                max_dist = dx
                walk_time = elapsed
            
            if elapsed - last_print > 1.0:
                status = "WALK" if pos[2] > 0.5 else "FALL"
                speed = dx / elapsed if elapsed > 0.5 else 0
                print(f"t={elapsed:5.1f}s | X={pos[0]:+6.2f} Y={pos[1]:+6.2f} Z={pos[2]:.2f} | "
                      f"pitch={np.rad2deg(pitch):+5.1f}° roll={np.rad2deg(roll):+5.1f}° | "
                      f"d={dx:+5.2f}m | v={speed:.2f}m/s | {status}")
                last_print = elapsed
            
            if terminated or elapsed > duration:
                break
            
            v.sync()
            time.sleep(0.005)
    
    print("-" * 70)
    print(f"FINAL: {max_dist:.2f}m forward in {walk_time:.1f}s")
    if walk_time > 0.5:
        print(f"SPEED: {max_dist/walk_time:.2f} m/s")
    print(f"Y drift: {dy:+.2f}m")


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "models/g1_walk_final"
    evaluate(path)
