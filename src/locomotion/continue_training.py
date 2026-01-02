"""Continue training with Y-drift penalty"""

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import os


class G1WalkEnv(gym.Env):
    def __init__(self):
        super().__init__()
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
        self.n_actions = len(self.ctrl_indices)
        
        self.action_space = spaces.Box(-1, 1, (self.n_actions,), dtype=np.float32)
        self.obs_dim = 6 + self.n_actions + 2
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_dim,), dtype=np.float32)
        
        self.phase = 0.0
        self.phase_freq = 1.5
        self.step_count = 0
        self.max_steps = 1000
        self.start_pos = None
        
    def _get_obs(self):
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
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        for _ in range(100):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        self.start_pos = self.data.qpos[:3].copy()
        self.phase = 0.0
        self.step_count = 0
        return self._get_obs(), {}
    
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
        self.step_count += 1
        
        pos = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        vel_x = self.data.qvel[0]
        vel_y = self.data.qvel[1]
        
        # === REWARD ===
        reward = 0.0
        
        # Forward velocity
        reward += 2.0 * vel_x
        
        # Y velocity penalty (don't drift sideways)
        reward -= 1.0 * abs(vel_y)
        
        # Y position penalty (stay on course)
        y_drift = abs(pos[1] - self.start_pos[1])
        reward -= 0.5 * y_drift
        
        # Alive bonus
        reward += 2.5
        
        # Upright
        reward -= 2.0 * (pitch**2 + roll**2)
        
        # Smooth actions
        reward -= 0.01 * np.sum(action**2)
        
        # Termination
        terminated = False
        if pos[2] < 0.5 or abs(pitch) > 1.0 or abs(roll) > 1.0:
            terminated = True
            reward -= 15.0
        
        truncated = self.step_count >= self.max_steps
        
        return self._get_obs(), reward, terminated, truncated, {}


class PrintCallback(BaseCallback):
    def __init__(self, freq=20000):
        super().__init__()
        self.freq = freq
        self.best = -np.inf
        
    def _on_step(self):
        if self.n_calls % self.freq == 0 and len(self.model.ep_info_buffer) > 0:
            mean_r = np.mean([ep['r'] for ep in self.model.ep_info_buffer])
            mean_l = np.mean([ep['l'] for ep in self.model.ep_info_buffer])
            print(f"Steps: {self.n_calls:,} | Reward: {mean_r:.1f} | Length: {mean_l:.0f}")
            if mean_r > self.best:
                self.best = mean_r
                self.model.save("models/g1_walk_best")
                print("  ^ Saved best!")
        return True


if __name__ == "__main__":
    import sys
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 300000
    
    print("=" * 60)
    print("Continue Training - With Y-Drift Penalty")
    print("=" * 60)
    
    env = DummyVecEnv([lambda: G1WalkEnv()])
    
    print("Loading models/g1_walk_final...")
    model = PPO.load("models/g1_walk_final", env=env, device='cpu')
    
    print(f"Training for {steps:,} more steps...")
    print("-" * 60)
    
    model.learn(total_timesteps=steps, callback=PrintCallback(), progress_bar=True)
    model.save("models/g1_walk_final")
    print("\nDone! Saved to models/g1_walk_final")
