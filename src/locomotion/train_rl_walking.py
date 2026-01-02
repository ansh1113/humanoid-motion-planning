"""
G1 Walking - RL Training with better rewards for stability
"""

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
        self.max_steps = 1000  # Longer episodes
        self.start_pos = None
        
    def _get_obs(self):
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        obs = np.concatenate([
            [pitch, roll],
            self.data.qvel[3:5],
            self.data.qvel[0:2],
            [self.data.ctrl[i] for i in self.ctrl_indices],
            [np.sin(2*np.pi*self.phase), np.cos(2*np.pi*self.phase)]
        ]).astype(np.float32)
        return obs
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        for _ in range(100):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        self.start_pos = self.data.qpos[0]
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
        
        # === IMPROVED REWARD ===
        # Target velocity (not too fast)
        target_vel = 0.3
        reward_vel = 1.0 - 2.0 * abs(vel_x - target_vel)  # Peak at target_vel
        reward_vel = max(reward_vel, 0.0)
        
        # Forward progress bonus
        reward_forward = 1.0 * vel_x if vel_x > 0 else 2.0 * vel_x  # Penalize backward more
        
        # BIG alive bonus (encourages staying up)
        reward_alive = 2.0
        
        # Strong upright bonus
        pitch_penalty = 5.0 * pitch**2
        roll_penalty = 5.0 * roll**2
        reward_upright = 1.0 - pitch_penalty - roll_penalty
        
        # Height bonus (stay at proper height)
        target_height = 0.78
        height_error = abs(pos[2] - target_height)
        reward_height = 0.5 * (1.0 - height_error * 2)
        
        # Smooth action penalty
        reward_energy = -0.02 * np.sum(action**2)
        
        reward = reward_forward + reward_vel + reward_alive + reward_upright + reward_height + reward_energy
        
        # Termination
        terminated = False
        if pos[2] < 0.5:
            terminated = True
            reward -= 10.0  # Big penalty for falling
        if abs(pitch) > 1.0 or abs(roll) > 1.0:
            terminated = True
            reward -= 10.0
        
        truncated = self.step_count >= self.max_steps
        
        obs = self._get_obs()
        info = {'x_pos': pos[0], 'vel_x': vel_x, 'distance': pos[0] - self.start_pos}
        
        return obs, reward, terminated, truncated, info


class ProgressCallback(BaseCallback):
    def __init__(self, check_freq=10000):
        super().__init__()
        self.check_freq = check_freq
        self.best_mean_reward = -np.inf
        
    def _on_step(self):
        if self.n_calls % self.check_freq == 0:
            if len(self.model.ep_info_buffer) > 0:
                rewards = [ep['r'] for ep in self.model.ep_info_buffer]
                lengths = [ep['l'] for ep in self.model.ep_info_buffer]
                mean_r = np.mean(rewards)
                mean_l = np.mean(lengths)
                print(f"Steps: {self.n_calls:,} | Reward: {mean_r:.1f} | Episode length: {mean_l:.0f}")
                
                if mean_r > self.best_mean_reward:
                    self.best_mean_reward = mean_r
                    self.model.save("models/g1_walk_best")
        return True


def train(total_timesteps=200000, continue_training=True):
    print("=" * 60)
    print("G1 Walking - PPO Training (Stability Focused)")
    print("=" * 60)
    
    os.makedirs("models", exist_ok=True)
    env = DummyVecEnv([lambda: G1WalkEnv()])
    
    # Try to load existing model
    if continue_training and os.path.exists("models/g1_walk_best.zip"):
        print("Loading existing model and continuing training...")
        model = PPO.load("models/g1_walk_best", env=env, device='cpu')
    else:
        print("Creating new model...")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.005,
            policy_kwargs=dict(net_arch=[128, 128]),
            verbose=0,
            device='cpu'
        )
    
    print(f"Training for {total_timesteps:,} steps...")
    print("-" * 60)
    
    callback = ProgressCallback(check_freq=10000)
    
    try:
        model.learn(total_timesteps=total_timesteps, callback=callback, progress_bar=True)
    except KeyboardInterrupt:
        print("\nTraining interrupted!")
    
    model.save("models/g1_walk_final")
    print(f"\nModel saved!")


if __name__ == "__main__":
    import sys
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 200000
    train(steps)
