"""
Fresh training with proven reward (the one that got 3.8m)
"""

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
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
        
        # === REWARD (the one that worked for 3.8m) ===
        reward = 0.0
        
        # Forward velocity (main driver)
        reward += 1.5 * vel_x
        
        # Small penalty for sideways velocity
        reward -= 0.3 * abs(vel_y)
        
        # Alive bonus
        reward += 3.0
        
        # Upright bonus
        reward -= 3.0 * (pitch**2 + roll**2)
        
        # Smooth actions
        reward -= 0.01 * np.sum(action**2)
        
        # Termination
        terminated = False
        if pos[2] < 0.5 or abs(pitch) > 1.0 or abs(roll) > 1.0:
            terminated = True
            reward -= 20.0
        
        truncated = self.step_count >= self.max_steps
        
        return self._get_obs(), reward, terminated, truncated, {}


class PrintCallback(BaseCallback):
    def __init__(self, freq=25000):
        super().__init__()
        self.freq = freq
        self.best = -np.inf
        
    def _on_step(self):
        if self.n_calls % self.freq == 0 and len(self.model.ep_info_buffer) > 0:
            mean_r = np.mean([ep['r'] for ep in self.model.ep_info_buffer])
            mean_l = np.mean([ep['l'] for ep in self.model.ep_info_buffer])
            print(f"Steps: {self.n_calls:,} | Reward: {mean_r:.1f} | Ep.Length: {mean_l:.0f}")
            if mean_r > self.best:
                self.best = mean_r
                self.model.save("models/g1_walk_best")
                print("  ^ New best saved!")
        return True


def make_env():
    return G1WalkEnv()


if __name__ == "__main__":
    import sys
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 800000
    
    print("=" * 60)
    print("G1 Walking - Fresh RL Training")
    print("=" * 60)
    
    # Use multiple environments for faster training
    n_envs = 4
    env = SubprocVecEnv([make_env for _ in range(n_envs)])
    
    print(f"Using {n_envs} parallel environments")
    print(f"Training for {steps:,} steps...")
    
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        policy_kwargs=dict(net_arch=[256, 256]),  # Bigger network
        verbose=0,
        device='cpu'
    )
    
    print("-" * 60)
    
    model.learn(total_timesteps=steps, callback=PrintCallback(), progress_bar=True)
    model.save("models/g1_walk_final")
    
    print("\nDone! Saved to models/g1_walk_final")
    print("Run: python3 src/locomotion/eval_rl.py")
