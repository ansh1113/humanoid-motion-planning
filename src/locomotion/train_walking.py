"""
G1 Walking - Reinforcement Learning with PPO

This is the modern approach used by Boston Dynamics, Tesla, Figure, etc.

Key components:
1. Custom Gym environment for G1
2. Reward shaping for stable forward walking
3. Domain randomization for robustness
4. PPO algorithm (state of the art for continuous control)

References:
- "Learning Agile Robotic Locomotion Skills by Imitating Animals" (ETH)
- "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots" (Google)
- Tesla Optimus locomotion training
"""

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
import os
import time


class G1WalkingEnv(gym.Env):
    """
    G1 Humanoid Walking Environment
    
    Observation space:
    - Body orientation (roll, pitch, yaw) and angular velocities
    - Joint positions and velocities  
    - Previous action
    - Phase variable (for periodic gait)
    
    Action space:
    - Target joint positions for lower body
    
    Reward:
    - Forward velocity
    - Alive bonus
    - Penalties for falling, energy, jerky motion
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode=None):
        super().__init__()
        
        # Load model
        scene_path = os.path.expanduser(
            "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        
        # Build actuator mapping
        self.act_names = []
        self.act_ids = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.act_names.append(name)
                self.act_ids[name] = i
        
        # Controlled joints (lower body)
        self.controlled_joints = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
        ]
        
        # Filter to only joints that exist
        self.controlled_joints = [j for j in self.controlled_joints if j in self.act_ids]
        self.n_actions = len(self.controlled_joints)
        
        # Action and observation spaces
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.n_actions,), dtype=np.float32
        )
        
        # Observation: orientation(6) + joint_pos(n) + joint_vel(n) + phase(2) + prev_action(n)
        obs_dim = 6 + self.n_actions * 2 + 2 + self.n_actions
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        
        # Action scaling
        self.action_scale = 0.3  # Scale actions to joint limits
        
        # Simulation params
        self.dt = self.model.opt.timestep
        self.frame_skip = 4  # Control frequency
        self.max_episode_steps = 1000
        
        # Gait parameters
        self.gait_freq = 1.0  # Hz
        self.phase = 0.0
        
        # State
        self.step_count = 0
        self.prev_action = np.zeros(self.n_actions)
        self.render_mode = render_mode
        self.viewer = None
        
        # Initial pose
        self._init_qpos = None
        
        print(f"G1 Walking Environment Created")
        print(f"  Action dim: {self.n_actions}")
        print(f"  Obs dim: {obs_dim}")
        print(f"  Controlled joints: {self.controlled_joints}")
    
    def _get_obs(self):
        """Construct observation vector"""
        # Body orientation
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        # Angular velocity
        ang_vel = self.data.qvel[3:6]
        
        # Joint positions and velocities for controlled joints
        joint_pos = []
        joint_vel = []
        for name in self.controlled_joints:
            idx = self.act_ids[name]
            # Get joint qpos/qvel indices (simplified - assuming direct mapping)
            joint_pos.append(self.data.ctrl[idx])  # Use ctrl as proxy
            joint_vel.append(0.0)  # Simplified
        
        # Phase encoding (sin, cos for periodicity)
        phase_obs = [np.sin(2 * np.pi * self.phase), np.cos(2 * np.pi * self.phase)]
        
        obs = np.concatenate([
            [roll, pitch, yaw],       # 3
            ang_vel,                   # 3
            joint_pos,                 # n_actions
            joint_vel,                 # n_actions
            phase_obs,                 # 2
            self.prev_action,          # n_actions
        ]).astype(np.float32)
        
        return obs
    
    def _compute_reward(self):
        """
        Reward function for walking
        
        Components:
        1. Forward velocity reward
        2. Alive bonus
        3. Orientation penalty
        4. Energy penalty
        5. Action smoothness penalty
        """
        # Forward velocity (in world X direction)
        vel_x = self.data.qvel[0]
        target_vel = 0.5  # Target 0.5 m/s
        vel_reward = np.exp(-2.0 * (vel_x - target_vel)**2)  # Gaussian around target
        
        # Alive bonus
        alive_bonus = 0.5
        
        # Orientation penalties
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        orientation_penalty = -0.5 * (pitch**2 + roll**2)
        
        # Height penalty (don't crouch or jump)
        height = self.data.qpos[2]
        target_height = 0.79
        height_penalty = -1.0 * (height - target_height)**2
        
        # Energy penalty (minimize joint torques)
        torque = self.data.ctrl
        energy_penalty = -0.001 * np.sum(torque**2)
        
        # Action smoothness (minimize jerky motions)
        action_diff = self.data.ctrl[:self.n_actions] - self.prev_action
        smoothness_penalty = -0.01 * np.sum(action_diff**2)
        
        reward = (
            1.0 * vel_reward +
            1.0 * alive_bonus +
            0.5 * orientation_penalty +
            0.5 * height_penalty +
            0.1 * energy_penalty +
            0.1 * smoothness_penalty
        )
        
        return reward
    
    def _is_terminated(self):
        """Check if episode should terminate (fallen, etc.)"""
        height = self.data.qpos[2]
        
        # Fallen if height too low
        if height < 0.5:
            return True
        
        # Fallen if tilted too much
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        if abs(pitch) > 1.0 or abs(roll) > 1.0:  # ~57 degrees
            return True
        
        return False
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        mujoco.mj_resetData(self.model, self.data)
        
        # Small random perturbation for robustness
        if seed is not None:
            np.random.seed(seed)
        self.data.qpos[:3] += np.random.uniform(-0.01, 0.01, 3)
        
        mujoco.mj_forward(self.model, self.data)
        
        # Let robot settle
        for _ in range(100):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        
        self.step_count = 0
        self.phase = 0.0
        self.prev_action = np.zeros(self.n_actions)
        
        return self._get_obs(), {}
    
    def step(self, action):
        # Scale action
        action = np.clip(action, -1, 1) * self.action_scale
        
        # Apply action to controlled joints
        for i, name in enumerate(self.controlled_joints):
            idx = self.act_ids[name]
            self.data.ctrl[idx] = action[i]
        
        # Step simulation
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
        
        # Update phase
        self.phase += self.gait_freq * self.dt * self.frame_skip
        self.phase = self.phase % 1.0
        
        # Store action for smoothness penalty
        self.prev_action = action.copy()
        
        # Get observation and reward
        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = self._is_terminated()
        truncated = self.step_count >= self.max_episode_steps
        
        self.step_count += 1
        
        info = {
            'x_position': self.data.qpos[0],
            'x_velocity': self.data.qvel[0],
        }
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        if self.render_mode == "human":
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.sync()
    
    def close(self):
        if self.viewer is not None:
            self.viewer.close()


def make_env():
    def _init():
        return G1WalkingEnv()
    return _init


def train():
    print("=" * 60)
    print("G1 WALKING - PPO TRAINING")
    print("=" * 60)
    
    # Create vectorized environments
    n_envs = 4
    env = SubprocVecEnv([make_env() for _ in range(n_envs)])
    env = VecNormalize(env, norm_obs=True, norm_reward=True)
    
    # Create eval environment
    eval_env = DummyVecEnv([make_env()])
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)
    
    # PPO hyperparameters (tuned for locomotion)
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
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=dict(
            net_arch=dict(pi=[256, 256], vf=[256, 256])
        ),
        verbose=1,
        tensorboard_log="./logs/g1_walking/"
    )
    
    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path="./models/",
        name_prefix="g1_walk"
    )
    
    # Train
    total_timesteps = 500_000
    print(f"\nTraining for {total_timesteps:,} timesteps...")
    print("This will take a while. Press Ctrl+C to stop early.\n")
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_callback,
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    
    # Save final model
    model.save("models/g1_walk_final")
    env.save("models/g1_walk_vecnorm.pkl")
    
    print("\nTraining complete! Model saved to models/g1_walk_final")
    return model, env


def evaluate(model_path="models/g1_walk_final"):
    """Evaluate trained model with visualization"""
    print("=" * 60)
    print("G1 WALKING - EVALUATING TRAINED POLICY")
    print("=" * 60)
    
    # Load model
    env = DummyVecEnv([lambda: G1WalkingEnv(render_mode="human")])
    
    # Try to load normalization stats
    try:
        env = VecNormalize.load("models/g1_walk_vecnorm.pkl", env)
        env.training = False
        env.norm_reward = False
    except:
        print("No normalization stats found, using raw observations")
    
    model = PPO.load(model_path)
    
    obs = env.reset()
    total_reward = 0
    steps = 0
    
    print("\nRunning evaluation... Press Ctrl+C to stop\n")
    
    try:
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward[0]
            steps += 1
            
            if done[0]:
                print(f"Episode ended after {steps} steps, total reward: {total_reward:.2f}")
                print(f"Final X position: {info[0]['x_position']:.2f}m")
                obs = env.reset()
                total_reward = 0
                steps = 0
            
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nEvaluation stopped")
    
    env.close()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "eval":
        evaluate()
    else:
        model, env = train()
        print("\nStarting evaluation of trained model...")
        env.close()
        evaluate()
