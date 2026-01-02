"""
Run Unitree's official pre-trained G1 walking policy

This uses their trained model (motion.pt) with our MuJoCo setup.
"""

import time
import mujoco
import mujoco.viewer
import numpy as np
import torch
import os


def get_gravity_orientation(quaternion):
    """Convert quaternion to gravity vector in body frame"""
    qw, qx, qy, qz = quaternion
    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """PD control for joint torques"""
    return (target_q - q) * kp + (target_dq - dq) * kd


def main():
    print("=" * 70)
    print("G1 Walking - Unitree Official Pre-trained Policy")
    print("=" * 70)
    
    # Paths
    policy_path = os.path.expanduser(
        "~/humanoid_motion_planning/unitree_rl_gym/deploy/pre_train/g1/motion.pt"
    )
    
    # We need to use THEIR robot model since the policy was trained on it
    xml_path = os.path.expanduser(
        "~/humanoid_motion_planning/unitree_rl_gym/resources/robots/g1_description/scene.xml"
    )
    
    # Check if files exist
    if not os.path.exists(policy_path):
        print(f"ERROR: Policy not found at {policy_path}")
        return
    
    if not os.path.exists(xml_path):
        print(f"Their robot model not found, using mujoco_menagerie version...")
        xml_path = os.path.expanduser(
            "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
        )
    
    # Config from g1.yaml
    simulation_duration = 60.0
    simulation_dt = 0.002
    control_decimation = 10
    
    kps = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
    kds = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)
    
    # Default standing angles
    default_angles = np.array([
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,  # Left leg
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0   # Right leg
    ], dtype=np.float32)
    
    # Scaling factors
    ang_vel_scale = 0.25
    dof_pos_scale = 1.0
    dof_vel_scale = 0.05
    action_scale = 0.25
    cmd_scale = np.array([2.0, 2.0, 0.25], dtype=np.float32)
    
    num_actions = 12
    num_obs = 47
    
    # Command: [forward_vel, lateral_vel, yaw_rate]
    cmd = np.array([1.0, 0.0, 0.0], dtype=np.float32)  # Walk forward at 0.5 m/s
    
    print(f"Policy: {policy_path}")
    print(f"Robot: {xml_path}")
    print(f"Command: forward={cmd[0]}, lateral={cmd[1]}, yaw={cmd[2]}")
    
    # Initialize
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    counter = 0
    
    # Load MuJoCo model
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    
    # Load policy
    print("Loading policy...")
    policy = torch.jit.load(policy_path, map_location='cpu')
    policy.eval()
    print("Policy loaded!")
    
    # Figure out joint mapping
    print(f"\nModel has {m.nq} qpos, {m.nv} qvel, {m.nu} actuators")
    
    # Print actuator names to understand mapping
    print("\nActuators:")
    for i in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  {i}: {name}")
    
    input("\nPress ENTER to start walking...")
    
    start_pos = d.qpos[:3].copy()
    
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.distance = 4.0
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 90
        
        start = time.time()
        last_print = 0
        
        while viewer.is_running() and time.time() - start < simulation_duration:
            step_start = time.time()
            
            # PD control to track target positions
            q = d.qpos[7:7+num_actions] if m.nq > 7+num_actions else d.qpos[7:]
            dq = d.qvel[6:6+num_actions] if m.nv > 6+num_actions else d.qvel[6:]
            
            # Ensure correct sizes
            if len(q) >= num_actions and len(dq) >= num_actions:
                tau = pd_control(target_dof_pos, q[:num_actions], kps, 
                               np.zeros_like(kds), dq[:num_actions], kds)
                d.ctrl[:num_actions] = tau
            
            mujoco.mj_step(m, d)
            
            counter += 1
            if counter % control_decimation == 0:
                # Build observation
                qj = d.qpos[7:7+num_actions] if m.nq > 7+num_actions else d.qpos[7:]
                dqj = d.qvel[6:6+num_actions] if m.nv > 6+num_actions else d.qvel[6:]
                quat = d.qpos[3:7]
                omega = d.qvel[3:6]
                
                if len(qj) >= num_actions:
                    qj_scaled = (qj[:num_actions] - default_angles) * dof_pos_scale
                    dqj_scaled = dqj[:num_actions] * dof_vel_scale
                else:
                    qj_scaled = np.zeros(num_actions)
                    dqj_scaled = np.zeros(num_actions)
                
                gravity_orientation = get_gravity_orientation(quat)
                omega_scaled = omega * ang_vel_scale
                
                # Gait phase
                period = 0.8
                count = counter * simulation_dt
                phase = count % period / period
                sin_phase = np.sin(2 * np.pi * phase)
                cos_phase = np.cos(2 * np.pi * phase)
                
                # Build obs vector (47 dims)
                obs[:3] = omega_scaled
                obs[3:6] = gravity_orientation
                obs[6:9] = cmd * cmd_scale
                obs[9:9+num_actions] = qj_scaled
                obs[9+num_actions:9+2*num_actions] = dqj_scaled
                obs[9+2*num_actions:9+3*num_actions] = action
                obs[9+3*num_actions:9+3*num_actions+2] = [sin_phase, cos_phase]
                
                # Policy inference
                with torch.no_grad():
                    obs_tensor = torch.from_numpy(obs).unsqueeze(0).float()
                    action = policy(obs_tensor).numpy().squeeze()
                
                # Convert action to joint targets
                target_dof_pos = action * action_scale + default_angles
            
            # Update camera
            pos = d.qpos[:3]
            viewer.cam.lookat[:] = [pos[0], pos[1], 0.8]
            
            # Print status
            elapsed = time.time() - start
            if elapsed - last_print > 1.0:
                dx = pos[0] - start_pos[0]
                dy = pos[1] - start_pos[1]
                height = pos[2]
                speed = dx / elapsed if elapsed > 0.5 else 0
                
                quat = d.qpos[3:7]
                w, x, y, z = quat
                pitch = np.rad2deg(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
                roll = np.rad2deg(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
                
                status = "WALK" if height > 0.5 else "FALL"
                print(f"t={elapsed:5.1f}s | X={pos[0]:+6.2f} Y={pos[1]:+6.2f} Z={height:.2f} | "
                      f"pitch={pitch:+5.1f}° roll={roll:+5.1f}° | "
                      f"d={dx:+5.2f}m | v={speed:.2f}m/s | {status}")
                last_print = elapsed
                
                if height < 0.3:
                    print("\nRobot fell!")
                    break
            
            viewer.sync()
            
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    # Final stats
    elapsed = time.time() - start
    dx = d.qpos[0] - start_pos[0]
    print("-" * 70)
    print(f"FINAL: {dx:.2f}m in {elapsed:.1f}s = {dx/elapsed:.2f} m/s")


if __name__ == "__main__":
    main()
