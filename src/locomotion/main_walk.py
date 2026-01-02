"""
main_walk.py
Executes the G1 Walking Simulation with Safe Spawn.
"""
import mujoco
import mujoco.viewer
import time
import os
import numpy as np
import g1_controller
import g1_config as cfg

def set_initial_pose(model, data):
    """
    Sets the robot's initial joint configuration to match the controller's
    target crouch pose. This prevents the 'spawn shock' fall.
    """
    # 1. Calculate Target Angles based on Config
    q_hip_pitch = cfg.HIP_PITCH_OFFSET
    q_knee = cfg.KNEE_TARGET
    q_ankle_pitch = cfg.ANKLE_OFFSET
    q_hip_roll = cfg.HIP_ROLL_OFFSET

    # 2. Map these to the joint IDs
    # Note: We must check if the joint exists before setting
    val_map = {
        'left_hip_pitch_joint': q_hip_pitch,
        'right_hip_pitch_joint': q_hip_pitch,
        'left_knee_joint': q_knee,
        'right_knee_joint': q_knee,
        'left_ankle_pitch_joint': q_ankle_pitch,
        'right_ankle_pitch_joint': q_ankle_pitch,
        'left_hip_roll_joint': q_hip_roll,
        'right_hip_roll_joint': -q_hip_roll,
        'left_ankle_roll_joint': -q_hip_roll,
        'right_ankle_roll_joint': q_hip_roll,
    }

    for name, val in val_map.items():
        id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if id != -1:
            qpos_adr = model.jnt_qposadr[id]
            data.qpos[qpos_adr] = val

    # 3. Lower the Base Height (Z)
    # Since we are crouching, we can't spawn at 0.8m or we'll drop.
    # 0.7m is a safer bet for this crouch depth.
    data.qpos[2] = 0.73 

def quat_to_euler(quat):
    w, x, y, z = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def main():
    scene_path = os.path.expanduser(
        "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
    )
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    walker = g1_controller.G1Walker(model, data)
    
    # --- CRITICAL FIX: Set Pose BEFORE Sim Starts ---
    mujoco.mj_resetData(model, data)
    set_initial_pose(model, data)
    
    last_log_time = 0
    log_interval = 0.5

    print(f"{'TIME':<10} | {'POS X':<8} {'POS Y':<8} {'POS Z':<8} | {'ROLL':<8} {'PITCH':<8} {'YAW':<8}")
    print("-" * 80)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            current_sim_time = data.time
            
            targets = walker.update(current_sim_time)
            
            for i in range(model.nu):
                ctrl_val = targets[i]
                jnt_id = model.actuator_trnid[i, 0]
                qpos_adr = model.jnt_qposadr[jnt_id]
                qvel_adr = model.jnt_dofadr[jnt_id]
                
                current_pos = data.qpos[qpos_adr]
                current_vel = data.qvel[qvel_adr]
                
                torque = cfg.KP * (ctrl_val - current_pos) - cfg.KD * current_vel
                data.ctrl[i] = torque

            mujoco.mj_step(model, data)
            
            if time.time() - last_log_time > log_interval:
                pos_x, pos_y, pos_z = data.qpos[0:3]
                quat = data.qpos[3:7]
                r, p, y = quat_to_euler(quat)
                r_deg, p_deg, y_deg = np.rad2deg([r, p, y])
                
                print(f"{current_sim_time:10.4f} | {pos_x:8.4f} {pos_y:8.4f} {pos_z:8.4f} | {r_deg:8.2f} {p_deg:8.2f} {y_deg:8.2f}")
                last_log_time = time.time()
            
            viewer.sync()
            
            time_until_next_step = cfg.VIEWER_DT - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()