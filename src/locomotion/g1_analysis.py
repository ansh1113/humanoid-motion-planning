"""
Step 1: Analyze the G1 robot to understand its structure
"""

import numpy as np
import mujoco
import os


def analyze_g1():
    """Analyze the G1 robot model"""
    
    scene_path = os.path.expanduser(
        "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
    )
    
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    
    print("=" * 70)
    print("G1 ROBOT ANALYSIS")
    print("=" * 70)
    
    # Basic info
    print(f"\nModel: {model.nq} DOF, {model.nu} actuators, {model.nbody} bodies")
    
    # Analyze joints
    print("\n" + "=" * 70)
    print("JOINTS")
    print("=" * 70)
    
    leg_joints = []
    
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name is None:
            continue
            
        jnt_type = model.jnt_type[i]
        qpos_adr = model.jnt_qposadr[i]
        
        # Get joint limits
        limited = model.jnt_limited[i]
        if limited:
            range_low = model.jnt_range[i, 0]
            range_high = model.jnt_range[i, 1]
            range_str = f"[{np.rad2deg(range_low):+7.1f}°, {np.rad2deg(range_high):+7.1f}°]"
        else:
            range_str = "unlimited"
        
        # Get axis
        axis = model.jnt_axis[i]
        
        if 'hip' in name or 'knee' in name or 'ankle' in name:
            leg_joints.append({
                'name': name,
                'qpos_adr': qpos_adr,
                'axis': axis.copy(),
                'range': (range_low, range_high) if limited else None
            })
            print(f"  {name:30s} axis={axis} range={range_str}")
    
    # Analyze body masses
    print("\n" + "=" * 70)
    print("BODY MASSES (for CoM calculation)")
    print("=" * 70)
    
    total_mass = 0
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        mass = model.body_mass[i]
        total_mass += mass
        if mass > 0.1:  # Only show significant masses
            print(f"  {name:30s} {mass:6.2f} kg")
    
    print(f"\n  TOTAL MASS: {total_mass:.2f} kg")
    
    # Analyze actuators
    print("\n" + "=" * 70)
    print("LEG ACTUATORS")
    print("=" * 70)
    
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name and ('hip' in name or 'knee' in name or 'ankle' in name):
            # Get control range
            ctrl_limited = model.actuator_ctrllimited[i]
            if ctrl_limited:
                ctrl_range = model.actuator_ctrlrange[i]
                print(f"  {name:30s} ctrl=[{ctrl_range[0]:+.2f}, {ctrl_range[1]:+.2f}]")
            else:
                print(f"  {name:30s} ctrl=unlimited")
    
    # Initialize and get standing pose info
    print("\n" + "=" * 70)
    print("STANDING CONFIGURATION")
    print("=" * 70)
    
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    # Get foot positions
    for foot_name in ['left_ankle_roll', 'right_ankle_roll']:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_name)
        if bid >= 0:
            pos = data.xpos[bid]
            print(f"  {foot_name:20s} pos=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}]")
    
    # Get pelvis position
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
    if pelvis_id >= 0:
        pelvis_pos = data.xpos[pelvis_id]
        print(f"  {'pelvis':20s} pos=[{pelvis_pos[0]:+.3f}, {pelvis_pos[1]:+.3f}, {pelvis_pos[2]:+.3f}]")
    
    # Compute CoM
    com = np.zeros(3)
    for i in range(model.nbody):
        com += model.body_mass[i] * data.xpos[i]
    com /= total_mass
    print(f"  {'CoM':20s} pos=[{com[0]:+.3f}, {com[1]:+.3f}, {com[2]:+.3f}]")
    
    # Get foot geometry
    print("\n" + "=" * 70)
    print("FOOT GEOMETRY")
    print("=" * 70)
    
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name and ('foot' in name.lower() or 'ankle' in name.lower()):
            geom_type = model.geom_type[geom_id]
            geom_size = model.geom_size[geom_id]
            print(f"  {name:30s} type={geom_type} size={geom_size}")
    
    print("\n" + "=" * 70)
    print("KEY OBSERVATIONS FOR WALKING")
    print("=" * 70)
    print("""
    1. Hip pitch axis is Y (0,1,0) - positive rotation moves leg BACKWARD
    2. Need to shift weight laterally before lifting swing foot
    3. Total mass ~35kg, CoM height ~0.75m
    4. Feet are ~0.2m apart laterally
    
    Walking strategy:
    - Use hip roll to shift CoM over stance foot
    - Swing leg forward using hip pitch (negative = forward)
    - Bend knee during swing for clearance
    - Land foot flat, transfer weight
    """)
    
    return model, data, leg_joints


if __name__ == '__main__':
    analyze_g1()
