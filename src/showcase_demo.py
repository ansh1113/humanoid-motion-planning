"""
SHOWCASE DEMO - Continuous visualization for video recording

Shows the robot performing multiple tasks in ONE MuJoCo window:
1. Walking 2m
2. Reaching 4 targets  
3. Push recovery (front, side)
4. Victory wave
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from locomotion.g1_walker import G1Walker


def run_showcase():
    print("╔" + "═"*58 + "╗")
    print("║" + " "*14 + "HUMANOID SHOWCASE DEMO" + " "*22 + "║")
    print("║" + " "*8 + "Walk → Reach → Push Recovery → Wave" + " "*14 + "║")
    print("╚" + "═"*58 + "╝")
    
    base = Path(__file__).parent.parent
    
    # ═══════════════════════════════════════════════════════════════
    # PART 1: WALKING (using RL policy)
    # ═══════════════════════════════════════════════════════════════
    print("\n[PHASE 1] Walking 2 meters...")
    
    walker = G1Walker()
    walker.reset()
    walker.set_command(forward=0.5, lateral=0.0, yaw=0.0)
    
    walk_distance = 0
    
    with mujoco.viewer.launch_passive(walker.model, walker.data) as viewer:
        viewer.cam.distance = 3.5
        viewer.cam.elevation = -15
        viewer.cam.azimuth = 135
        
        t0 = time.time()
        
        while viewer.is_running():
            state = walker.step()
            elapsed = time.time() - t0
            
            viewer.cam.lookat[:] = [state['position'][0], 0, 0.8]
            viewer.sync()
            
            if state['distance'] >= 2.0:
                walk_distance = state['distance']
                print(f"  ✓ Walked {walk_distance:.2f}m")
                break
            
            if elapsed > 10 or state['is_fallen']:
                walk_distance = state['distance']
                break
            
            time.sleep(0.001)
        
        # Stop gracefully
        print("  Stopping...")
        for i in range(100):
            walker.set_command(forward=0.5*(1-i/100), lateral=0, yaw=0)
            walker.step()
            viewer.sync()
            time.sleep(0.01)
        
        time.sleep(0.3)
    
    # ═══════════════════════════════════════════════════════════════
    # PART 2-4: MANIPULATION, PUSH RECOVERY, WAVE (29-DOF model)
    # ═══════════════════════════════════════════════════════════════
    print("\n[PHASE 2-4] Manipulation, Push Recovery, Wave...")
    
    model = mujoco.MjModel.from_xml_path(str(base / "mujoco_menagerie/unitree_g1/scene.xml"))
    data = mujoco.MjData(model)
    
    # Build actuator map
    act = {}
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name:
            act[name] = i
    
    # Find end effectors
    ee_body = {}
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name and 'wrist_yaw' in name.lower():
            ee_body['left' if 'left' in name.lower() else 'right'] = i
    
    # Find torso for push
    torso_body = None
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name and 'torso' in name.lower():
            torso_body = i
            break
    
    # Standing pose
    standing = {
        'left_hip_pitch_joint': -0.12, 'left_hip_roll_joint': 0.0, 'left_hip_yaw_joint': 0.0,
        'left_knee_joint': 0.30, 'left_ankle_pitch_joint': -0.17, 'left_ankle_roll_joint': 0.0,
        'right_hip_pitch_joint': -0.12, 'right_hip_roll_joint': 0.0, 'right_hip_yaw_joint': 0.0,
        'right_knee_joint': 0.30, 'right_ankle_pitch_joint': -0.17, 'right_ankle_roll_joint': 0.0,
        'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0, 'waist_pitch_joint': 0.0,
        'left_shoulder_pitch_joint': 0.0, 'left_shoulder_roll_joint': 0.2,
        'left_shoulder_yaw_joint': 0.0, 'left_elbow_joint': 0.3,
        'right_shoulder_pitch_joint': 0.0, 'right_shoulder_roll_joint': -0.2,
        'right_shoulder_yaw_joint': 0.0, 'right_elbow_joint': 0.3,
    }
    
    current = {n: 0.0 for n in act}
    current.update(standing)
    
    def apply():
        for n, v in current.items():
            if n in act:
                data.ctrl[act[n]] = v
    
    def get_ee(arm):
        return data.xipos[ee_body[arm]].copy()
    
    def smooth_move(start_pose, end_pose, steps, viewer):
        """Smooth interpolation between poses."""
        for step in range(steps):
            t = step / steps
            s = 10*t**3 - 15*t**4 + 6*t**5  # min-jerk
            
            for n in end_pose:
                if n in start_pose:
                    current[n] = (1-s)*start_pose[n] + s*end_pose[n]
            
            apply()
            mujoco.mj_step(model, data)
            
            if step % 10 == 0:
                viewer.sync()
        
        return data.qpos[2] >= 0.5  # Return True if still standing
    
    # Initialize
    mujoco.mj_resetData(model, data)
    for _ in range(2000):
        apply()
        mujoco.mj_step(model, data)
    
    reach_success = 0
    push_success = 0
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.8
        viewer.cam.elevation = -12
        viewer.cam.azimuth = 150
        
        # ─────────────────────────────────────────────────────────
        # PART 2: REACHING
        # ─────────────────────────────────────────────────────────
        print("\n  [Reaching targets]")
        
        reach_targets = [
            ({'right_shoulder_pitch_joint': -0.8, 'right_shoulder_roll_joint': -0.3, 
              'right_elbow_joint': 0.2}, "Right forward"),
            ({'left_shoulder_pitch_joint': -0.8, 'left_shoulder_roll_joint': 0.3, 
              'left_elbow_joint': 0.2}, "Left forward"),
            ({'right_shoulder_pitch_joint': -1.2, 'right_shoulder_roll_joint': -0.4, 
              'right_elbow_joint': 0.1}, "Right high"),
            ({'left_shoulder_pitch_joint': -1.2, 'left_shoulder_roll_joint': 0.4, 
              'left_elbow_joint': 0.1}, "Left high"),
        ]
        
        for target_pose, name in reach_targets:
            print(f"    {name}...", end=" ", flush=True)
            
            # Move to target
            end = standing.copy()
            end.update(target_pose)
            
            ok = smooth_move(current.copy(), end, 600, viewer)
            
            if ok:
                reach_success += 1
                print("✓")
            else:
                print("✗ (fell)")
                # Reset if fell
                mujoco.mj_resetData(model, data)
                for n in standing:
                    current[n] = standing[n]
                for _ in range(1500):
                    apply()
                    mujoco.mj_step(model, data)
            
            time.sleep(0.15)
            
            # Return to standing
            smooth_move(current.copy(), standing.copy(), 400, viewer)
            time.sleep(0.1)
        
        print(f"  ✓ Reaching: {reach_success}/4")
        
        # ─────────────────────────────────────────────────────────
        # PART 3: PUSH RECOVERY
        # ─────────────────────────────────────────────────────────
        print("\n  [Push recovery]")
        
        pushes = [
            ([70, 0, 0], "Front (70N)"),
            ([0, 70, 0], "Side (70N)"),
        ]
        
        for force, name in pushes:
            print(f"    {name}...", end=" ", flush=True)
            
            # Reset to standing
            mujoco.mj_resetData(model, data)
            for n in standing:
                current[n] = standing[n]
            for _ in range(1500):
                apply()
                mujoco.mj_step(model, data)
                viewer.sync()
            
            # Apply push and recover
            for step in range(1500):
                if 200 <= step < 225:
                    data.xfrc_applied[torso_body, :3] = force
                else:
                    data.xfrc_applied[torso_body, :3] = 0
                
                apply()
                mujoco.mj_step(model, data)
                
                if step % 8 == 0:
                    viewer.sync()
                
                if data.qpos[2] < 0.5:
                    break
            
            if data.qpos[2] >= 0.5:
                push_success += 1
                print("✓ Survived")
            else:
                print("✗ Fell")
            
            time.sleep(0.2)
        
        print(f"  ✓ Push recovery: {push_success}/2")
        
        # ─────────────────────────────────────────────────────────
        # PART 4: VICTORY WAVE
        # ─────────────────────────────────────────────────────────
        print("\n  [Victory wave]")
        
        # Reset
        mujoco.mj_resetData(model, data)
        for n in standing:
            current[n] = standing[n]
        for _ in range(1000):
            apply()
            mujoco.mj_step(model, data)
            viewer.sync()
        
        # Wave animation
        for cycle in range(4):
            for phase in range(80):
                angle = np.sin(phase * 0.2) * 0.6
                
                current['right_shoulder_pitch_joint'] = -1.5
                current['right_shoulder_roll_joint'] = -0.5 + angle * 0.4
                current['right_elbow_joint'] = 0.4 + abs(angle) * 0.3
                
                apply()
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.012)
        
        print("  ✓ Wave complete!")
        
        # Return to standing
        smooth_move(current.copy(), standing.copy(), 500, viewer)
        time.sleep(0.5)
    
    # ═══════════════════════════════════════════════════════════════
    # SUMMARY
    # ═══════════════════════════════════════════════════════════════
    print("\n" + "═"*58)
    print("  SHOWCASE COMPLETE!")
    print("═"*58)
    print(f"  ✓ Walking:       {walk_distance:.2f}m")
    print(f"  ✓ Reaching:      {reach_success}/4 targets")
    print(f"  ✓ Push recovery: {push_success}/2 survived")
    print(f"  ✓ Victory wave:  Done!")
    print("═"*58)


if __name__ == "__main__":
    run_showcase()
