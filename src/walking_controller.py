"""
G1 Walking - The stance leg must PUSH BACKWARD to move forward
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import os


def main():
    scene_path = os.path.expanduser(
        "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/scene.xml"
    )
    
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    
    actuators = {}
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name:
            actuators[name] = i
    
    print("G1 Walking")
    
    mujoco.mj_resetData(model, data)
    for _ in range(2000):
        mujoco.mj_step(model, data)
    
    x0 = data.qpos[0]
    print(f"Start X={x0:.3f}")
    print("ENTER to walk...")
    input()
    
    last_t = 0
    cycle = 1.0
    
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.distance = 2.5
        v.cam.elevation = -15
        v.cam.azimuth = 90
        
        while v.is_running():
            t = data.time
            phase = (t % cycle) / cycle * 2 * np.pi
            
            # Reset all
            data.ctrl[:] = 0
            
            # Walking:
            # To walk FORWARD, the stance leg must extend/push BACKWARD
            # Hip pitch positive = leg goes backward
            # So stance leg needs POSITIVE hip pitch
            # And swing leg needs NEGATIVE hip pitch (to swing forward)
            
            # Phase 0: Right leg back (stance, pushing), Left leg forward (swing)
            # Phase pi: Left leg back (stance), Right leg forward (swing)
            
            hip_amp = 0.25
            knee_amp = 0.35
            ankle_amp = 0.1
            roll_amp = 0.05
            
            # Right hip: starts positive (back/stance), goes negative (forward/swing)
            data.ctrl[actuators['right_hip_pitch_joint']] = hip_amp * np.cos(phase)
            # Left hip: opposite
            data.ctrl[actuators['left_hip_pitch_joint']] = -hip_amp * np.cos(phase)
            
            # Knee bends during swing (when hip pitch is negative/forward)
            right_swing = max(0, -np.cos(phase))
            left_swing = max(0, np.cos(phase))
            
            data.ctrl[actuators['right_knee_joint']] = knee_amp * right_swing
            data.ctrl[actuators['left_knee_joint']] = knee_amp * left_swing
            
            # Ankle dorsiflexion during swing
            data.ctrl[actuators['right_ankle_pitch_joint']] = ankle_amp * right_swing
            data.ctrl[actuators['left_ankle_pitch_joint']] = ankle_amp * left_swing
            
            # Lateral weight shift
            # When right is stance (cos>0), shift weight right (positive roll)
            data.ctrl[actuators['left_hip_roll_joint']] = roll_amp * np.cos(phase)
            data.ctrl[actuators['right_hip_roll_joint']] = roll_amp * np.cos(phase)
            
            # Compensate at ankle
            data.ctrl[actuators['left_ankle_roll_joint']] = -roll_amp * 0.3 * np.cos(phase)
            data.ctrl[actuators['right_ankle_roll_joint']] = -roll_amp * 0.3 * np.cos(phase)
            
            # Arm swing
            data.ctrl[actuators['right_shoulder_pitch_joint']] = 0.2 * np.cos(phase)
            data.ctrl[actuators['left_shoulder_pitch_joint']] = -0.2 * np.cos(phase)
            
            mujoco.mj_step(model, data)
            
            p = data.qpos[:3]
            v.cam.lookat[:] = [p[0], p[1], 0.8]
            
            if t - last_t > 0.5:
                dx = p[0] - x0
                s = "OK" if p[2] > 0.5 else "FELL"
                print(f"t={t:5.1f} | X={p[0]:+.3f} | Z={p[2]:.3f} | dist={dx:+.3f} | {s}")
                last_t = t
            
            v.sync()
            time.sleep(max(0, model.opt.timestep - 0.0001))


if __name__ == '__main__':
    main()
