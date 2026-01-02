"""
WORKING DEMO - Shows the system working as designed
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import sys

sys.path.insert(0, 'src')

from g1_model import G1Model
from motion_planner import WholeBodyPlanner


def main():
    print("=" * 60)
    print("HUMANOID MOTION PLANNING SYSTEM")
    print("=" * 60)
    
    robot = G1Model()
    
    # Use TIGHT safety margin to trigger more compensation
    planner = WholeBodyPlanner(robot, safety_margin=0.035)
    
    mujoco.mj_forward(robot.model, robot.data)
    base_qpos = robot.data.qpos[:7].copy()
    
    hand = robot.get_end_effector_position('right_hand')
    
    print(f"\nHand position: {hand}")
    
    # Plan reaches of increasing difficulty
    targets = [
        ("Small reach", hand + np.array([0.06, 0.0, 0.0])),
        ("Medium reach", hand + np.array([0.10, 0.0, 0.0])),
        ("Far reach", hand + np.array([0.14, 0.0, -0.05])),
        ("Extended reach", hand + np.array([0.15, -0.05, -0.08])),
    ]
    
    print("\n[Planning motions...]")
    plans = []
    
    for name, target in targets:
        robot.reset()
        mujoco.mj_forward(robot.model, robot.data)
        
        plan = planner.plan_reach('right', target, duration=2.0)
        
        if plan.success:
            max_waist = max(abs(np.rad2deg(p.waist_joints[2])) for p in plan.trajectory)
            min_margin = plan.min_stability_margin * 1000
            print(f"\n  {name}:")
            print(f"    Distance: {np.linalg.norm(target - hand)*100:.1f}cm")
            print(f"    ✓ Waist compensation: {max_waist:.1f}°")
            print(f"    ✓ Min ZMP margin: {min_margin:.0f}mm")
            plans.append((name, plan, max_waist))
        else:
            print(f"\n  {name}: FAILED - {plan.message}")
    
    if not plans:
        print("\nNo valid plans!")
        return
    
    # Sort by waist compensation
    plans.sort(key=lambda x: x[2])
    
    print(f"\n{'='*60}")
    print(f"Running {len(plans)} demos")
    print("Press ENTER...")
    input()
    
    idx = 0
    traj_idx = 0
    phase = 'wait'
    phase_time = 0
    
    def fix():
        robot.data.qpos[:7] = base_qpos
        robot.data.qvel[:] = 0
    
    with mujoco.viewer.launch_passive(robot.model, robot.data) as v:
        v.cam.lookat[:] = [0.1, 0, 0.9]
        v.cam.distance = 1.8
        v.cam.azimuth = 90
        v.cam.elevation = -5
        
        print("\n[SIDE VIEW]\n")
        
        while v.is_running():
            t0 = time.time()
            t = robot.data.time
            fix()
            
            name, plan, max_w = plans[idx]
            traj = plan.trajectory
            
            if phase == 'wait':
                if t - phase_time > 1.0:
                    phase = 'go'
                    traj_idx = 0
                    print(f"\n>>> {name} (waist: {max_w:.1f}°)")
            
            elif phase == 'go':
                if traj_idx < len(traj):
                    pt = traj[traj_idx]
                    robot.set_joint_positions('right_arm', pt.arm_joints)
                    robot.set_joint_positions('waist', pt.waist_joints)
                    
                    if traj_idx % 20 == 0:
                        w = np.rad2deg(pt.waist_joints[2])
                        m = pt.stability_margin * 1000
                        print(f"    {100*traj_idx//len(traj):3d}% | waist: {w:5.1f}° | margin: {m:.0f}mm")
                    
                    traj_idx += 1
                else:
                    print(f"    REACHED!")
                    phase = 'hold'
                    phase_time = t
            
            elif phase == 'hold':
                pt = traj[-1]
                robot.set_joint_positions('right_arm', pt.arm_joints)
                robot.set_joint_positions('waist', pt.waist_joints)
                if t - phase_time > 2.0:
                    phase = 'back'
                    traj_idx = len(traj) - 1
            
            elif phase == 'back':
                if traj_idx >= 0:
                    pt = traj[traj_idx]
                    robot.set_joint_positions('right_arm', pt.arm_joints)
                    robot.set_joint_positions('waist', pt.waist_joints)
                    traj_idx -= 2
                else:
                    idx = (idx + 1) % len(plans)
                    phase = 'wait'
                    phase_time = t
            
            fix()
            mujoco.mj_forward(robot.model, robot.data)
            robot.data.time += robot.model.opt.timestep
            v.sync()
            
            dt = robot.model.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == '__main__':
    main()
