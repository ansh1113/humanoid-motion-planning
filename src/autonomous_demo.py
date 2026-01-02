"""
AUTONOMOUS Manipulation - FIXED VERSION
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import sys
import os

sys.path.insert(0, 'src')


class AutonomousRobot:
    def __init__(self):
        print("=" * 60)
        print("AUTONOMOUS ROBOT SYSTEM")
        print("=" * 60)
        
        # Create scene
        print("\n[1] Creating scene...")
        self._create_scene()
        
        # Setup joint indices
        self._setup_joints()
        
        # Initialize planner using original G1Model (for planning only)
        print("\n[2] Initializing planner...")
        from g1_model import G1Model
        from motion_planner import WholeBodyPlanner
        
        self.planning_robot = G1Model()
        self.planner = WholeBodyPlanner(self.planning_robot, safety_margin=0.025)
        print("    ✓ WholeBodyPlanner ready")
        
        # Get positions
        mujoco.mj_forward(self.model, self.data)
        self.base_qpos = self.data.qpos[:7].copy()
        
        hand = self._get_hand_pos()
        print(f"\n[3] Scene info:")
        print(f"    Hand: [{hand[0]:.3f}, {hand[1]:.3f}, {hand[2]:.3f}]")
        
        tid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'target')
        self.target_pos = self.data.xpos[tid].copy()
        print(f"    Target: [{self.target_pos[0]:.3f}, {self.target_pos[1]:.3f}, {self.target_pos[2]:.3f}]")
        print(f"    Distance: {np.linalg.norm(self.target_pos - hand)*100:.1f} cm")
    
    def _create_scene(self):
        """Create scene with robot and objects."""
        g1_path = os.path.expanduser(
            "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/g1.xml"
        )
        spec = mujoco.MjSpec.from_file(g1_path)
        
        # Table - in front of robot
        table = spec.worldbody.add_body(name="table", pos=[0.38, 0, 0.30])
        table.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[0.18, 0.28, 0.012], rgba=[0.6, 0.4, 0.2, 1])
        for lx, ly in [(0.14, 0.24), (0.14, -0.24), (-0.14, 0.24), (-0.14, -0.24)]:
            table.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                          size=[0.012, 0.14, 0], pos=[lx, ly, -0.14],
                          rgba=[0.5, 0.3, 0.15, 1])
        
        # Target RED ball - MUST be reachable (within ~10cm of default hand position)
        # Default hand is at approximately [0.20, -0.15, 0.89]
        target = spec.worldbody.add_body(name="target", pos=[0.28, -0.20, 0.87])
        target.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE,
                       size=[0.022, 0, 0], rgba=[1, 0.1, 0.1, 1])
        
        # Obstacles
        obs1 = spec.worldbody.add_body(name="obstacle1", pos=[0.24, -0.16, 0.88])
        obs1.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                     size=[0.02, 0.035, 0], rgba=[0.1, 0.85, 0.15, 1])
        
        obs2 = spec.worldbody.add_body(name="obstacle2", pos=[0.25, -0.10, 0.86])
        obs2.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX,
                     size=[0.018, 0.025, 0.025], rgba=[0.15, 0.4, 1, 1])
        
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
    
    def _setup_joints(self):
        """Setup joint indices."""
        self.arm_indices = []
        self.waist_indices = []
        
        arm_names = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                    'right_shoulder_yaw_joint', 'right_elbow_joint',
                    'right_wrist_roll_joint', 'right_wrist_pitch_joint',
                    'right_wrist_yaw_joint']
        waist_names = ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
        
        for name in arm_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.arm_indices.append(self.model.jnt_qposadr[jid])
        
        for name in waist_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.waist_indices.append(self.model.jnt_qposadr[jid])
        
        # Find hand body
        self.hand_id = -1
        for name in ['right_wrist_yaw_rubber', 'right_wrist_yaw', 'right_hand']:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                self.hand_id = bid
                break
        
        print(f"    Arm joints: {len(self.arm_indices)}")
        print(f"    Waist joints: {len(self.waist_indices)}")
        print(f"    Hand body ID: {self.hand_id}")
    
    def _get_hand_pos(self):
        """Get hand position."""
        if self.hand_id >= 0:
            return self.data.xpos[self.hand_id].copy()
        return np.array([0.2, -0.15, 0.89])  # Default
    
    def _set_arm(self, values):
        for i, idx in enumerate(self.arm_indices):
            if i < len(values):
                self.data.qpos[idx] = values[i]
    
    def _set_waist(self, values):
        for i, idx in enumerate(self.waist_indices):
            if i < len(values):
                self.data.qpos[idx] = values[i]
    
    def _fix_base(self):
        self.data.qpos[:7] = self.base_qpos
        self.data.qvel[:] = 0
    
    def plan_reach(self, target_pos):
        """Plan a reach using our WholeBodyPlanner."""
        # Reset planning robot
        self.planning_robot.reset()
        mujoco.mj_forward(self.planning_robot.model, self.planning_robot.data)
        
        # Plan
        plan = self.planner.plan_reach('right', target_pos, duration=2.0)
        return plan
    
    def run(self):
        """Run the demo."""
        print("\n" + "=" * 60)
        print("AUTONOMOUS OPERATION")  
        print("=" * 60)
        
        # PERCEIVE
        print("\n[PERCEIVE] Scanning scene...")
        hand = self._get_hand_pos()
        target = self.target_pos.copy()
        dist = np.linalg.norm(target - hand)
        print(f"    Found target at distance: {dist*100:.1f}cm")
        
        # DECIDE
        print("\n[DECIDE] Analyzing...")
        if dist > 0.25:
            print("    Target too far, cannot reach")
            return
        print("    Decision: REACH for target")
        
        # PLAN
        print("\n[PLAN] Computing motion...")
        goal = target.copy()
        goal[2] += 0.01
        
        plan = self.plan_reach(goal)
        
        if not plan.success:
            print(f"    Planning failed: {plan.message}")
            return
        
        trajectory = plan.trajectory
        max_waist = max(abs(np.rad2deg(p.waist_joints[2])) for p in trajectory)
        min_margin = plan.min_stability_margin * 1000
        
        print(f"    ✓ Trajectory: {len(trajectory)} points")
        print(f"    ✓ Max waist compensation: {max_waist:.1f}°")
        print(f"    ✓ Min ZMP margin: {min_margin:.0f}mm")
        
        # EXECUTE
        print("\n[EXECUTE] Starting motion...")
        print("Press ENTER to begin...")
        input()
        
        idx = 0
        phase = 'forward'
        hold_start = 0
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.lookat[:] = [0.22, -0.15, 0.85]
            viewer.cam.distance = 1.1
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -12
            
            print("\n    [Executing reach...]")
            
            while viewer.is_running():
                t0 = time.time()
                self._fix_base()
                
                if phase == 'forward':
                    if idx < len(trajectory):
                        pt = trajectory[idx]
                        self._set_arm(pt.arm_joints)
                        self._set_waist(pt.waist_joints)
                        
                        if idx % 20 == 0:
                            pct = 100 * idx / len(trajectory)
                            w = np.rad2deg(pt.waist_joints[2])
                            m = pt.stability_margin * 1000
                            print(f"    {pct:5.1f}% | waist: {w:5.1f}° | ZMP margin: {m:.0f}mm")
                        
                        idx += 1
                    else:
                        print("    >>> TARGET REACHED! <<<")
                        phase = 'hold'
                        hold_start = self.data.time
                
                elif phase == 'hold':
                    pt = trajectory[-1]
                    self._set_arm(pt.arm_joints)
                    self._set_waist(pt.waist_joints)
                    
                    if self.data.time - hold_start > 2.0:
                        print("    [Returning to home...]")
                        phase = 'return'
                        idx = len(trajectory) - 1
                
                elif phase == 'return':
                    if idx >= 0:
                        pt = trajectory[idx]
                        self._set_arm(pt.arm_joints)
                        self._set_waist(pt.waist_joints)
                        idx -= 2
                    else:
                        print("    [Home position. Loop repeating...]")
                        phase = 'forward'
                        idx = 0
                
                self._fix_base()
                mujoco.mj_forward(self.model, self.data)
                self.data.time += self.model.opt.timestep
                viewer.sync()
                
                dt = self.model.opt.timestep - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
        
        print("\nDemo ended.")


if __name__ == '__main__':
    AutonomousRobot().run()
