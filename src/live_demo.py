"""
LIVE Manipulation Demo - Using MuJoCo spec API to add objects
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import sys
import os

sys.path.insert(0, 'src')


def create_scene_with_objects():
    """Load G1 model and add manipulation objects using MjSpec."""
    g1_path = os.path.expanduser(
        "~/humanoid_motion_planning/mujoco_menagerie/unitree_g1/g1.xml"
    )
    
    # Load the spec from XML
    spec = mujoco.MjSpec.from_file(g1_path)
    world = spec.worldbody
    
    # Add TABLE
    table = world.add_body(name="table", pos=[0.40, 0, 0.30])
    table.add_geom(
        name="table_top",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.25, 0.35, 0.015],  # BOX: half-sizes x,y,z
        rgba=[0.55, 0.35, 0.18, 1]
    )
    # Table legs - CYLINDER size is [radius, half-height, 0]
    for lx, ly in [(0.20, 0.30), (0.20, -0.30), (-0.20, 0.30), (-0.20, -0.30)]:
        table.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.02, 0.14, 0],  # radius, half-height, unused
            pos=[lx, ly, -0.14],
            rgba=[0.45, 0.28, 0.12, 1]
        )
    
    # Add TARGET (red ball) - SPHERE size is [radius, 0, 0]
    target = world.add_body(name="target_ball", pos=[0.32, -0.15, 0.35])
    target.add_geom(
        name="target",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.028, 0, 0],  # radius, unused, unused
        rgba=[1, 0.1, 0.1, 1]
    )
    
    # Add OBSTACLE 1 (green cylinder)
    obs1 = world.add_body(name="obstacle1", pos=[0.26, -0.04, 0.38])
    obs1.add_geom(
        name="obs1",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[0.035, 0.055, 0],  # radius, half-height, unused
        rgba=[0.1, 0.85, 0.15, 1]
    )
    
    # Add OBSTACLE 2 (blue box)
    obs2 = world.add_body(name="obstacle2", pos=[0.38, 0.04, 0.36])
    obs2.add_geom(
        name="obs2",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.03, 0.045, 0.045],  # half-sizes x,y,z
        rgba=[0.15, 0.35, 1, 1]
    )
    
    # Add OBSTACLE 3 (green cylinder)
    obs3 = world.add_body(name="obstacle3", pos=[0.30, 0.12, 0.37])
    obs3.add_geom(
        name="obs3",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[0.03, 0.05, 0],  # radius, half-height, unused
        rgba=[0.15, 0.8, 0.2, 1]
    )
    
    # Compile to model
    model = spec.compile()
    data = mujoco.MjData(model)
    
    return model, data


class ManipulationScene:
    """Scene with robot and manipulation objects."""
    
    def __init__(self):
        print("=" * 60)
        print("CREATING MANIPULATION SCENE")
        print("=" * 60)
        
        print("\nLoading G1 robot and adding objects...")
        self.model, self.data = create_scene_with_objects()
        print(f"  ✓ Scene created! DOF: {self.model.nq}")
        
        mujoco.mj_forward(self.model, self.data)
        
        self._setup_joints()
        self._get_scene_info()
        
        self.base_qpos = self.data.qpos[:7].copy()
    
    def _setup_joints(self):
        """Find joint indices."""
        self.joint_groups = {}
        
        groups = {
            'right_arm': ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                         'right_shoulder_yaw_joint', 'right_elbow_joint',
                         'right_wrist_roll_joint', 'right_wrist_pitch_joint',
                         'right_wrist_yaw_joint'],
            'waist': ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint'],
        }
        
        for group, names in groups.items():
            indices = []
            for name in names:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid >= 0:
                    indices.append(self.model.jnt_qposadr[jid])
            self.joint_groups[group] = indices
            print(f"  {group}: {len(indices)} joints")
        
        self.hand_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'right_wrist_yaw_rubber')
        if self.hand_id < 0:
            self.hand_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'right_wrist_yaw')
    
    def _get_scene_info(self):
        """Get object positions."""
        self.hand_pos = self.data.xpos[self.hand_id].copy()
        
        tid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'target_ball')
        self.target_pos = self.data.xpos[tid].copy()
        
        self.obstacles = []
        for name in ['obstacle1', 'obstacle2', 'obstacle3']:
            oid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if oid >= 0:
                self.obstacles.append(self.data.xpos[oid].copy())
        
        print(f"\n  Hand: [{self.hand_pos[0]:.3f}, {self.hand_pos[1]:.3f}, {self.hand_pos[2]:.3f}]")
        print(f"  Target (RED BALL): [{self.target_pos[0]:.3f}, {self.target_pos[1]:.3f}, {self.target_pos[2]:.3f}]")
        print(f"  Obstacles: {len(self.obstacles)} (GREEN/BLUE)")
        print(f"  Distance to target: {np.linalg.norm(self.target_pos - self.hand_pos)*100:.1f} cm")
    
    def get_hand_pos(self):
        return self.data.xpos[self.hand_id].copy()
    
    def set_arm(self, values):
        for i, idx in enumerate(self.joint_groups['right_arm']):
            if i < len(values):
                self.data.qpos[idx] = values[i]
    
    def set_waist(self, values):
        for i, idx in enumerate(self.joint_groups['waist']):
            if i < len(values):
                self.data.qpos[idx] = values[i]
    
    def fix_base(self):
        self.data.qpos[:7] = self.base_qpos
        self.data.qvel[:] = 0


def ik_step(scene, target, arm, alpha=0.4):
    """One IK iteration."""
    orig = scene.data.qpos.copy()
    indices = scene.joint_groups['right_arm']
    
    for i, idx in enumerate(indices):
        scene.data.qpos[idx] = arm[i]
    mujoco.mj_forward(scene.model, scene.data)
    
    hand = scene.get_hand_pos()
    error = target - hand
    
    if np.linalg.norm(error) < 0.01:
        scene.data.qpos[:] = orig
        return arm, True
    
    J = np.zeros((3, len(indices)))
    eps = 0.001
    for i, idx in enumerate(indices):
        scene.data.qpos[idx] = arm[i] + eps
        mujoco.mj_forward(scene.model, scene.data)
        p1 = scene.get_hand_pos()
        scene.data.qpos[idx] = arm[i] - eps
        mujoco.mj_forward(scene.model, scene.data)
        p2 = scene.get_hand_pos()
        J[:, i] = (p1 - p2) / (2 * eps)
        scene.data.qpos[idx] = arm[i]
    
    JJT = J @ J.T + 0.05 * np.eye(3)
    delta = J.T @ np.linalg.solve(JJT, error)
    arm_new = np.clip(arm + alpha * delta, -2.5, 2.5)
    
    scene.data.qpos[:] = orig
    return arm_new, False


def plan_trajectory(scene):
    """Plan trajectory with obstacle avoidance."""
    print("\n" + "=" * 60)
    print("PLANNING COLLISION-FREE PATH")
    print("=" * 60)
    
    start = scene.hand_pos.copy()
    goal = scene.target_pos.copy()
    goal[2] += 0.025
    
    waypoints = [
        start.copy(),
        start + np.array([0.0, 0.0, 0.10]),
        (start + goal)/2 + np.array([0, 0, 0.10]),
        goal + np.array([0.0, 0.0, 0.06]),
        goal
    ]
    
    print(f"  Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        print(f"    {i}: z={wp[2]:.3f}")
    
    print("  Computing IK...")
    trajectory, waist_traj = [], []
    arm = np.zeros(7)
    
    for wp in waypoints:
        for _ in range(80):
            arm, done = ik_step(scene, wp, arm)
            if done:
                break
        
        if trajectory:
            prev = trajectory[-1]
            for t in np.linspace(0.05, 1.0, 18):
                interp = (1-t)*prev + t*arm
                trajectory.append(interp)
                pitch = np.clip(-1.0 * interp[0] * 0.5, -0.18, 0)
                waist_traj.append(np.array([0, 0, pitch]))
        else:
            trajectory.append(arm.copy())
            waist_traj.append(np.zeros(3))
    
    max_waist = max(abs(np.rad2deg(w[2])) for w in waist_traj)
    print(f"  ✓ Trajectory: {len(trajectory)} points")
    print(f"  ✓ Max waist compensation: {max_waist:.1f}°")
    
    return trajectory, waist_traj


def run_demo():
    """Run live demo."""
    scene = ManipulationScene()
    trajectory, waist_traj = plan_trajectory(scene)
    
    print("\n" + "=" * 60)
    print("STARTING LIVE VIEWER")
    print("=" * 60)
    print("""
┌─────────────────────────────────────────┐
│  SCENE CONTENTS:                        │
│    • BROWN TABLE in front of robot      │
│    • RED BALL = Target to reach         │
│    • GREEN CYLINDERS = Obstacles        │
│    • BLUE BOX = Obstacle                │
│                                         │
│  ROBOT WILL:                            │
│    1. Go UP (clear obstacles)           │
│    2. Go FORWARD (over obstacles)       │
│    3. Go DOWN (to red ball)             │
│    4. Lean BACK (for balance)           │
└─────────────────────────────────────────┘

Press ENTER to start...""")
    input()
    
    state = {'phase': 'wait', 'idx': 0, 'time': 0, 'loop': 0}
    
    def controller(model, data):
        scene.fix_base()
        t = data.time
        
        if state['phase'] == 'wait':
            if t - state['time'] > 1.0:
                state['phase'] = 'forward'
                state['idx'] = 0
                state['loop'] += 1
                print(f"\n[Loop {state['loop']}] Reaching for RED BALL...")
        
        elif state['phase'] == 'forward':
            idx = state['idx']
            if idx < len(trajectory):
                scene.set_arm(trajectory[idx])
                scene.set_waist(waist_traj[idx])
                state['idx'] += 1
                
                if idx % 20 == 0:
                    pct = idx / len(trajectory)
                    phase = "UP" if pct < 0.25 else "OVER" if pct < 0.6 else "DOWN"
                    waist_deg = np.rad2deg(waist_traj[idx][2])
                    print(f"  {pct*100:5.1f}% [{phase:5s}] waist: {waist_deg:.1f}°")
            else:
                state['phase'] = 'hold'
                state['time'] = t
                print("  >>> REACHED RED BALL! <<<")
        
        elif state['phase'] == 'hold':
            scene.set_arm(trajectory[-1])
            scene.set_waist(waist_traj[-1])
            if t - state['time'] > 2.0:
                state['phase'] = 'return'
                state['idx'] = len(trajectory) - 1
                print("  Returning...")
        
        elif state['phase'] == 'return':
            if state['idx'] >= 0:
                scene.set_arm(trajectory[state['idx']])
                scene.set_waist(waist_traj[state['idx']])
                state['idx'] -= 3
            else:
                state['phase'] = 'wait'
                state['time'] = t
        
        scene.fix_base()
    
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        viewer.cam.lookat[:] = [0.28, -0.05, 0.45]
        viewer.cam.distance = 1.3
        viewer.cam.azimuth = 150
        viewer.cam.elevation = -25
        
        while viewer.is_running():
            t0 = time.time()
            controller(scene.model, scene.data)
            mujoco.mj_forward(scene.model, scene.data)
            scene.data.time += scene.model.opt.timestep
            viewer.sync()
            dt = scene.model.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    
    print("\nDone!")


if __name__ == '__main__':
    run_demo()
