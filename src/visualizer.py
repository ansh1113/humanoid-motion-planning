"""
MuJoCo Visualization for Motion Plans

Provides real-time 3D visualization of planned trajectories.
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
from typing import Optional
from motion_planner import MotionPlan, WholeBodyPlanner
from g1_model import G1Model


class MotionVisualizer:
    """
    Visualizes motion plans in MuJoCo viewer.
    """
    
    def __init__(self, robot: G1Model):
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
    def playback_trajectory(
        self,
        plan: MotionPlan,
        arm: str,
        speed: float = 1.0,
        loop: bool = False
    ):
        """
        Play back a motion plan in the MuJoCo viewer.
        
        Uses kinematic playback (no physics simulation) to show
        the planned motion accurately.
        
        Args:
            plan: Motion plan to visualize
            arm: Which arm ('left' or 'right')
            speed: Playback speed multiplier
            loop: If True, loop the trajectory
        """
        if not plan.success:
            print(f"Cannot visualize failed plan: {plan.message}")
            return
        
        print(f"Playing trajectory: {len(plan.trajectory)} points, {plan.duration}s duration")
        print(f"Controls: Close window to exit")
        
        # Store the initial base position (floating base)
        # First 7 qpos values are: [x, y, z, qw, qx, qy, qz] for the floating base
        initial_qpos = self.robot.get_qpos().copy()
        base_pos = initial_qpos[:3].copy()
        base_quat = initial_qpos[3:7].copy()
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Set camera
            viewer.cam.distance = 2.0
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 135
            viewer.cam.lookat[:] = [0, 0, 0.8]  # Look at robot center
            
            while viewer.is_running():
                start_time = time.time()
                
                for i, point in enumerate(plan.trajectory):
                    if not viewer.is_running():
                        break
                    
                    # Reset base position to keep robot in place (kinematic playback)
                    self.data.qpos[:3] = base_pos
                    self.data.qpos[3:7] = base_quat
                    
                    # Zero out base velocities to prevent drift
                    self.data.qvel[:6] = 0
                    
                    # Set arm joints
                    self.robot.set_joint_positions(f'{arm}_arm', point.arm_joints)
                    
                    # Set waist joints
                    self.robot.set_joint_positions('waist', point.waist_joints)
                    
                    # Reset base again after set_joint_positions (which calls mj_forward)
                    self.data.qpos[:3] = base_pos
                    self.data.qpos[3:7] = base_quat
                    self.data.qvel[:6] = 0
                    
                    # Update kinematics without stepping physics
                    mujoco.mj_forward(self.model, self.data)
                    
                    # Sync viewer
                    viewer.sync()
                    
                    # Wait for real-time playback
                    elapsed = time.time() - start_time
                    target_time = point.time / speed
                    if elapsed < target_time:
                        time.sleep(target_time - elapsed)
                
                if not loop:
                    # Hold final pose
                    print("Motion complete. Close window to exit.")
                    while viewer.is_running():
                        # Keep resetting base to prevent falling
                        self.data.qpos[:3] = base_pos
                        self.data.qpos[3:7] = base_quat
                        self.data.qvel[:6] = 0
                        mujoco.mj_forward(self.model, self.data)
                        viewer.sync()
                        time.sleep(0.02)
                    break
                else:
                    # Small pause before loop restart
                    time.sleep(0.3)
                
                # Reset for next loop
                self.robot.reset()
                base_pos = self.robot.get_qpos()[:3].copy()
                base_quat = self.robot.get_qpos()[3:7].copy()
    
    def interactive_viewer(self):
        """
        Launch interactive viewer for manual exploration.
        """
        print("Launching interactive MuJoCo viewer...")
        print("Controls:")
        print("  - Mouse drag: Rotate view")
        print("  - Scroll: Zoom")
        print("  - Double-click: Select body")
        print("  - Close window to exit")
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.distance = 2.5
            viewer.cam.elevation = -15
            viewer.cam.azimuth = 135
            
            while viewer.is_running():
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.01)


def demo_motion_sequence():
    """
    Demonstrate a sequence of motions with visualization.
    """
    print("=" * 70)
    print("G1 Motion Planning Demo with MuJoCo Visualization")
    print("=" * 70)
    
    robot = G1Model()
    planner = WholeBodyPlanner(robot, safety_margin=0.015)
    viz = MotionVisualizer(robot)
    
    current_right = robot.get_end_effector_position('right_hand')
    current_left = robot.get_end_effector_position('left_hand')
    
    # Plan a sequence of motions
    print("\nPlanning motion sequence...")
    
    # Motion 1: Right arm forward
    target1 = current_right + np.array([0.10, 0.0, 0.0])
    plan1 = planner.plan_reach('right', target1, duration=1.5)
    print(f"Motion 1 (right forward): {'✓' if plan1.success else '✗'}")
    
    # Motion 2: Right arm to side
    robot.reset()
    target2 = current_right + np.array([0.0, -0.08, -0.05])
    plan2 = planner.plan_reach('right', target2, duration=1.5)
    print(f"Motion 2 (right side):    {'✓' if plan2.success else '✗'}")
    
    # Play back motions
    if plan1.success:
        print("\n--- Playing Motion 1: Right arm forward ---")
        robot.reset()
        viz.playback_trajectory(plan1, 'right', speed=1.0)
    
    if plan2.success:
        print("\n--- Playing Motion 2: Right arm to side ---")
        robot.reset()
        viz.playback_trajectory(plan2, 'right', speed=1.0)
    
    print("\nDemo complete!")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--interactive':
        # Just launch interactive viewer
        robot = G1Model()
        viz = MotionVisualizer(robot)
        viz.interactive_viewer()
    else:
        # Run demo sequence
        demo_motion_sequence()
