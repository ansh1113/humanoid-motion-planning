"""
Video Recorder for MuJoCo Simulations

Records motion plans as MP4 videos with multiple camera angles.
"""

import numpy as np
import mujoco
import imageio
from typing import List, Optional


class VideoRecorder:
    """Records MuJoCo simulations to video files."""
    
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        width: int = 640,
        height: int = 480
    ):
        self.model = model
        self.data = data
        self.width = width
        self.height = height
        
        # Create renderer with safe dimensions
        self.renderer = mujoco.Renderer(model, height, width)
        
        # Camera settings
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    
    def set_camera(
        self,
        lookat: np.ndarray = None,
        distance: float = 2.5,
        azimuth: float = 135,
        elevation: float = -20
    ):
        """Configure camera view."""
        if lookat is not None:
            self.camera.lookat[:] = lookat
        else:
            self.camera.lookat[:] = [0, 0, 0.8]
        
        self.camera.distance = distance
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
    
    def capture_frame(self) -> np.ndarray:
        """Capture a single frame."""
        self.renderer.update_scene(self.data, self.camera)
        return self.renderer.render()
    
    def record_trajectory(
        self,
        trajectory_func,
        duration: float,
        fps: int = 30,
        output_path: str = "output.mp4",
        camera_motion: str = "static"
    ) -> str:
        """
        Record a trajectory to video.
        
        Args:
            trajectory_func: Function that takes (time) and sets robot state
            duration: Total duration in seconds
            fps: Frames per second
            output_path: Output video path
            camera_motion: "static", "orbit", or "follow"
            
        Returns:
            Path to saved video
        """
        n_frames = int(duration * fps)
        frames = []
        
        initial_azimuth = self.camera.azimuth
        
        print(f"Recording {n_frames} frames at {fps} FPS...")
        
        for i in range(n_frames):
            t = i / fps
            
            # Update robot state
            trajectory_func(t)
            mujoco.mj_forward(self.model, self.data)
            
            # Update camera for motion effects
            if camera_motion == "orbit":
                self.camera.azimuth = initial_azimuth + 60 * (t / duration)
            elif camera_motion == "follow":
                com = self.data.subtree_com[1].copy()
                self.camera.lookat[:] = com
            
            # Capture frame
            frame = self.capture_frame()
            frames.append(frame)
            
            if (i + 1) % max(1, n_frames // 10) == 0:
                print(f"  {100 * (i + 1) // n_frames}% complete")
        
        # Save video
        print(f"Saving video to {output_path}...")
        imageio.mimsave(output_path, frames, fps=fps)
        print(f"✓ Video saved: {output_path}")
        
        return output_path


def record_waist_compensation_demo():
    """Record a video specifically showing waist compensation."""
    import sys
    sys.path.insert(0, 'src')
    
    from g1_model import G1Model
    from motion_planner import WholeBodyPlanner
    
    print("=" * 60)
    print("Recording Waist Compensation Demo")
    print("=" * 60)
    
    robot = G1Model()
    planner = WholeBodyPlanner(robot, safety_margin=0.030)
    recorder = VideoRecorder(robot.model, robot.data)
    
    right_hand = robot.get_end_effector_position('right_hand')
    
    # Plan aggressive forward reach
    print("\nPlanning aggressive forward reach...")
    target = right_hand + np.array([0.12, 0.0, 0.0])
    plan = planner.plan_reach('right', target, duration=2.5)
    
    if not plan.success:
        print(f"Planning failed: {plan.message}")
        return
    
    max_pitch = max(abs(np.rad2deg(p.waist_joints[2])) for p in plan.trajectory)
    print(f"  ✓ Plan found with {max_pitch:.1f}° waist pitch compensation")
    
    # Store base position
    base_qpos = robot.data.qpos[:7].copy()
    
    # Total duration: forward + pause + return
    total_duration = plan.duration * 2 + 0.5
    
    def trajectory_func(t):
        # Keep base fixed
        robot.data.qpos[:7] = base_qpos
        robot.data.qvel[:6] = 0
        
        # Forward motion
        if t < plan.duration:
            idx = int(t / plan.duration * (len(plan.trajectory) - 1))
            idx = min(idx, len(plan.trajectory) - 1)
            point = plan.trajectory[idx]
        # Pause
        elif t < plan.duration + 0.5:
            point = plan.trajectory[-1]
        # Return
        else:
            return_t = t - plan.duration - 0.5
            idx = len(plan.trajectory) - 1 - int(return_t / plan.duration * (len(plan.trajectory) - 1))
            idx = max(0, idx)
            point = plan.trajectory[idx]
        
        robot.set_joint_positions('right_arm', point.arm_joints)
        robot.set_joint_positions('waist', point.waist_joints)
        
        # Re-fix base
        robot.data.qpos[:7] = base_qpos
        robot.data.qvel[:6] = 0
    
    # Side view (best for seeing waist pitch)
    print("\nRecording side view...")
    recorder.set_camera(
        lookat=np.array([0.1, 0, 0.85]),
        distance=2.0,
        azimuth=90,
        elevation=-10
    )
    
    recorder.record_trajectory(
        trajectory_func,
        duration=total_duration,
        fps=30,
        output_path="waist_compensation_side.mp4"
    )
    
    # Front-side view
    print("\nRecording front-side view...")
    robot.reset()
    base_qpos = robot.data.qpos[:7].copy()
    
    recorder.set_camera(
        lookat=np.array([0.05, 0, 0.85]),
        distance=2.2,
        azimuth=150,
        elevation=-15
    )
    
    recorder.record_trajectory(
        trajectory_func,
        duration=total_duration,
        fps=30,
        output_path="waist_compensation_front.mp4"
    )
    
    print("\n" + "=" * 60)
    print("Videos saved!")
    print("=" * 60)


def record_multi_motion_demo():
    """Record multiple arm motions."""
    import sys
    sys.path.insert(0, 'src')
    
    from g1_model import G1Model
    from motion_planner import WholeBodyPlanner
    
    print("=" * 60)
    print("Recording Multi-Motion Demo")
    print("=" * 60)
    
    robot = G1Model()
    planner = WholeBodyPlanner(robot, safety_margin=0.025)
    recorder = VideoRecorder(robot.model, robot.data)
    
    right_hand = robot.get_end_effector_position('right_hand')
    
    # Plan motions
    motions = []
    
    print("\nPlanning motions...")
    
    # Forward
    plan1 = planner.plan_reach('right', right_hand + np.array([0.10, 0.0, 0.0]), duration=1.5)
    if plan1.success:
        motions.append(plan1)
        print("  ✓ Forward reach")
    robot.reset()
    
    # Lateral
    plan2 = planner.plan_reach('right', right_hand + np.array([0.0, -0.10, 0.0]), duration=1.5)
    if plan2.success:
        motions.append(plan2)
        print("  ✓ Lateral reach")
    robot.reset()
    
    # Up
    plan3 = planner.plan_reach('right', right_hand + np.array([0.03, 0.0, 0.08]), duration=1.5)
    if plan3.success:
        motions.append(plan3)
        print("  ✓ Upward reach")
    robot.reset()
    
    base_qpos = robot.data.qpos[:7].copy()
    
    # State for trajectory function
    state = {'motion_idx': 0, 'motion_start': 0.0, 'returning': False}
    pause_duration = 0.3
    
    def get_total_duration():
        # Each motion: forward + pause + return + pause
        return sum((p.duration * 2 + pause_duration * 2) for p in motions)
    
    total_duration = get_total_duration()
    
    def trajectory_func(t):
        robot.data.qpos[:7] = base_qpos
        robot.data.qvel[:6] = 0
        
        # Find which motion we're in
        elapsed = 0
        for i, plan in enumerate(motions):
            motion_total = plan.duration * 2 + pause_duration * 2
            if t < elapsed + motion_total:
                local_t = t - elapsed
                
                # Forward
                if local_t < plan.duration:
                    idx = int(local_t / plan.duration * (len(plan.trajectory) - 1))
                    point = plan.trajectory[min(idx, len(plan.trajectory) - 1)]
                # Pause at end
                elif local_t < plan.duration + pause_duration:
                    point = plan.trajectory[-1]
                # Return
                elif local_t < plan.duration * 2 + pause_duration:
                    return_t = local_t - plan.duration - pause_duration
                    idx = len(plan.trajectory) - 1 - int(return_t / plan.duration * (len(plan.trajectory) - 1))
                    point = plan.trajectory[max(0, idx)]
                # Pause at start
                else:
                    point = plan.trajectory[0]
                
                robot.set_joint_positions('right_arm', point.arm_joints)
                robot.set_joint_positions('waist', point.waist_joints)
                break
            elapsed += motion_total
        
        robot.data.qpos[:7] = base_qpos
        robot.data.qvel[:6] = 0
    
    # Record with orbit
    print(f"\nRecording {total_duration:.1f}s video with orbiting camera...")
    recorder.set_camera(
        lookat=np.array([0.0, 0, 0.85]),
        distance=2.3,
        azimuth=135,
        elevation=-15
    )
    
    recorder.record_trajectory(
        trajectory_func,
        duration=total_duration,
        fps=30,
        output_path="multi_motion_demo.mp4",
        camera_motion="orbit"
    )
    
    print("\n" + "=" * 60)
    print("Video saved: multi_motion_demo.mp4")
    print("=" * 60)


if __name__ == '__main__':
    record_waist_compensation_demo()
    print("\n")
    record_multi_motion_demo()
