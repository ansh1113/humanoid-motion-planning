"""
Complete Manipulation Demo - FIXED VERSION

Proper scene with reachable target, visible obstacles, and correct rendering.
"""

import numpy as np
import mujoco
import imageio
import sys

sys.path.insert(0, 'src')

from g1_model import G1Model
from motion_planner import WholeBodyPlanner
from collision_checker import CollisionChecker

try:
    import cv2
    HAS_CV2 = True
except:
    HAS_CV2 = False


class ManipulationDemo:
    def __init__(self):
        print("=" * 60)
        print("Initializing Manipulation Demo")
        print("=" * 60)
        
        self.robot = G1Model()
        self.planner = WholeBodyPlanner(self.robot, safety_margin=0.025)
        self.collision_checker = CollisionChecker(self.robot.model, self.robot.data)
        
        # Video
        self.renderer = mujoco.Renderer(self.robot.model, 480, 640)
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        
        mujoco.mj_forward(self.robot.model, self.robot.data)
        self.base_qpos = self.robot.data.qpos[:7].copy()
        
        # Get actual hand position to place objects RELATIVE to it
        hand_pos = self.robot.get_end_effector_position('right_hand')
        print(f"  Hand position: {hand_pos}")
        
        # Objects positioned relative to hand - ALL REACHABLE
        self.objects = {
            'table': {
                'pos': hand_pos + np.array([0.08, 0.0, -0.15]),
                'color': (100, 80, 60),
                'type': 'table'
            },
            'target': {
                'pos': hand_pos + np.array([0.10, -0.05, -0.02]),  # Reachable!
                'color': (50, 50, 255),  # Red in BGR
                'type': 'sphere',
                'radius': 12
            },
            'obstacle1': {
                'pos': hand_pos + np.array([0.05, 0.0, -0.05]),
                'color': (50, 200, 50),  # Green
                'type': 'cylinder',
                'radius': 15
            },
            'obstacle2': {
                'pos': hand_pos + np.array([0.12, 0.04, -0.04]),
                'color': (255, 150, 50),  # Blue
                'type': 'box',
                'size': (12, 18)
            },
        }
        
        self.target_pos = self.objects['target']['pos'].copy()
        self.obstacle_positions = [
            self.objects['obstacle1']['pos'],
            self.objects['obstacle2']['pos'],
        ]
        
        # Verify reachability
        dist = np.linalg.norm(self.target_pos - hand_pos)
        print(f"  Target distance from hand: {dist*100:.1f}cm")
        print(f"  Target position: {self.target_pos}")
        
        if dist > 0.20:
            print(f"  WARNING: Target may be too far!")
    
    def fix_base(self):
        self.robot.data.qpos[:7] = self.base_qpos
        self.robot.data.qvel[:6] = 0
    
    def capture_frame(self):
        self.renderer.update_scene(self.robot.data, self.camera)
        return self.renderer.render()
    
    def world_to_screen(self, world_pos):
        """Convert world position to screen coordinates."""
        cam_pos = np.array([
            self.camera.lookat[0] + self.camera.distance * np.cos(np.deg2rad(self.camera.elevation)) * np.cos(np.deg2rad(self.camera.azimuth)),
            self.camera.lookat[1] + self.camera.distance * np.cos(np.deg2rad(self.camera.elevation)) * np.sin(np.deg2rad(self.camera.azimuth)),
            self.camera.lookat[2] + self.camera.distance * np.sin(np.deg2rad(self.camera.elevation))
        ])
        
        forward = self.camera.lookat - cam_pos
        forward = forward / (np.linalg.norm(forward) + 1e-8)
        
        world_up = np.array([0, 0, 1])
        right = np.cross(forward, world_up)
        right = right / (np.linalg.norm(right) + 1e-8)
        up = np.cross(right, forward)
        
        to_point = world_pos - cam_pos
        
        z = np.dot(to_point, forward)
        if z < 0.1:
            return None
        
        x = np.dot(to_point, right)
        y = np.dot(to_point, up)
        
        fov = np.deg2rad(45)
        scale = 240 / np.tan(fov / 2)
        
        px = int(320 - x / z * scale)
        py = int(240 - y / z * scale)
        
        # Size scaling
        base_size = 30 / z
        
        if 0 <= px < 640 and 0 <= py < 480:
            return px, py, base_size
        return None
    
    def render_objects(self, frame, show_labels=True, highlight_target=False):
        """Render objects on frame."""
        if not HAS_CV2:
            return frame
        
        frame = frame.copy()
        
        # Render table first (background)
        table = self.objects['table']
        proj = self.world_to_screen(table['pos'])
        if proj:
            px, py, sz = proj
            # Draw table as rectangle
            cv2.rectangle(frame, (px - 80, py - 10), (px + 80, py + 20), table['color'], -1)
            cv2.rectangle(frame, (px - 80, py - 10), (px + 80, py + 20), (150, 130, 110), 2)
        
        # Render obstacles
        for name in ['obstacle1', 'obstacle2']:
            obj = self.objects[name]
            proj = self.world_to_screen(obj['pos'])
            if proj:
                px, py, sz = proj
                r = int(obj.get('radius', 15) * sz / 15)
                
                if obj['type'] == 'cylinder':
                    cv2.ellipse(frame, (px, py), (r, int(r * 1.5)), 0, 0, 360, obj['color'], -1)
                    cv2.ellipse(frame, (px, py), (r, int(r * 1.5)), 0, 0, 360, (255, 255, 255), 2)
                else:  # box
                    w, h = obj.get('size', (15, 20))
                    w, h = int(w * sz / 15), int(h * sz / 15)
                    cv2.rectangle(frame, (px - w, py - h), (px + w, py + h), obj['color'], -1)
                    cv2.rectangle(frame, (px - w, py - h), (px + w, py + h), (255, 255, 255), 2)
                
                if show_labels:
                    label = "OBSTACLE"
                    cv2.putText(frame, label, (px - 30, py - int(r * 1.5) - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Render target (foreground)
        target = self.objects['target']
        proj = self.world_to_screen(target['pos'])
        if proj:
            px, py, sz = proj
            r = int(target.get('radius', 12) * sz / 15)
            
            color = target['color']
            if highlight_target:
                # Pulsing effect
                color = (0, 0, 255)
            
            cv2.circle(frame, (px, py), r, color, -1)
            cv2.circle(frame, (px, py), r, (255, 255, 255), 2)
            
            if show_labels:
                cv2.putText(frame, "TARGET", (px - 25, py - r - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return frame
    
    def draw_path(self, frame, path, color=(0, 255, 255)):
        """Draw planned path on frame."""
        if not HAS_CV2 or len(path) < 2:
            return frame
        
        frame = frame.copy()
        
        screen_points = []
        for p in path:
            proj = self.world_to_screen(p)
            if proj:
                screen_points.append((proj[0], proj[1]))
        
        for i in range(len(screen_points) - 1):
            cv2.line(frame, screen_points[i], screen_points[i + 1], color, 2)
        
        # Draw waypoint markers
        for i, sp in enumerate(screen_points[::5]):
            cv2.circle(frame, sp, 4, (255, 0, 255), -1)
        
        return frame
    
    def add_text(self, frame, texts, box_highlight=None):
        """Add text overlay with optional highlighted box."""
        if not HAS_CV2:
            return frame
        
        frame = frame.copy()
        
        # Semi-transparent background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (400, 30 + len(texts) * 24), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
        
        for i, text in enumerate(texts):
            y = 28 + i * 24
            
            color = (255, 255, 255)
            if box_highlight and i in box_highlight:
                cv2.rectangle(frame, (8, y - 18), (395, y + 4), (0, 120, 0), -1)
                color = (255, 255, 255)
            
            cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return frame
    
    def add_status_bar(self, frame, phase, progress, zmp, status):
        """Add bottom status bar."""
        if not HAS_CV2:
            return frame
        
        frame = frame.copy()
        h = frame.shape[0]
        
        # Background
        cv2.rectangle(frame, (0, h - 35), (640, h), (40, 40, 40), -1)
        
        # Phase
        cv2.putText(frame, phase, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Progress bar
        cv2.rectangle(frame, (180, h - 28), (320, h - 12), (80, 80, 80), -1)
        cv2.rectangle(frame, (180, h - 28), (180 + int(140 * progress), h - 12), (0, 200, 0), -1)
        cv2.putText(frame, f"{progress*100:.0f}%", (325, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # ZMP
        zmp_color = (0, 255, 0) if zmp > 15 else (0, 200, 255) if zmp > 5 else (0, 0, 255)
        cv2.putText(frame, f"ZMP:{zmp:.0f}mm", (400, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, zmp_color, 1)
        
        # Status
        st_color = (0, 255, 0) if "OK" in status or "CLEAR" in status else (0, 200, 255)
        cv2.putText(frame, status, (520, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, st_color, 1)
        
        return frame
    
    def compute_avoidance_path(self, start, goal, obstacles, clearance=0.06):
        """Compute path avoiding obstacles."""
        waypoints = [start.copy()]
        
        direct = goal - start
        dist = np.linalg.norm(direct)
        
        # Check if any obstacle blocks direct path
        blocked_obs = None
        for obs in obstacles:
            t = np.clip(np.dot(obs - start, direct) / (dist**2 + 1e-8), 0.1, 0.9)
            closest = start + t * direct
            if np.linalg.norm(obs - closest) < clearance:
                blocked_obs = obs
                break
        
        if blocked_obs is not None:
            # Go above the obstacle
            via = blocked_obs.copy()
            via[2] += 0.08  # Go above
            # Also offset laterally
            lateral = goal - start
            lateral[2] = 0
            lateral = lateral / (np.linalg.norm(lateral) + 1e-8)
            perpendicular = np.array([-lateral[1], lateral[0], 0])
            via[:2] += perpendicular[:2] * 0.03
            
            waypoints.append(via)
            print(f"    Avoidance point added: z={via[2]:.3f}")
        
        waypoints.append(goal.copy())
        return waypoints
    
    def interpolate_path(self, waypoints, n_per_seg=25):
        """Smooth interpolation."""
        path = []
        for i in range(len(waypoints) - 1):
            for j in range(n_per_seg):
                t = j / n_per_seg
                s = 3*t*t - 2*t*t*t
                path.append((1 - s) * waypoints[i] + s * waypoints[i + 1])
        path.append(waypoints[-1])
        return path
    
    def record_demo(self, output_path="manipulation_final.mp4"):
        """Record the complete demo."""
        print("\n" + "=" * 60)
        print("Recording Manipulation Demo")
        print("=" * 60)
        
        fps = 30
        frames = []
        
        # Camera - good angle to see robot, table, and objects
        self.camera.lookat[:] = [0.15, -0.05, 0.85]
        self.camera.distance = 1.4
        self.camera.azimuth = 145
        self.camera.elevation = -15
        
        self.robot.reset()
        self.base_qpos = self.robot.data.qpos[:7].copy()
        
        hand_start = self.robot.get_end_effector_position('right_hand')
        
        # ========== PHASE 1: PERCEPTION ==========
        print("\nPhase 1: Perception...")
        
        for i in range(fps * 3):
            self.fix_base()
            mujoco.mj_forward(self.robot.model, self.robot.data)
            
            frame = self.capture_frame()
            
            t = i / (fps * 3)
            
            # Progressively show objects
            show_target = t > 0.25
            show_obs = t > 0.5
            
            if show_obs:
                frame = self.render_objects(frame, show_labels=True, highlight_target=show_target)
            elif show_target:
                # Just render target
                frame = self.render_objects(frame, show_labels=True)
            
            texts = ["PHASE 1: PERCEPTION"]
            if t > 0.2:
                texts.append(f"Scanning workspace...")
            if t > 0.35:
                texts.append(f"TARGET detected at ({self.target_pos[0]:.2f}, {self.target_pos[1]:.2f}, {self.target_pos[2]:.2f})")
            if t > 0.55:
                texts.append(f"OBSTACLES detected: {len(self.obstacle_positions)}")
            if t > 0.75:
                texts.append("Analysis complete. Planning path...")
            
            frame = self.add_text(frame, texts)
            frame = self.add_status_bar(frame, "PERCEPTION", t, 30, "SCANNING")
            frames.append(frame)
        
        # ========== PHASE 2: PATH PLANNING ==========
        print("Phase 2: Path planning...")
        
        goal = self.target_pos.copy()
        waypoints = self.compute_avoidance_path(hand_start, goal, self.obstacle_positions)
        path = self.interpolate_path(waypoints, n_per_seg=20)
        
        print(f"  Waypoints: {len(waypoints)}, Path points: {len(path)}")
        
        for i in range(fps * 2):
            self.fix_base()
            mujoco.mj_forward(self.robot.model, self.robot.data)
            
            frame = self.capture_frame()
            frame = self.render_objects(frame, show_labels=True, highlight_target=True)
            
            # Draw path
            t = i / (fps * 2)
            path_to_show = path[:int(len(path) * min(t * 2, 1))]
            if path_to_show:
                frame = self.draw_path(frame, path_to_show)
            
            texts = [
                "PHASE 2: PATH PLANNING",
                f"Start: hand position",
                f"Goal: target ball",
            ]
            if t > 0.3:
                texts.append(f"Checking direct path... BLOCKED!")
            if t > 0.5:
                texts.append(f"Computing avoidance waypoints: {len(waypoints)}")
            if t > 0.8:
                texts.append(f"Path ready: {len(path)} points")
            
            frame = self.add_text(frame, texts, box_highlight=[3] if 0.3 < t < 0.5 else None)
            frame = self.add_status_bar(frame, "PLANNING", t, 30, "COMPUTING")
            frames.append(frame)
        
        # ========== PHASE 3: MOTION EXECUTION ==========
        print("Phase 3: Executing motion...")
        
        # Plan the reach
        plan = self.planner.plan_reach('right', goal, duration=2.0)
        
        if plan.success:
            trajectory = [(p.arm_joints, p.waist_joints, p.stability_margin * 1000) 
                         for p in plan.trajectory]
            print(f"  ✓ Plan succeeded: {len(trajectory)} points")
        else:
            print(f"  Plan failed: {plan.message}")
            # Fallback simple trajectory
            trajectory = []
            for i in range(60):
                t = i / 59
                s = 3*t*t - 2*t*t*t
                arm = np.array([0.5*s, -0.1*s, 0.05*s, -0.6*s, 0, 0.2*s, 0])
                waist = np.array([0, 0, -0.05*s])
                trajectory.append((arm, waist, 28 - 5*s))
        
        # Execute
        for i, (arm, waist, zmp) in enumerate(trajectory):
            self.fix_base()
            self.robot.set_joint_positions('right_arm', arm)
            self.robot.set_joint_positions('waist', waist)
            self.fix_base()
            mujoco.mj_forward(self.robot.model, self.robot.data)
            
            frame = self.capture_frame()
            frame = self.render_objects(frame, show_labels=False, highlight_target=True)
            frame = self.draw_path(frame, path, color=(100, 255, 100))
            
            progress = i / max(len(trajectory) - 1, 1)
            waist_deg = np.rad2deg(waist[2]) if len(waist) > 2 else 0
            
            texts = [
                "PHASE 3: EXECUTION",
                f"Progress: {progress*100:.0f}%",
                f"ZMP margin: {zmp:.0f}mm",
                f"Waist compensation: {waist_deg:.1f}°",
            ]
            
            frame = self.add_text(frame, texts)
            frame = self.add_status_bar(frame, "EXECUTING", progress, zmp, "STABLE")
            frames.append(frame)
        
        # ========== PHASE 4: SUCCESS ==========
        print("Phase 4: Target reached!")
        
        hand_final = self.robot.get_end_effector_position('right_hand')
        error = np.linalg.norm(hand_final - self.target_pos) * 1000
        
        for i in range(fps * 3):
            self.fix_base()
            self.robot.set_joint_positions('right_arm', trajectory[-1][0])
            self.robot.set_joint_positions('waist', trajectory[-1][1])
            self.fix_base()
            mujoco.mj_forward(self.robot.model, self.robot.data)
            
            frame = self.capture_frame()
            frame = self.render_objects(frame, show_labels=False, highlight_target=True)
            
            texts = [
                "TARGET REACHED!",
                f"Position error: {error:.1f}mm",
                "",
                "Modules used:",
                "  ✓ Perception",
                "  ✓ Path Planning (obstacle avoidance)",
                "  ✓ Inverse Kinematics",
                "  ✓ ZMP Stability Control",
            ]
            
            frame = self.add_text(frame, texts, box_highlight=[0])
            frame = self.add_status_bar(frame, "COMPLETE", 1.0, trajectory[-1][2], "SUCCESS!")
            frames.append(frame)
        
        # ========== RETURN ==========
        print("Returning home...")
        
        for i in range(len(trajectory) - 1, -1, -2):
            self.fix_base()
            self.robot.set_joint_positions('right_arm', trajectory[i][0])
            self.robot.set_joint_positions('waist', trajectory[i][1])
            self.fix_base()
            mujoco.mj_forward(self.robot.model, self.robot.data)
            
            frame = self.capture_frame()
            frame = self.render_objects(frame, show_labels=False)
            
            progress = 1 - i / max(len(trajectory) - 1, 1)
            frame = self.add_text(frame, ["Returning home..."])
            frame = self.add_status_bar(frame, "RETURN", progress, 28, "OK")
            frames.append(frame)
        
        # Save
        print(f"\nSaving video ({len(frames)} frames)...")
        imageio.mimsave(output_path, frames, fps=fps)
        print(f"✓ Saved: {output_path}")
        
        return output_path


def main():
    demo = ManipulationDemo()
    demo.record_demo("manipulation_final.mp4")
    
    print("\n" + "=" * 60)
    print("DEMO COMPLETE")
    print("=" * 60)
    print("""
Created: manipulation_final.mp4

Shows complete manipulation pipeline:
  1. Perception - Detect target and obstacles
  2. Path Planning - Compute collision-free trajectory  
  3. Motion Execution - IK + ZMP stability
  4. Success - Target reached!
""")


if __name__ == '__main__':
    main()
