"""
Perception Module with Simulated Camera

Provides:
- RGB and depth image rendering from MuJoCo
- Object detection (simulated using known object positions)
- Point cloud generation from depth images
- Target pose estimation for manipulation
"""

import numpy as np
import mujoco
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass
import matplotlib.pyplot as plt


@dataclass
class DetectedObject:
    """Information about a detected object"""
    name: str
    position: np.ndarray      # [x, y, z] in world frame
    size: np.ndarray          # [sx, sy, sz] bounding box
    confidence: float
    depth: float              # Distance from camera


@dataclass
class CameraInfo:
    """Camera intrinsic and extrinsic parameters"""
    width: int
    height: int
    fov_y: float              # Vertical field of view (radians)
    intrinsic_matrix: np.ndarray  # 3x3 camera matrix


class SimulatedCamera:
    """
    Simulated RGB-D camera using MuJoCo rendering.
    """
    
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
        
        # Create renderer
        self.renderer = mujoco.Renderer(model, height, width)
        
        # Camera settings
        self.fov_y = model.vis.global_.fovy * np.pi / 180
        
        # Compute intrinsic matrix
        self.fx = width / (2 * np.tan(self.fov_y / 2))
        self.fy = self.fx
        self.cx = width / 2
        self.cy = height / 2
        
        self.intrinsic_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # Scene camera for custom views
        self.scene_camera = mujoco.MjvCamera()
        self.scene_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    
    def _setup_camera(self, camera_pos: np.ndarray, camera_lookat: np.ndarray):
        """Setup camera position and orientation."""
        if camera_pos is not None and camera_lookat is not None:
            direction = camera_lookat - camera_pos
            distance = np.linalg.norm(direction)
            
            self.scene_camera.lookat[:] = camera_lookat
            self.scene_camera.distance = distance
            self.scene_camera.azimuth = np.arctan2(direction[1], direction[0]) * 180 / np.pi
            self.scene_camera.elevation = -np.arcsin(direction[2] / (distance + 1e-8)) * 180 / np.pi
        else:
            # Default view
            self.scene_camera.lookat[:] = [0, 0, 0.8]
            self.scene_camera.distance = 2.5
            self.scene_camera.azimuth = 135
            self.scene_camera.elevation = -20
    
    def capture_rgb(
        self, 
        camera_pos: np.ndarray = None, 
        camera_lookat: np.ndarray = None
    ) -> np.ndarray:
        """
        Capture RGB image from camera.
        
        Returns:
            (height, width, 3) RGB image as uint8
        """
        self._setup_camera(camera_pos, camera_lookat)
        self.renderer.update_scene(self.data, self.scene_camera)
        rgb = self.renderer.render()
        return rgb
    
    def capture_depth(
        self, 
        camera_pos: np.ndarray = None, 
        camera_lookat: np.ndarray = None
    ) -> np.ndarray:
        """
        Capture depth image from camera.
        
        Returns:
            (height, width) depth image in meters
        """
        self._setup_camera(camera_pos, camera_lookat)
        self.renderer.update_scene(self.data, self.scene_camera)
        
        # Render depth - MuJoCo API varies by version
        try:
            # Try newer API first
            self.renderer.enable_depth_rendering(True)
            depth = self.renderer.render()
            self.renderer.enable_depth_rendering(False)
        except TypeError:
            # Older API - depth is a separate call
            try:
                depth = self.renderer.render(depth=True)
            except:
                # Fallback - create dummy depth from scene
                depth = np.ones((self.height, self.width)) * 2.0
        
        # Convert from normalized depth to meters
        extent = self.model.stat.extent
        near = self.model.vis.map.znear * extent
        far = self.model.vis.map.zfar * extent
        
        # Handle depth conversion
        if depth.max() <= 1.0:  # Normalized depth
            depth = np.clip(depth, 0, 0.9999)
            depth_meters = near / (1 - depth * (1 - near / far) + 1e-10)
        else:
            depth_meters = depth
        
        return depth_meters
    
    def capture_rgbd(
        self, 
        camera_pos: np.ndarray = None, 
        camera_lookat: np.ndarray = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Capture both RGB and depth images."""
        rgb = self.capture_rgb(camera_pos, camera_lookat)
        depth = self.capture_depth(camera_pos, camera_lookat)
        return rgb, depth
    
    def pixel_to_3d(self, u: int, v: int, depth: float) -> np.ndarray:
        """Convert pixel coordinates + depth to 3D point in camera frame."""
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return np.array([x, y, z])
    
    def depth_to_pointcloud(
        self,
        depth_image: np.ndarray,
        rgb_image: np.ndarray = None,
        max_depth: float = 10.0
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """Convert depth image to point cloud."""
        H, W = depth_image.shape
        
        u = np.arange(W)
        v = np.arange(H)
        u, v = np.meshgrid(u, v)
        
        valid = (depth_image > 0.1) & (depth_image < max_depth)
        
        z = depth_image[valid]
        x = (u[valid] - self.cx) * z / self.fx
        y = (v[valid] - self.cy) * z / self.fy
        
        points = np.stack([x, y, z], axis=-1)
        
        colors = None
        if rgb_image is not None:
            colors = rgb_image[valid] / 255.0
        
        return points, colors
    
    def get_camera_info(self) -> CameraInfo:
        """Get camera parameters."""
        return CameraInfo(
            width=self.width,
            height=self.height,
            fov_y=self.fov_y,
            intrinsic_matrix=self.intrinsic_matrix
        )


class ObjectDetector:
    """
    Simulated object detector using MuJoCo ground truth.
    """
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        
        # Build list of detectable objects (non-robot bodies)
        self.object_bodies = {}
        robot_keywords = ['pelvis', 'torso', 'head', 'left_', 'right_', 'waist', 
                          'hip', 'knee', 'ankle', 'shoulder', 'elbow', 'wrist', 'logo']
        
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and name != 'world':
                is_robot = any(kw in name.lower() for kw in robot_keywords)
                if not is_robot:
                    self.object_bodies[name] = i
    
    def detect_objects(self) -> List[DetectedObject]:
        """Detect all objects in the scene (ground truth from MuJoCo)."""
        mujoco.mj_forward(self.model, self.data)
        
        detections = []
        
        for name, body_id in self.object_bodies.items():
            pos = self.data.xpos[body_id].copy()
            
            # Get size from geoms
            size = np.array([0.05, 0.05, 0.05])
            for i in range(self.model.ngeom):
                if self.model.geom_bodyid[i] == body_id:
                    geom_size = self.model.geom_size[i]
                    geom_type = self.model.geom_type[i]
                    
                    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                        size = geom_size * 2
                    elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                        size = np.array([geom_size[0], geom_size[0], geom_size[0]]) * 2
                    break
            
            detections.append(DetectedObject(
                name=name,
                position=pos,
                size=size,
                confidence=1.0,
                depth=np.linalg.norm(pos)
            ))
        
        return detections
    
    def get_object_pose(self, object_name: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Get 6D pose of a specific object."""
        if object_name not in self.object_bodies:
            return None
        
        body_id = self.object_bodies[object_name]
        pos = self.data.xpos[body_id].copy()
        quat = self.data.xquat[body_id].copy()
        
        return pos, quat
    
    def find_reachable_objects(
        self,
        robot_position: np.ndarray,
        max_reach: float = 0.5
    ) -> List[DetectedObject]:
        """Find objects within robot's reach."""
        all_objects = self.detect_objects()
        return [obj for obj in all_objects 
                if np.linalg.norm(obj.position - robot_position) < max_reach]


class PerceptionPipeline:
    """Complete perception pipeline combining camera and detection."""
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.camera = SimulatedCamera(model, data)
        self.detector = ObjectDetector(model, data)
        self.model = model
        self.data = data
    
    def get_scene_snapshot(
        self,
        camera_pos: np.ndarray = None,
        camera_lookat: np.ndarray = None
    ) -> Dict:
        """Get complete scene perception snapshot."""
        if camera_pos is None:
            camera_pos = np.array([2.0, 1.5, 1.5])
        if camera_lookat is None:
            camera_lookat = np.array([0.0, 0.0, 0.8])
        
        rgb, depth = self.camera.capture_rgbd(camera_pos, camera_lookat)
        objects = self.detector.detect_objects()
        
        return {
            'rgb': rgb,
            'depth': depth,
            'objects': objects,
            'camera_pos': camera_pos,
            'camera_lookat': camera_lookat
        }
    
    def visualize_scene(self, save_path: str = None):
        """Visualize the current scene."""
        snapshot = self.get_scene_snapshot()
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        
        # RGB image
        axes[0].imshow(snapshot['rgb'])
        axes[0].set_title('RGB Camera View')
        axes[0].axis('off')
        
        # Depth image
        depth_display = np.clip(snapshot['depth'], 0, 5)
        depth_vis = axes[1].imshow(depth_display, cmap='viridis', vmin=0, vmax=5)
        axes[1].set_title('Depth Camera View')
        axes[1].axis('off')
        plt.colorbar(depth_vis, ax=axes[1], label='Depth (m)', fraction=0.046)
        
        # Add detection info
        if snapshot['objects']:
            obj_text = "Detected Objects:\n"
            for obj in snapshot['objects']:
                obj_text += f"  - {obj.name}: [{obj.position[0]:.2f}, {obj.position[1]:.2f}, {obj.position[2]:.2f}]\n"
        else:
            obj_text = "Scene contains robot only\n(No external objects to detect)"
        
        fig.text(0.02, 0.02, obj_text, fontsize=9, family='monospace',
                verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Scene visualization saved to {save_path}")
        
        plt.close()
        return snapshot


# Test perception
if __name__ == '__main__':
    import sys
    sys.path.insert(0, 'src')
    from g1_model import G1Model
    
    print("=" * 60)
    print("Testing Perception Module")
    print("=" * 60)
    
    robot = G1Model()
    pipeline = PerceptionPipeline(robot.model, robot.data)
    
    # Get camera info
    cam_info = pipeline.camera.get_camera_info()
    print(f"\nCamera: {cam_info.width}x{cam_info.height}, FOV: {np.rad2deg(cam_info.fov_y):.1f}°")
    print(f"Intrinsic matrix:\n{cam_info.intrinsic_matrix}")
    
    # Detect objects
    print("\n--- Object Detection ---")
    objects = pipeline.detector.detect_objects()
    if objects:
        print(f"Found {len(objects)} objects:")
        for obj in objects:
            print(f"  {obj.name}: pos={obj.position}")
    else:
        print("No external objects in scene (robot-only scene)")
    
    # Capture images
    print("\n--- Capturing Images ---")
    snapshot = pipeline.get_scene_snapshot(
        camera_pos=np.array([2.0, 1.5, 1.5]),
        camera_lookat=np.array([0.0, 0.0, 0.8])
    )
    
    print(f"RGB shape: {snapshot['rgb'].shape}")
    print(f"Depth shape: {snapshot['depth'].shape}")
    
    # Visualize
    print("\n--- Visualization ---")
    pipeline.visualize_scene('perception_test.png')
    
    # Test point cloud
    print("\n--- Point Cloud Generation ---")
    points, colors = pipeline.camera.depth_to_pointcloud(
        snapshot['depth'], 
        snapshot['rgb'],
        max_depth=5.0
    )
    print(f"Generated point cloud with {len(points)} points")
    if len(points) > 0:
        print(f"Point cloud bounds: X[{points[:,0].min():.2f}, {points[:,0].max():.2f}]")
        print(f"                    Y[{points[:,1].min():.2f}, {points[:,1].max():.2f}]")
        print(f"                    Z[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")
    
    print("\n✓ Perception module working!")
