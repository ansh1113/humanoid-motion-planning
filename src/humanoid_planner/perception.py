"""
Perception Module for Humanoid Motion Planning

Provides interfaces for object detection, environment state estimation,
and workspace analysis for reaching tasks.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum


class ObjectType(Enum):
    """Types of objects that can be detected."""
    UNKNOWN = 0
    TARGET = 1
    OBSTACLE = 2
    GRIPPER = 3
    SURFACE = 4


@dataclass
class DetectedObject:
    """Represents a detected object in the environment."""
    
    id: int
    object_type: ObjectType
    position: np.ndarray  # 3D position [x, y, z]
    orientation: Optional[np.ndarray] = None  # Quaternion [qx, qy, qz, qw]
    dimensions: Optional[np.ndarray] = None  # [length, width, height]
    confidence: float = 1.0
    timestamp: float = 0.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            'id': self.id,
            'type': self.object_type.name,
            'position': self.position.tolist(),
            'orientation': self.orientation.tolist() if self.orientation is not None else None,
            'dimensions': self.dimensions.tolist() if self.dimensions is not None else None,
            'confidence': self.confidence,
            'timestamp': self.timestamp
        }


class ObjectDetector:
    """Interface for object detection in the robot's workspace."""
    
    def __init__(self, 
                 detection_range: float = 2.0,
                 confidence_threshold: float = 0.5):
        """
        Initialize object detector.
        
        Args:
            detection_range: Maximum detection distance (meters)
            confidence_threshold: Minimum confidence for detection
        """
        self.detection_range = detection_range
        self.confidence_threshold = confidence_threshold
        self.detected_objects = []
        self.next_id = 0
        
    def detect_objects(self, 
                      sensor_data: Optional[dict] = None) -> List[DetectedObject]:
        """
        Detect objects from sensor data.
        
        This is a placeholder interface. In practice, this would integrate
        with actual perception systems (camera, depth sensors, etc.).
        
        Args:
            sensor_data: Dictionary containing sensor readings
            
        Returns:
            objects: List of detected objects
        """
        # Placeholder implementation - returns simulated objects
        if sensor_data is None:
            return self.detected_objects
        
        # In real implementation, this would process camera/sensor data
        # and use CV/ML models for detection
        return self._process_sensor_data(sensor_data)
    
    def _process_sensor_data(self, sensor_data: dict) -> List[DetectedObject]:
        """
        Process raw sensor data to detect objects.
        
        Args:
            sensor_data: Raw sensor data
            
        Returns:
            objects: List of detected objects
        """
        objects = []
        
        # Example: Process point cloud or RGB-D data
        if 'point_cloud' in sensor_data:
            objects.extend(self._detect_from_point_cloud(sensor_data['point_cloud']))
        
        if 'rgb_image' in sensor_data and 'depth_image' in sensor_data:
            objects.extend(self._detect_from_rgbd(
                sensor_data['rgb_image'],
                sensor_data['depth_image']
            ))
        
        # Filter by confidence and range
        filtered_objects = [
            obj for obj in objects
            if obj.confidence >= self.confidence_threshold
            and np.linalg.norm(obj.position) <= self.detection_range
        ]
        
        return filtered_objects
    
    def _detect_from_point_cloud(self, point_cloud: np.ndarray) -> List[DetectedObject]:
        """
        Detect objects from point cloud data.
        
        Args:
            point_cloud: Point cloud (N x 3)
            
        Returns:
            objects: Detected objects
        """
        # Placeholder - would use clustering/segmentation
        objects = []
        
        # Simple clustering by spatial proximity
        if len(point_cloud) > 0:
            # Could use DBSCAN or region growing
            pass
        
        return objects
    
    def _detect_from_rgbd(self, 
                         rgb_image: np.ndarray,
                         depth_image: np.ndarray) -> List[DetectedObject]:
        """
        Detect objects from RGB-D images.
        
        Args:
            rgb_image: RGB image
            depth_image: Depth image
            
        Returns:
            objects: Detected objects
        """
        # Placeholder - would use CNN-based detection
        objects = []
        
        # In practice: use YOLO, Mask R-CNN, or similar
        # for object detection and segmentation
        
        return objects
    
    def add_object(self, 
                  position: np.ndarray,
                  object_type: ObjectType = ObjectType.TARGET,
                  dimensions: Optional[np.ndarray] = None,
                  confidence: float = 1.0) -> DetectedObject:
        """
        Manually add a detected object.
        
        Args:
            position: Object position (3,)
            object_type: Type of object
            dimensions: Object dimensions [length, width, height]
            confidence: Detection confidence
            
        Returns:
            obj: Created object
        """
        obj = DetectedObject(
            id=self.next_id,
            object_type=object_type,
            position=position.copy(),
            dimensions=dimensions.copy() if dimensions is not None else None,
            confidence=confidence
        )
        
        self.next_id += 1
        self.detected_objects.append(obj)
        
        return obj
    
    def get_objects_by_type(self, object_type: ObjectType) -> List[DetectedObject]:
        """
        Get all detected objects of a specific type.
        
        Args:
            object_type: Type to filter by
            
        Returns:
            objects: Filtered list of objects
        """
        return [obj for obj in self.detected_objects if obj.object_type == object_type]
    
    def get_nearest_object(self, 
                          reference_point: np.ndarray,
                          object_type: Optional[ObjectType] = None) -> Optional[DetectedObject]:
        """
        Get nearest object to a reference point.
        
        Args:
            reference_point: Reference position (3,)
            object_type: Optional type filter
            
        Returns:
            nearest_obj: Nearest object or None
        """
        objects = self.detected_objects
        if object_type is not None:
            objects = self.get_objects_by_type(object_type)
        
        if len(objects) == 0:
            return None
        
        distances = [np.linalg.norm(obj.position - reference_point) for obj in objects]
        min_idx = np.argmin(distances)
        
        return objects[min_idx]


class EnvironmentState:
    """Represents the current state of the robot's environment."""
    
    def __init__(self):
        """Initialize environment state."""
        self.objects: List[DetectedObject] = []
        self.robot_position: np.ndarray = np.zeros(3)
        self.robot_orientation: np.ndarray = np.array([0, 0, 0, 1])  # Quaternion
        self.workspace_bounds: Tuple[np.ndarray, np.ndarray] = (
            np.array([-1.0, -1.0, 0.0]),
            np.array([1.0, 1.0, 2.0])
        )
        self.timestamp: float = 0.0
        
    def update(self, 
              objects: List[DetectedObject],
              robot_position: Optional[np.ndarray] = None,
              robot_orientation: Optional[np.ndarray] = None,
              timestamp: Optional[float] = None):
        """
        Update environment state.
        
        Args:
            objects: List of detected objects
            robot_position: Robot base position
            robot_orientation: Robot base orientation
            timestamp: Current timestamp
        """
        self.objects = objects
        
        if robot_position is not None:
            self.robot_position = robot_position.copy()
        
        if robot_orientation is not None:
            self.robot_orientation = robot_orientation.copy()
        
        if timestamp is not None:
            self.timestamp = timestamp
    
    def get_obstacles_in_region(self, 
                                center: np.ndarray,
                                radius: float) -> List[DetectedObject]:
        """
        Get all obstacles within a spherical region.
        
        Args:
            center: Center of region (3,)
            radius: Radius of region (meters)
            
        Returns:
            obstacles: Objects within region
        """
        obstacles = []
        
        for obj in self.objects:
            if obj.object_type == ObjectType.OBSTACLE:
                dist = np.linalg.norm(obj.position - center)
                if dist <= radius:
                    obstacles.append(obj)
        
        return obstacles
    
    def is_position_free(self, 
                        position: np.ndarray,
                        min_clearance: float = 0.1) -> bool:
        """
        Check if a position is free of obstacles.
        
        Args:
            position: Position to check (3,)
            min_clearance: Minimum clearance required (meters)
            
        Returns:
            free: True if position is free
        """
        for obj in self.objects:
            if obj.object_type == ObjectType.OBSTACLE:
                dist = np.linalg.norm(obj.position - position)
                
                # Account for object dimensions
                if obj.dimensions is not None:
                    obj_radius = np.max(obj.dimensions) / 2
                else:
                    obj_radius = 0.05  # Default radius
                
                if dist < obj_radius + min_clearance:
                    return False
        
        return True


class WorkspaceAnalyzer:
    """Analyzes robot workspace for reachability and collision."""
    
    def __init__(self, 
                 resolution: float = 0.1,
                 workspace_bounds: Optional[Tuple[np.ndarray, np.ndarray]] = None):
        """
        Initialize workspace analyzer.
        
        Args:
            resolution: Grid resolution for discretization (meters)
            workspace_bounds: (min_bounds, max_bounds) as 3D vectors
        """
        self.resolution = resolution
        
        if workspace_bounds is None:
            self.workspace_bounds = (
                np.array([-1.0, -1.0, 0.0]),
                np.array([1.0, 1.0, 2.0])
            )
        else:
            self.workspace_bounds = workspace_bounds
        
        self.reachability_map = None
        
    def compute_reachability_map(self,
                                kinematics_solver,
                                num_samples: int = 1000) -> np.ndarray:
        """
        Compute reachability map for robot end-effector.
        
        Args:
            kinematics_solver: Forward kinematics solver
            num_samples: Number of random configurations to sample
            
        Returns:
            reachability_map: 3D grid of reachability scores
        """
        min_bounds, max_bounds = self.workspace_bounds
        grid_size = ((max_bounds - min_bounds) / self.resolution).astype(int)
        
        reachability_map = np.zeros(grid_size)
        
        # Sample random joint configurations
        for _ in range(num_samples):
            # Generate random joint angles (would use actual joint limits)
            q = np.random.uniform(-np.pi, np.pi, kinematics_solver.num_joints)
            
            # Compute forward kinematics
            pos, _ = kinematics_solver.compute_fk(q)
            
            # Map to grid cell
            grid_idx = ((pos - min_bounds) / self.resolution).astype(int)
            
            # Check bounds
            if (grid_idx >= 0).all() and (grid_idx < grid_size).all():
                reachability_map[tuple(grid_idx)] += 1
        
        # Normalize
        if reachability_map.max() > 0:
            reachability_map = reachability_map / reachability_map.max()
        
        self.reachability_map = reachability_map
        return reachability_map
    
    def is_reachable(self, 
                    position: np.ndarray,
                    threshold: float = 0.1) -> bool:
        """
        Check if a position is reachable.
        
        Args:
            position: Target position (3,)
            threshold: Minimum reachability score
            
        Returns:
            reachable: True if position is reachable
        """
        if self.reachability_map is None:
            # No map computed yet
            return True
        
        min_bounds, _ = self.workspace_bounds
        grid_idx = ((position - min_bounds) / self.resolution).astype(int)
        
        # Check bounds
        if (grid_idx < 0).any() or (grid_idx >= self.reachability_map.shape).any():
            return False
        
        return self.reachability_map[tuple(grid_idx)] >= threshold
    
    def find_nearest_reachable_position(self,
                                       target: np.ndarray,
                                       threshold: float = 0.1) -> Optional[np.ndarray]:
        """
        Find nearest reachable position to target.
        
        Args:
            target: Desired position (3,)
            threshold: Minimum reachability score
            
        Returns:
            position: Nearest reachable position or None
        """
        if self.reachability_map is None:
            return target
        
        min_bounds, max_bounds = self.workspace_bounds
        grid_idx = ((target - min_bounds) / self.resolution).astype(int)
        
        # Search in expanding radius
        max_search_radius = 10  # grid cells
        
        for radius in range(max_search_radius):
            # Check all cells at this radius
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for dz in range(-radius, radius + 1):
                        if abs(dx) == radius or abs(dy) == radius or abs(dz) == radius:
                            idx = grid_idx + np.array([dx, dy, dz])
                            
                            if (idx >= 0).all() and (idx < self.reachability_map.shape).all():
                                if self.reachability_map[tuple(idx)] >= threshold:
                                    # Convert back to world coordinates
                                    pos = min_bounds + idx * self.resolution
                                    return pos
        
        return None


class GoalPredictor:
    """Predicts optimal goal positions for reaching tasks."""
    
    def __init__(self):
        """Initialize goal predictor."""
        self.history = []
        
    def predict_grasp_position(self,
                              object_position: np.ndarray,
                              object_dimensions: Optional[np.ndarray] = None,
                              approach_direction: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict optimal grasp position for an object.
        
        Args:
            object_position: Object center position (3,)
            object_dimensions: Object dimensions [length, width, height]
            approach_direction: Preferred approach direction (3,), normalized
            
        Returns:
            grasp_position: Predicted grasp position (3,)
            grasp_orientation: Predicted grasp orientation as quaternion (4,)
        """
        # Default: approach from above
        if approach_direction is None:
            approach_direction = np.array([0, 0, -1])
        
        approach_direction = approach_direction / np.linalg.norm(approach_direction)
        
        # Compute grasp position with offset
        grasp_offset = 0.1  # 10cm offset for gripper
        
        if object_dimensions is not None:
            # Adjust offset based on object size
            grasp_offset = object_dimensions[2] / 2 + 0.05
        
        grasp_position = object_position - approach_direction * grasp_offset
        
        # Compute orientation (simplified - gripper aligned with approach)
        # In practice, would consider object geometry and gripper constraints
        grasp_orientation = np.array([0, 0, 0, 1])  # Identity quaternion
        
        return grasp_position, grasp_orientation
    
    def predict_via_points(self,
                          start_position: np.ndarray,
                          goal_position: np.ndarray,
                          obstacles: List[DetectedObject],
                          num_via_points: int = 3) -> List[np.ndarray]:
        """
        Predict intermediate via points to avoid obstacles.
        
        Args:
            start_position: Starting position (3,)
            goal_position: Goal position (3,)
            obstacles: List of obstacles
            num_via_points: Number of via points to generate
            
        Returns:
            via_points: List of intermediate positions
        """
        via_points = []
        
        # Linear interpolation as baseline
        t_values = np.linspace(0, 1, num_via_points + 2)[1:-1]
        
        for t in t_values:
            point = start_position + t * (goal_position - start_position)
            
            # Check for obstacles and adjust
            for obstacle in obstacles:
                dist = np.linalg.norm(point - obstacle.position)
                min_clearance = 0.2  # 20cm clearance
                
                if dist < min_clearance:
                    # Push point away from obstacle
                    direction = (point - obstacle.position) / dist
                    point = obstacle.position + direction * min_clearance
            
            via_points.append(point)
        
        return via_points
    
    def evaluate_goal_quality(self,
                             goal_position: np.ndarray,
                             environment: EnvironmentState,
                             robot_position: np.ndarray) -> float:
        """
        Evaluate quality of a goal position.
        
        Args:
            goal_position: Candidate goal position (3,)
            environment: Current environment state
            robot_position: Current robot position (3,)
            
        Returns:
            quality: Quality score (0 to 1, higher is better)
        """
        quality = 1.0
        
        # Penalize distance
        distance = np.linalg.norm(goal_position - robot_position)
        quality *= np.exp(-distance / 2.0)  # Exponential decay
        
        # Penalize proximity to obstacles
        for obj in environment.objects:
            if obj.object_type == ObjectType.OBSTACLE:
                dist = np.linalg.norm(goal_position - obj.position)
                min_clearance = 0.3
                
                if dist < min_clearance:
                    quality *= (dist / min_clearance) ** 2
        
        # Check workspace bounds
        min_bounds, max_bounds = environment.workspace_bounds
        if not ((goal_position >= min_bounds).all() and (goal_position <= max_bounds).all()):
            quality *= 0.1  # Heavily penalize out-of-bounds
        
        return quality
