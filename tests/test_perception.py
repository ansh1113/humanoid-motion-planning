"""
Unit tests for perception module.
"""

import pytest
import numpy as np
from humanoid_planner import (
    ObjectDetector, EnvironmentState, WorkspaceAnalyzer,
    GoalPredictor, DetectedObject, ObjectType, ForwardKinematics
)


class TestObjectDetector:
    """Test object detection functionality."""
    
    def test_initialization(self):
        """Test detector initialization."""
        detector = ObjectDetector(detection_range=3.0, confidence_threshold=0.7)
        
        assert detector.detection_range == 3.0
        assert detector.confidence_threshold == 0.7
        assert len(detector.detected_objects) == 0
    
    def test_add_object(self):
        """Test manually adding objects."""
        detector = ObjectDetector()
        
        position = np.array([0.5, 0.3, 1.0])
        obj = detector.add_object(
            position=position,
            object_type=ObjectType.TARGET,
            confidence=0.95
        )
        
        assert obj.id == 0
        assert obj.object_type == ObjectType.TARGET
        assert np.allclose(obj.position, position)
        assert obj.confidence == 0.95
        assert len(detector.detected_objects) == 1
    
    def test_get_objects_by_type(self):
        """Test filtering objects by type."""
        detector = ObjectDetector()
        
        detector.add_object(np.array([1.0, 0.0, 0.0]), ObjectType.TARGET)
        detector.add_object(np.array([2.0, 0.0, 0.0]), ObjectType.OBSTACLE)
        detector.add_object(np.array([3.0, 0.0, 0.0]), ObjectType.TARGET)
        
        targets = detector.get_objects_by_type(ObjectType.TARGET)
        obstacles = detector.get_objects_by_type(ObjectType.OBSTACLE)
        
        assert len(targets) == 2
        assert len(obstacles) == 1
    
    def test_get_nearest_object(self):
        """Test finding nearest object."""
        detector = ObjectDetector()
        
        detector.add_object(np.array([1.0, 0.0, 0.0]), ObjectType.TARGET)
        detector.add_object(np.array([2.0, 0.0, 0.0]), ObjectType.TARGET)
        detector.add_object(np.array([0.5, 0.0, 0.0]), ObjectType.TARGET)
        
        reference = np.array([0.0, 0.0, 0.0])
        nearest = detector.get_nearest_object(reference, ObjectType.TARGET)
        
        assert nearest is not None
        assert np.allclose(nearest.position, [0.5, 0.0, 0.0])
    
    def test_detected_object_to_dict(self):
        """Test object serialization."""
        obj = DetectedObject(
            id=1,
            object_type=ObjectType.TARGET,
            position=np.array([0.5, 0.3, 1.0]),
            confidence=0.9
        )
        
        obj_dict = obj.to_dict()
        
        assert obj_dict['id'] == 1
        assert obj_dict['type'] == 'TARGET'
        assert obj_dict['confidence'] == 0.9


class TestEnvironmentState:
    """Test environment state management."""
    
    def test_initialization(self):
        """Test environment initialization."""
        env = EnvironmentState()
        
        assert len(env.objects) == 0
        assert np.allclose(env.robot_position, [0.0, 0.0, 0.0])
    
    def test_update(self):
        """Test state update."""
        env = EnvironmentState()
        
        obj = DetectedObject(
            id=0,
            object_type=ObjectType.TARGET,
            position=np.array([1.0, 0.0, 0.5])
        )
        
        robot_pos = np.array([0.0, 0.0, 0.0])
        
        env.update([obj], robot_position=robot_pos, timestamp=1.0)
        
        assert len(env.objects) == 1
        assert np.allclose(env.robot_position, robot_pos)
        assert env.timestamp == 1.0
    
    def test_get_obstacles_in_region(self):
        """Test getting obstacles in region."""
        env = EnvironmentState()
        
        # Add obstacles at different distances
        env.objects = [
            DetectedObject(0, ObjectType.OBSTACLE, np.array([0.5, 0.0, 0.0])),
            DetectedObject(1, ObjectType.OBSTACLE, np.array([2.0, 0.0, 0.0])),
            DetectedObject(2, ObjectType.TARGET, np.array([0.3, 0.0, 0.0]))  # Not obstacle
        ]
        
        center = np.array([0.0, 0.0, 0.0])
        obstacles = env.get_obstacles_in_region(center, radius=1.0)
        
        # Should only get the first obstacle (within 1m)
        assert len(obstacles) == 1
        assert obstacles[0].id == 0
    
    def test_is_position_free(self):
        """Test collision-free position checking."""
        env = EnvironmentState()
        
        # Add obstacle
        env.objects = [
            DetectedObject(
                0, ObjectType.OBSTACLE,
                np.array([1.0, 0.0, 0.0]),
                dimensions=np.array([0.1, 0.1, 0.1])
            )
        ]
        
        # Position far from obstacle - should be free
        assert env.is_position_free(np.array([5.0, 0.0, 0.0]), min_clearance=0.1)
        
        # Position close to obstacle - should not be free
        assert not env.is_position_free(np.array([1.0, 0.0, 0.0]), min_clearance=0.1)


class TestWorkspaceAnalyzer:
    """Test workspace analysis."""
    
    def test_initialization(self):
        """Test analyzer initialization."""
        analyzer = WorkspaceAnalyzer(resolution=0.1)
        
        assert analyzer.resolution == 0.1
        assert analyzer.reachability_map is None
    
    def test_is_reachable_without_map(self):
        """Test reachability check without map."""
        analyzer = WorkspaceAnalyzer()
        
        # Without computed map, should return True
        position = np.array([0.5, 0.0, 1.0])
        assert analyzer.is_reachable(position)
    
    def test_compute_reachability_map(self):
        """Test reachability map computation."""
        # Create simple FK solver
        dh_params = np.array([
            [0.3, 0, 0, 0],
            [0.3, 0, 0, 0],
            [0.2, 0, 0, 0]
        ])
        fk = ForwardKinematics(dh_params)
        
        analyzer = WorkspaceAnalyzer(
            resolution=0.2,
            workspace_bounds=(
                np.array([0.0, -0.5, 0.0]),
                np.array([1.0, 0.5, 1.0])
            )
        )
        
        # Compute with small number of samples for speed
        reachability_map = analyzer.compute_reachability_map(fk, num_samples=100)
        
        assert reachability_map is not None
        assert reachability_map.shape == (5, 5, 5)  # (1.0/0.2, 1.0/0.2, 1.0/0.2)
        assert analyzer.reachability_map is not None


class TestGoalPredictor:
    """Test goal prediction."""
    
    def test_initialization(self):
        """Test predictor initialization."""
        predictor = GoalPredictor()
        
        assert len(predictor.history) == 0
    
    def test_predict_grasp_position(self):
        """Test grasp position prediction."""
        predictor = GoalPredictor()
        
        object_pos = np.array([0.5, 0.0, 1.0])
        object_dims = np.array([0.1, 0.1, 0.2])
        
        grasp_pos, grasp_orient = predictor.predict_grasp_position(
            object_pos, object_dims
        )
        
        assert grasp_pos.shape == (3,)
        assert grasp_orient.shape == (4,)
        
        # Grasp position should be offset from object
        assert not np.allclose(grasp_pos, object_pos)
    
    def test_predict_via_points(self):
        """Test via point generation."""
        predictor = GoalPredictor()
        
        start = np.array([0.0, 0.0, 1.0])
        goal = np.array([1.0, 0.0, 1.0])
        
        # No obstacles
        via_points = predictor.predict_via_points(start, goal, [], num_via_points=3)
        
        assert len(via_points) == 3
        
        # Via points should be between start and goal
        for point in via_points:
            assert 0.0 < point[0] < 1.0
    
    def test_predict_via_points_with_obstacles(self):
        """Test via point generation avoiding obstacles."""
        predictor = GoalPredictor()
        
        start = np.array([0.0, 0.0, 1.0])
        goal = np.array([1.0, 0.0, 1.0])
        
        # Add obstacle in the path
        obstacle = DetectedObject(
            0, ObjectType.OBSTACLE,
            np.array([0.5, 0.0, 1.0]),
            dimensions=np.array([0.1, 0.1, 0.1])
        )
        
        via_points = predictor.predict_via_points(
            start, goal, [obstacle], num_via_points=3
        )
        
        assert len(via_points) == 3
        
        # Via points should be adjusted away from obstacle
        for point in via_points:
            dist = np.linalg.norm(point - obstacle.position)
            # Should maintain some clearance
            assert dist > 0.05 or not (0.4 < point[0] < 0.6)
    
    def test_evaluate_goal_quality(self):
        """Test goal quality evaluation."""
        predictor = GoalPredictor()
        env = EnvironmentState()
        
        robot_pos = np.array([0.0, 0.0, 0.0])
        goal_pos = np.array([0.5, 0.0, 1.0])
        
        quality = predictor.evaluate_goal_quality(goal_pos, env, robot_pos)
        
        assert 0.0 <= quality <= 1.0
    
    def test_goal_quality_with_obstacles(self):
        """Test that obstacles reduce goal quality."""
        predictor = GoalPredictor()
        env = EnvironmentState()
        
        robot_pos = np.array([0.0, 0.0, 0.0])
        goal_pos = np.array([0.5, 0.0, 1.0])
        
        # Quality without obstacles
        quality_no_obs = predictor.evaluate_goal_quality(goal_pos, env, robot_pos)
        
        # Add obstacle near goal
        env.objects = [
            DetectedObject(0, ObjectType.OBSTACLE, np.array([0.5, 0.1, 1.0]))
        ]
        
        # Quality with obstacle
        quality_with_obs = predictor.evaluate_goal_quality(goal_pos, env, robot_pos)
        
        # Quality should be lower with obstacle nearby
        assert quality_with_obs < quality_no_obs


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
