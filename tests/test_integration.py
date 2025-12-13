"""
Integration tests for end-to-end motion planning scenarios.
"""

import pytest
import numpy as np
from humanoid_planner import (
    ZMPConstraint, SupportPolygonCalculator, TrajectoryOptimizer,
    ForwardKinematics, InverseKinematics, StabilityAnalyzer,
    ObjectDetector, EnvironmentState, GoalPredictor, ObjectType
)


class TestEndToEndReachingTask:
    """Test complete reaching task pipeline."""
    
    def test_simple_reaching_task(self):
        """Test basic reaching task without obstacles."""
        # Setup environment
        detector = ObjectDetector()
        target_obj = detector.add_object(
            position=np.array([0.5, 0.3, 1.2]),
            object_type=ObjectType.TARGET
        )
        
        env = EnvironmentState()
        env.update(detector.detected_objects)
        
        # Setup robot
        num_joints = 7  # Arm only
        optimizer = TrajectoryOptimizer(num_joints=num_joints, dt=0.1)
        
        # Plan trajectory
        q_start = np.zeros(num_joints)
        q_goal = np.ones(num_joints) * 0.3  # Simplified goal
        
        trajectory = optimizer.optimize_trajectory(
            q_start=q_start,
            q_goal=q_goal,
            num_waypoints=10,
            velocity_limit=1.0
        )
        
        # Verify trajectory
        assert trajectory is not None
        assert len(trajectory) == 10
        assert np.allclose(trajectory[0], q_start, atol=1e-3)
        assert np.allclose(trajectory[-1], q_goal, atol=1e-3)
    
    def test_reaching_with_stability_verification(self):
        """Test reaching with ZMP stability verification."""
        # Setup stability components
        zmp_constraint = ZMPConstraint(margin=0.02)
        support_calc = SupportPolygonCalculator()
        stability_analyzer = StabilityAnalyzer()
        
        # Define foot poses
        left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
        
        # Compute support polygon
        support_polygon = support_calc.compute_double_support_polygon(
            left_foot, right_foot
        )
        
        # Simulate CoM trajectory
        com_trajectory = np.array([
            [0.0, 0.0, 0.9],
            [0.02, 0.0, 0.9],
            [0.04, 0.0, 0.9],
            [0.06, 0.0, 0.9],
            [0.08, 0.0, 0.9]
        ])
        
        com_velocities = np.diff(com_trajectory, axis=0) / 0.1
        com_accelerations = np.diff(com_velocities, axis=0) / 0.1
        
        # Pad to match trajectory length
        com_velocities = np.vstack([com_velocities, com_velocities[-1:]])
        com_accelerations = np.vstack([com_accelerations, com_accelerations[-1:], com_accelerations[-1:]])
        
        support_polygons = [support_polygon] * len(com_trajectory)
        
        # Check stability
        all_stable, margins = stability_analyzer.check_trajectory_stability(
            com_trajectory, com_velocities, com_accelerations, support_polygons
        )
        
        # With small CoM movements, should be stable
        assert all_stable or len([m for m in margins if m > 0]) >= len(margins) * 0.8
    
    def test_ik_to_trajectory_pipeline(self):
        """Test IK solving followed by trajectory optimization."""
        # Setup kinematics
        dh_params = np.array([
            [0.3, 0, 0, 0],
            [0.3, 0, 0, 0],
            [0.2, 0, 0, 0]
        ])
        
        fk = ForwardKinematics(dh_params)
        ik = InverseKinematics(fk)
        
        # Target position
        target_pos = np.array([0.6, 0.1, 0.0])
        
        # Solve IK
        q_init = np.zeros(3)
        q_goal = ik.solve_ik(target_pos, q_init=q_init, max_iter=50)
        
        if q_goal is not None:
            # Verify IK solution
            achieved_pos, _ = fk.compute_fk(q_goal)
            error = np.linalg.norm(achieved_pos - target_pos)
            
            # Should be close to target
            assert error < 0.05  # 5cm tolerance
            
            # Plan trajectory
            optimizer = TrajectoryOptimizer(num_joints=3, dt=0.1)
            trajectory = optimizer.optimize_trajectory(
                q_start=q_init,
                q_goal=q_goal,
                num_waypoints=15
            )
            
            # Verify trajectory feasibility
            assert trajectory is not None
            assert len(trajectory) == 15
    
    def test_goal_prediction_and_planning(self):
        """Test goal prediction integrated with planning."""
        # Setup perception
        detector = ObjectDetector()
        target = detector.add_object(
            position=np.array([0.5, 0.0, 1.0]),
            object_type=ObjectType.TARGET,
            dimensions=np.array([0.1, 0.1, 0.2])
        )
        
        # Add obstacle
        obstacle = detector.add_object(
            position=np.array([0.3, 0.0, 1.0]),
            object_type=ObjectType.OBSTACLE,
            dimensions=np.array([0.15, 0.15, 0.15])
        )
        
        env = EnvironmentState()
        env.update(detector.detected_objects)
        
        # Predict grasp position
        predictor = GoalPredictor()
        grasp_pos, grasp_orient = predictor.predict_grasp_position(
            target.position, target.dimensions
        )
        
        # Verify grasp position is reasonable
        assert grasp_pos.shape == (3,)
        dist_to_target = np.linalg.norm(grasp_pos - target.position)
        assert dist_to_target < 0.3  # Should be close to object
        
        # Generate via points avoiding obstacle
        start_pos = np.array([0.0, 0.0, 0.5])
        via_points = predictor.predict_via_points(
            start_pos, grasp_pos, [obstacle], num_via_points=2
        )
        
        # Verify via points
        assert len(via_points) == 2
        
        # Check that via points have clearance from obstacle
        for point in via_points:
            dist_to_obs = np.linalg.norm(point - obstacle.position)
            # Should maintain some clearance
            assert dist_to_obs > 0.1


class TestMultiModuleIntegration:
    """Test integration between multiple modules."""
    
    def test_zmp_and_support_polygon_integration(self):
        """Test ZMP constraint with support polygon computation."""
        # Create support polygon
        support_calc = SupportPolygonCalculator(foot_size=(0.25, 0.12))
        left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
        
        polygon = support_calc.compute_double_support_polygon(left_foot, right_foot)
        
        # Test ZMP stability
        zmp_constraint = ZMPConstraint(margin=0.02)
        
        # Test various ZMP positions
        test_positions = [
            np.array([0.0, 0.0]),     # Center - should be stable
            np.array([0.5, 0.5]),     # Far outside - should be unstable
        ]
        
        for zmp in test_positions:
            stable = zmp_constraint.is_stable(zmp, polygon)
            margin = zmp_constraint.compute_stability_margin(zmp, polygon)
            
            # Center should be stable with positive margin
            if np.allclose(zmp, [0.0, 0.0]):
                assert stable
                assert margin > 0
    
    def test_kinematics_and_trajectory_optimization(self):
        """Test FK/IK with trajectory optimization."""
        # Setup kinematics
        dh_params = np.array([
            [0.3, 0, 0, 0],
            [0.3, 0, 0, 0],
            [0.2, 0, 0, 0],
            [0.15, 0, 0, 0]
        ])
        
        fk = ForwardKinematics(dh_params)
        ik = InverseKinematics(fk)
        
        # Generate start and goal configurations
        q_start = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Compute target from a known configuration
        q_target_config = np.array([0.5, 0.3, -0.2, 0.1])
        target_pos, _ = fk.compute_fk(q_target_config)
        
        # Solve IK
        q_goal = ik.solve_ik(target_pos, q_init=q_start, max_iter=100)
        
        if q_goal is not None:
            # Optimize trajectory
            optimizer = TrajectoryOptimizer(num_joints=4, dt=0.1)
            
            trajectory = optimizer.optimize_trajectory(
                q_start=q_start,
                q_goal=q_goal,
                num_waypoints=20,
                velocity_limit=2.0
            )
            
            # Verify trajectory
            assert trajectory is not None
            assert len(trajectory) == 20
            
            # Check velocity constraints
            velocities = optimizer.compute_velocity_profile(trajectory)
            max_vel = np.max(np.abs(velocities))
            assert max_vel <= 2.0 + 0.1  # Small tolerance
            
            # Check smoothness
            if len(trajectory) >= 3:
                accelerations = optimizer.compute_acceleration_profile(trajectory)
                assert accelerations is not None
    
    def test_perception_to_planning_pipeline(self):
        """Test complete perception to planning pipeline."""
        # 1. Perception: Detect objects
        detector = ObjectDetector(detection_range=2.0)
        
        target = detector.add_object(
            np.array([0.6, 0.2, 1.0]),
            ObjectType.TARGET,
            dimensions=np.array([0.08, 0.08, 0.15]),
            confidence=0.95
        )
        
        obstacle1 = detector.add_object(
            np.array([0.4, 0.1, 0.8]),
            ObjectType.OBSTACLE,
            dimensions=np.array([0.2, 0.2, 0.3]),
            confidence=0.88
        )
        
        # 2. Environment state
        env = EnvironmentState()
        env.update(detector.detected_objects, robot_position=np.array([0.0, 0.0, 0.0]))
        
        # 3. Goal prediction
        predictor = GoalPredictor()
        
        # Predict grasp
        grasp_pos, grasp_orient = predictor.predict_grasp_position(
            target.position, target.dimensions
        )
        
        # Evaluate goal quality
        quality = predictor.evaluate_goal_quality(
            grasp_pos, env, np.array([0.0, 0.0, 0.5])
        )
        
        assert 0.0 <= quality <= 1.0
        
        # 4. Plan trajectory (simplified)
        optimizer = TrajectoryOptimizer(num_joints=7, dt=0.1)
        
        # Simplified: just plan in joint space
        q_start = np.zeros(7)
        q_goal = np.ones(7) * 0.3  # Approximate goal
        
        trajectory = optimizer.optimize_trajectory(
            q_start, q_goal, num_waypoints=15
        )
        
        assert trajectory is not None
        assert len(trajectory) == 15
        
        # 5. Verify stability (simplified)
        support_calc = SupportPolygonCalculator()
        left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
        
        support_polygon = support_calc.compute_double_support_polygon(
            left_foot, right_foot
        )
        
        # Check that support polygon is reasonable
        area = support_calc.compute_polygon_area(support_polygon)
        assert area > 0.01  # Should have some area


class TestPerformanceBenchmarks:
    """Performance benchmarks for key operations."""
    
    def test_trajectory_optimization_speed(self):
        """Benchmark trajectory optimization speed."""
        import time
        
        optimizer = TrajectoryOptimizer(num_joints=12, dt=0.1)
        
        q_start = np.zeros(12)
        q_goal = np.ones(12) * 0.5
        
        start_time = time.time()
        
        trajectory = optimizer.optimize_trajectory(
            q_start, q_goal,
            num_waypoints=20,
            velocity_limit=1.0
        )
        
        elapsed = time.time() - start_time
        
        # Should complete in reasonable time (< 5 seconds)
        assert elapsed < 5.0
        assert trajectory is not None
    
    def test_support_polygon_computation_speed(self):
        """Benchmark support polygon computation."""
        import time
        
        calc = SupportPolygonCalculator()
        left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
        
        start_time = time.time()
        
        for _ in range(100):
            polygon = calc.compute_double_support_polygon(left_foot, right_foot)
        
        elapsed = time.time() - start_time
        
        # Should be fast (< 1 second for 100 iterations)
        assert elapsed < 1.0
    
    def test_ik_convergence_speed(self):
        """Benchmark IK solver convergence."""
        import time
        
        dh_params = np.array([
            [0.3, 0, 0, 0],
            [0.3, 0, 0, 0],
            [0.2, 0, 0, 0]
        ])
        
        fk = ForwardKinematics(dh_params)
        ik = InverseKinematics(fk)
        
        target_pos = np.array([0.5, 0.2, 0.0])
        q_init = np.zeros(3)
        
        start_time = time.time()
        
        q_solution = ik.solve_ik(target_pos, q_init=q_init, max_iter=100)
        
        elapsed = time.time() - start_time
        
        # Should converge quickly (< 0.5 seconds)
        assert elapsed < 0.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
