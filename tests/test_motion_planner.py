"""
Unit tests for humanoid motion planning components.
"""

import pytest
import numpy as np
from humanoid_planner import (
    ZMPConstraint,
    SupportPolygonCalculator,
    TrajectoryOptimizer
)


class TestZMPConstraint:
    """Test ZMP constraint calculations."""
    
    def test_compute_zmp_simple(self):
        """Test ZMP computation with simple force distribution."""
        zmp_constraint = ZMPConstraint()
        
        # Four contact points in a square
        positions = np.array([
            [0.1, 0.1, 0.0],
            [0.1, -0.1, 0.0],
            [-0.1, 0.1, 0.0],
            [-0.1, -0.1, 0.0]
        ])
        
        # Equal forces
        forces = np.array([
            [0, 0, 250],
            [0, 0, 250],
            [0, 0, 250],
            [0, 0, 250]
        ])
        
        zmp = zmp_constraint.compute_zmp(forces, positions)
        
        # ZMP should be at center
        assert np.allclose(zmp, [0.0, 0.0], atol=1e-6)
    
    def test_compute_zmp_from_com(self):
        """Test ZMP computation from CoM dynamics."""
        zmp_constraint = ZMPConstraint()
        
        com_pos = np.array([0.0, 0.0, 0.8])
        com_acc = np.array([1.0, 0.0, 0.0])  # Accelerating forward
        
        zmp = zmp_constraint.compute_zmp_from_com(com_pos, com_acc, height=0.8)
        
        # ZMP should shift backward due to forward acceleration
        assert zmp[0] < com_pos[0]
    
    def test_stability_check(self):
        """Test stability checking."""
        zmp_constraint = ZMPConstraint(margin=0.01)
        
        # Square support polygon
        support_polygon = np.array([
            [0.1, 0.1],
            [0.1, -0.1],
            [-0.1, -0.1],
            [-0.1, 0.1]
        ])
        
        # ZMP at center - should be stable
        zmp_center = np.array([0.0, 0.0])
        assert zmp_constraint.is_stable(zmp_center, support_polygon)
        
        # ZMP outside polygon - should be unstable
        zmp_outside = np.array([0.5, 0.5])
        assert not zmp_constraint.is_stable(zmp_outside, support_polygon)


class TestSupportPolygonCalculator:
    """Test support polygon calculations."""
    
    def test_compute_support_polygon(self):
        """Test support polygon computation from contact points."""
        calculator = SupportPolygonCalculator()
        
        # Four contact points
        contacts = np.array([
            [0.1, 0.1, 0.0],
            [0.1, -0.1, 0.0],
            [-0.1, 0.1, 0.0],
            [-0.1, -0.1, 0.0]
        ])
        
        polygon = calculator.compute_support_polygon(contacts)
        
        # Should have 4 vertices
        assert len(polygon) == 4
        
        # Check that it forms a convex hull
        area = calculator.compute_polygon_area(polygon)
        expected_area = 0.04  # 0.2m x 0.2m square
        assert np.isclose(area, expected_area, atol=1e-3)
    
    def test_get_foot_corners(self):
        """Test foot corner generation."""
        calculator = SupportPolygonCalculator(foot_size=(0.2, 0.1))
        
        # Foot at origin, no rotation
        foot_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        corners = calculator.get_foot_corners(foot_pose)
        
        # Should have 4 corners
        assert corners.shape == (4, 3)
        
        # Check corner positions
        assert np.allclose(corners[:, 2], 0.0)  # All on ground
    
    def test_double_support_polygon(self):
        """Test double support polygon computation."""
        calculator = SupportPolygonCalculator()
        
        left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
        
        polygon = calculator.compute_double_support_polygon(left_foot, right_foot)
        
        # Should have vertices
        assert len(polygon) >= 4
        
        # Centroid should be between feet
        centroid = calculator.compute_polygon_centroid(polygon)
        assert np.abs(centroid[1]) < 0.1


class TestTrajectoryOptimizer:
    """Test trajectory optimization."""
    
    def test_optimize_trajectory_straight_line(self):
        """Test trajectory optimization with no obstacles."""
        optimizer = TrajectoryOptimizer(num_joints=3, dt=0.1)
        
        q_start = np.array([0.0, 0.0, 0.0])
        q_goal = np.array([1.0, 1.0, 1.0])
        
        trajectory = optimizer.optimize_trajectory(
            q_start=q_start,
            q_goal=q_goal,
            num_waypoints=10,
            velocity_limit=2.0
        )
        
        # Check start and end
        assert np.allclose(trajectory[0], q_start, atol=1e-3)
        assert np.allclose(trajectory[-1], q_goal, atol=1e-3)
        
        # Check trajectory length
        assert len(trajectory) == 10
    
    def test_velocity_profile(self):
        """Test velocity profile computation."""
        optimizer = TrajectoryOptimizer(num_joints=2, dt=0.1)
        
        trajectory = np.array([
            [0.0, 0.0],
            [0.1, 0.1],
            [0.2, 0.2],
            [0.3, 0.3]
        ])
        
        velocities = optimizer.compute_velocity_profile(trajectory)
        
        # Velocity should be constant
        assert velocities.shape == (3, 2)
        assert np.allclose(velocities, 1.0, atol=1e-6)
    
    def test_trajectory_feasibility(self):
        """Test trajectory feasibility checking."""
        optimizer = TrajectoryOptimizer(num_joints=2, dt=0.1)
        
        trajectory = np.array([
            [0.0, 0.0],
            [0.1, 0.1],
            [0.2, 0.2]
        ])
        
        q_limits = (np.array([-1.0, -1.0]), np.array([1.0, 1.0]))
        
        # Should be feasible
        assert optimizer.check_trajectory_feasibility(
            trajectory, q_limits, velocity_limit=2.0, acceleration_limit=5.0
        )
        
        # Test with violated limits
        trajectory_invalid = np.array([
            [0.0, 0.0],
            [2.0, 2.0],  # Outside joint limits
            [4.0, 4.0]
        ])
        
        assert not optimizer.check_trajectory_feasibility(
            trajectory_invalid, q_limits, velocity_limit=2.0, acceleration_limit=5.0
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
