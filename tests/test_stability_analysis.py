"""
Unit tests for stability analysis module.
"""

import pytest
import numpy as np
from humanoid_planner import StabilityAnalyzer, CenterOfMassTracker


class TestStabilityAnalyzer:
    """Test stability analysis functionality."""
    
    def test_compute_center_of_mass(self):
        """Test CoM computation from link positions and masses."""
        analyzer = StabilityAnalyzer()
        
        # Three links in a line
        link_positions = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0]
        ])
        link_masses = np.array([10.0, 10.0, 10.0])
        
        com = analyzer.compute_center_of_mass(link_positions, link_masses)
        
        # CoM should be at center
        assert np.allclose(com, [1.0, 0.0, 0.0], atol=1e-6)
    
    def test_compute_center_of_mass_unequal_masses(self):
        """Test CoM with unequal masses."""
        analyzer = StabilityAnalyzer()
        
        link_positions = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
        ])
        link_masses = np.array([30.0, 10.0])
        
        com = analyzer.compute_center_of_mass(link_positions, link_masses)
        
        # Weighted average: (30*0 + 10*1) / 40 = 0.25
        assert np.allclose(com, [0.25, 0.0, 0.0], atol=1e-6)
    
    def test_static_stability_margin(self):
        """Test static stability margin calculation."""
        analyzer = StabilityAnalyzer()
        
        com_pos = np.array([0.0, 0.0, 1.0])
        support_polygon = np.array([
            [0.2, 0.2],
            [0.2, -0.2],
            [-0.2, -0.2],
            [-0.2, 0.2]
        ])
        
        margin = analyzer.compute_static_stability_margin(com_pos, support_polygon)
        
        # CoM at center, margin should be ~0.2m (distance to edge)
        assert margin > 0.0
        assert margin < 0.3
    
    def test_dynamic_stability(self):
        """Test dynamic stability calculation."""
        analyzer = StabilityAnalyzer()
        
        com_pos = np.array([0.0, 0.0, 1.0])
        com_vel = np.array([0.1, 0.0, 0.0])
        com_acc = np.array([0.0, 0.0, 0.0])
        
        support_polygon = np.array([
            [0.2, 0.2],
            [0.2, -0.2],
            [-0.2, -0.2],
            [-0.2, 0.2]
        ])
        
        stable, margin = analyzer.compute_dynamic_stability(
            com_pos, com_vel, com_acc, support_polygon
        )
        
        assert isinstance(stable, bool)
        assert isinstance(margin, float)
    
    def test_capture_point(self):
        """Test capture point calculation."""
        analyzer = StabilityAnalyzer()
        
        com_pos = np.array([0.0, 0.0, 1.0])
        com_vel = np.array([0.5, 0.0, 0.0])  # Moving forward
        
        cp = analyzer.compute_capture_point(com_pos, com_vel)
        
        # Capture point should be ahead of CoM
        assert cp[0] > com_pos[0]
        assert cp.shape == (2,)
    
    def test_trajectory_stability(self):
        """Test stability checking along trajectory."""
        analyzer = StabilityAnalyzer()
        
        # Simple trajectory
        com_trajectory = np.array([
            [0.0, 0.0, 1.0],
            [0.05, 0.0, 1.0],
            [0.1, 0.0, 1.0]
        ])
        
        com_velocities = np.array([
            [0.5, 0.0, 0.0],
            [0.5, 0.0, 0.0]
        ])
        
        com_accelerations = np.array([
            [0.0, 0.0, 0.0]
        ])
        
        support_polygon = np.array([
            [0.2, 0.2],
            [0.2, -0.2],
            [-0.2, -0.2],
            [-0.2, 0.2]
        ])
        
        support_polygons = [support_polygon] * 3
        
        all_stable, margins = analyzer.check_trajectory_stability(
            com_trajectory, com_velocities, com_accelerations, support_polygons
        )
        
        assert isinstance(all_stable, bool)
        assert len(margins) == len(com_trajectory)
    
    def test_stability_metrics(self):
        """Test comprehensive stability metrics."""
        analyzer = StabilityAnalyzer()
        
        com_pos = np.array([0.0, 0.0, 1.0])
        com_vel = np.array([0.1, 0.0, 0.0])
        com_acc = np.array([0.0, 0.0, 0.0])
        
        support_polygon = np.array([
            [0.2, 0.2],
            [0.2, -0.2],
            [-0.2, -0.2],
            [-0.2, 0.2]
        ])
        
        metrics = analyzer.compute_stability_metrics(
            com_pos, com_vel, com_acc, support_polygon
        )
        
        # Check all expected keys
        assert 'static_margin' in metrics
        assert 'dynamic_margin' in metrics
        assert 'is_stable' in metrics
        assert 'zmp' in metrics
        assert 'capture_point' in metrics
        assert 'com_height' in metrics
        assert np.allclose(metrics['com_height'], 1.0)


class TestCenterOfMassTracker:
    """Test CoM tracking functionality."""
    
    def test_initialization(self):
        """Test tracker initialization."""
        tracker = CenterOfMassTracker(dt=0.01)
        
        assert tracker.dt == 0.01
        assert len(tracker.history) == 0
    
    def test_update(self):
        """Test CoM state update."""
        tracker = CenterOfMassTracker()
        
        com_pos = np.array([0.0, 0.0, 1.0])
        tracker.update(com_pos)
        
        assert len(tracker.history) == 1
        assert np.allclose(tracker.history[0]['position'], com_pos)
    
    def test_velocity_estimation(self):
        """Test velocity estimation from history."""
        tracker = CenterOfMassTracker(dt=0.1)
        
        # Add positions with constant velocity
        for i in range(5):
            com_pos = np.array([i * 0.1, 0.0, 1.0])
            tracker.update(com_pos, timestamp=i * 0.1)
        
        velocity = tracker.get_velocity()
        
        # Velocity should be ~1.0 m/s in x direction
        assert velocity.shape == (3,)
        assert np.allclose(velocity[0], 1.0, atol=0.1)
        assert np.allclose(velocity[1], 0.0, atol=0.1)
    
    def test_acceleration_estimation(self):
        """Test acceleration estimation."""
        tracker = CenterOfMassTracker(dt=0.1)
        
        # Add positions with constant acceleration
        for i in range(10):
            t = i * 0.1
            # x = 0.5 * a * t^2, with a = 1.0
            x = 0.5 * 1.0 * t**2
            com_pos = np.array([x, 0.0, 1.0])
            tracker.update(com_pos, timestamp=t)
        
        acceleration = tracker.get_acceleration()
        
        # Should estimate acceleration ~1.0
        assert acceleration.shape == (3,)
        # Finite differences have numerical error
        assert np.abs(acceleration[0] - 1.0) < 0.5
    
    def test_get_trajectory(self):
        """Test getting full trajectory."""
        tracker = CenterOfMassTracker()
        
        positions = [
            np.array([0.0, 0.0, 1.0]),
            np.array([0.1, 0.0, 1.0]),
            np.array([0.2, 0.0, 1.0])
        ]
        
        for pos in positions:
            tracker.update(pos)
        
        trajectory = tracker.get_trajectory()
        
        assert trajectory.shape == (3, 3)
        assert np.allclose(trajectory, positions)
    
    def test_clear(self):
        """Test clearing history."""
        tracker = CenterOfMassTracker()
        
        tracker.update(np.array([0.0, 0.0, 1.0]))
        tracker.update(np.array([0.1, 0.0, 1.0]))
        
        assert len(tracker.history) == 2
        
        tracker.clear()
        
        assert len(tracker.history) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
