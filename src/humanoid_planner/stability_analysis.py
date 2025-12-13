"""
Stability Analysis Module

Provides comprehensive stability analysis for humanoid robots including:
- Center of Mass (CoM) computation
- ZMP calculation and verification
- Static stability margins
- Dynamic stability criteria
"""

import numpy as np
from typing import Tuple, Optional, List
from .zmp_constraint import ZMPConstraint
from .support_polygon import SupportPolygonCalculator


class StabilityAnalyzer:
    """Analyzes robot stability using ZMP and CoM metrics."""
    
    def __init__(self, 
                 gravity: float = 9.81,
                 zmp_margin: float = 0.02):
        """
        Initialize stability analyzer.
        
        Args:
            gravity: Gravitational acceleration (m/s^2)
            zmp_margin: Safety margin from support polygon edge (meters)
        """
        self.gravity = gravity
        self.zmp_constraint = ZMPConstraint(margin=zmp_margin, gravity=gravity)
        
    def compute_center_of_mass(self,
                               link_positions: np.ndarray,
                               link_masses: np.ndarray) -> np.ndarray:
        """
        Compute robot's center of mass.
        
        Args:
            link_positions: Positions of robot links (N x 3)
            link_masses: Mass of each link (N,)
            
        Returns:
            com: Center of mass position (3,) [x, y, z]
        """
        total_mass = np.sum(link_masses)
        if total_mass < 1e-6:
            return np.zeros(3)
        
        com = np.sum(link_positions * link_masses[:, np.newaxis], axis=0) / total_mass
        return com
    
    def compute_com_jacobian(self,
                            link_jacobians: List[np.ndarray],
                            link_masses: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian of center of mass.
        
        Args:
            link_jacobians: List of link Jacobians (each 6 x num_joints)
            link_masses: Mass of each link (N,)
            
        Returns:
            J_com: CoM Jacobian (3 x num_joints)
        """
        total_mass = np.sum(link_masses)
        if total_mass < 1e-6:
            return np.zeros((3, link_jacobians[0].shape[1]))
        
        # Linear velocity part of Jacobian
        J_com = np.zeros((3, link_jacobians[0].shape[1]))
        
        for i, (J_link, mass) in enumerate(zip(link_jacobians, link_masses)):
            # Add weighted contribution of each link
            J_com += mass * J_link[:3, :] / total_mass
        
        return J_com
    
    def compute_static_stability_margin(self,
                                       com_pos: np.ndarray,
                                       support_polygon: np.ndarray) -> float:
        """
        Compute static stability margin.
        
        The margin is the minimum distance from CoM projection to support polygon edges.
        
        Args:
            com_pos: Center of mass position (3,)
            support_polygon: Support polygon vertices (N x 2)
            
        Returns:
            margin: Stability margin (positive = stable)
        """
        com_2d = com_pos[:2]
        return self.zmp_constraint.compute_stability_margin(com_2d, support_polygon)
    
    def compute_dynamic_stability(self,
                                 com_pos: np.ndarray,
                                 com_vel: np.ndarray,
                                 com_acc: np.ndarray,
                                 support_polygon: np.ndarray) -> Tuple[bool, float]:
        """
        Compute dynamic stability using ZMP criterion.
        
        Args:
            com_pos: CoM position (3,)
            com_vel: CoM velocity (3,)
            com_acc: CoM acceleration (3,)
            support_polygon: Support polygon vertices (N x 2)
            
        Returns:
            stable: True if dynamically stable
            margin: Stability margin (distance to polygon edge)
        """
        # Compute ZMP from CoM dynamics
        height = com_pos[2]
        zmp = self.zmp_constraint.compute_zmp_from_com(com_pos, com_acc, height)
        
        # Check if ZMP is within support polygon
        stable = self.zmp_constraint.is_stable(zmp, support_polygon)
        margin = self.zmp_constraint.compute_stability_margin(zmp, support_polygon)
        
        return stable, margin
    
    def compute_capture_point(self,
                             com_pos: np.ndarray,
                             com_vel: np.ndarray) -> np.ndarray:
        """
        Compute capture point (also called extrapolated CoM).
        
        The capture point is where the robot would need to step to come to rest.
        
        Args:
            com_pos: CoM position (3,)
            com_vel: CoM velocity (3,)
            
        Returns:
            capture_point: Capture point position (2,) [x, y]
        """
        height = com_pos[2]
        omega = np.sqrt(self.gravity / height)  # Natural frequency
        
        # Capture point formula: CP = CoM + CoM_vel / omega
        cp_x = com_pos[0] + com_vel[0] / omega
        cp_y = com_pos[1] + com_vel[1] / omega
        
        return np.array([cp_x, cp_y])
    
    def check_trajectory_stability(self,
                                  com_trajectory: np.ndarray,
                                  com_velocities: np.ndarray,
                                  com_accelerations: np.ndarray,
                                  support_polygons: List[np.ndarray]) -> Tuple[bool, List[float]]:
        """
        Check stability along entire trajectory.
        
        Args:
            com_trajectory: CoM positions over time (T x 3)
            com_velocities: CoM velocities over time (T x 3)
            com_accelerations: CoM accelerations over time (T x 3)
            support_polygons: Support polygons at each timestep (list of N x 2 arrays)
            
        Returns:
            all_stable: True if stable throughout trajectory
            margins: Stability margins at each timestep
        """
        margins = []
        all_stable = True
        
        for i in range(len(com_trajectory)):
            com_pos = com_trajectory[i]
            com_vel = com_velocities[i] if i < len(com_velocities) else np.zeros(3)
            com_acc = com_accelerations[i] if i < len(com_accelerations) else np.zeros(3)
            support_poly = support_polygons[min(i, len(support_polygons) - 1)]
            
            stable, margin = self.compute_dynamic_stability(
                com_pos, com_vel, com_acc, support_poly
            )
            
            margins.append(margin)
            if not stable:
                all_stable = False
        
        return all_stable, margins
    
    def compute_tipping_axis(self,
                            com_pos: np.ndarray,
                            support_polygon: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Compute the support polygon edge closest to tipping.
        
        Args:
            com_pos: CoM position (3,)
            support_polygon: Support polygon vertices (N x 2)
            
        Returns:
            edge_start: Start of critical edge (2,) or None
            edge_end: End of critical edge (2,) or None
        """
        com_2d = com_pos[:2]
        
        # Find edge with minimum distance to CoM projection
        min_dist = float('inf')
        critical_edge = None
        
        for i in range(len(support_polygon)):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % len(support_polygon)]
            
            dist = self.zmp_constraint._point_to_line_distance(com_2d, p1, p2)
            
            if dist < min_dist:
                min_dist = dist
                critical_edge = (p1, p2)
        
        if critical_edge is None:
            return None, None
        
        return critical_edge[0], critical_edge[1]
    
    def compute_stability_metrics(self,
                                 com_pos: np.ndarray,
                                 com_vel: np.ndarray,
                                 com_acc: np.ndarray,
                                 support_polygon: np.ndarray) -> dict:
        """
        Compute comprehensive stability metrics.
        
        Args:
            com_pos: CoM position (3,)
            com_vel: CoM velocity (3,)
            com_acc: CoM acceleration (3,)
            support_polygon: Support polygon vertices (N x 2)
            
        Returns:
            metrics: Dictionary of stability metrics
        """
        # Static stability
        static_margin = self.compute_static_stability_margin(com_pos, support_polygon)
        
        # Dynamic stability
        dynamic_stable, dynamic_margin = self.compute_dynamic_stability(
            com_pos, com_vel, com_acc, support_polygon
        )
        
        # Capture point
        capture_point = self.compute_capture_point(com_pos, com_vel)
        
        # ZMP
        height = com_pos[2]
        zmp = self.zmp_constraint.compute_zmp_from_com(com_pos, com_acc, height)
        
        # Tipping axis
        edge_start, edge_end = self.compute_tipping_axis(com_pos, support_polygon)
        
        return {
            'static_margin': static_margin,
            'dynamic_margin': dynamic_margin,
            'is_stable': dynamic_stable,
            'zmp': zmp,
            'capture_point': capture_point,
            'com_height': com_pos[2],
            'com_2d': com_pos[:2],
            'tipping_edge_start': edge_start,
            'tipping_edge_end': edge_end
        }


class CenterOfMassTracker:
    """Tracks center of mass state over time."""
    
    def __init__(self, dt: float = 0.01):
        """
        Initialize CoM tracker.
        
        Args:
            dt: Time step (seconds)
        """
        self.dt = dt
        self.history = []
        
    def update(self, com_pos: np.ndarray, timestamp: Optional[float] = None):
        """
        Update CoM state.
        
        Args:
            com_pos: Current CoM position (3,)
            timestamp: Optional timestamp
        """
        if timestamp is None:
            if len(self.history) > 0:
                timestamp = self.history[-1]['timestamp'] + self.dt
            else:
                timestamp = 0.0
        
        self.history.append({
            'timestamp': timestamp,
            'position': com_pos.copy(),
        })
    
    def get_velocity(self, window_size: int = 5) -> np.ndarray:
        """
        Estimate CoM velocity using finite differences.
        
        Args:
            window_size: Number of samples for smoothing
            
        Returns:
            velocity: Estimated velocity (3,)
        """
        if len(self.history) < 2:
            return np.zeros(3)
        
        # Use last few samples for smoother estimate
        n = min(window_size, len(self.history))
        positions = np.array([self.history[-i]['position'] for i in range(n, 0, -1)])
        times = np.array([self.history[-i]['timestamp'] for i in range(n, 0, -1)])
        
        # Linear regression for velocity
        dt = times[-1] - times[0]
        if dt < 1e-6:
            return np.zeros(3)
        
        velocity = (positions[-1] - positions[0]) / dt
        return velocity
    
    def get_acceleration(self, window_size: int = 5) -> np.ndarray:
        """
        Estimate CoM acceleration using finite differences.
        
        Args:
            window_size: Number of samples for smoothing
            
        Returns:
            acceleration: Estimated acceleration (3,)
        """
        if len(self.history) < 3:
            return np.zeros(3)
        
        # Use last few samples
        n = min(window_size, len(self.history))
        positions = np.array([self.history[-i]['position'] for i in range(n, 0, -1)])
        
        # Second-order finite difference
        if n >= 3:
            acc = (positions[-1] - 2*positions[-2] + positions[-3]) / (self.dt ** 2)
            return acc
        
        return np.zeros(3)
    
    def clear(self):
        """Clear history."""
        self.history = []
    
    def get_trajectory(self) -> np.ndarray:
        """
        Get full CoM trajectory.
        
        Returns:
            trajectory: Array of positions (T x 3)
        """
        if len(self.history) == 0:
            return np.array([])
        
        return np.array([h['position'] for h in self.history])
