"""
Zero Moment Point (ZMP) Constraint Module

Handles ZMP computation and stability verification for humanoid robots.
"""

import numpy as np
from typing import Tuple, Optional


class ZMPConstraint:
    """Handles Zero Moment Point constraint formulation and checking."""
    
    def __init__(self, margin: float = 0.02, gravity: float = 9.81):
        """
        Initialize ZMP constraint.
        
        Args:
            margin: Safety margin from support polygon edge (meters)
            gravity: Gravitational acceleration (m/s^2)
        """
        self.margin = margin
        self.gravity = gravity
        
    def compute_zmp(self, 
                    forces: np.ndarray, 
                    positions: np.ndarray) -> np.ndarray:
        """
        Compute Zero Moment Point from ground reaction forces.
        
        The ZMP is the point on the ground where the resultant of ground
        reaction forces would have zero moment.
        
        Args:
            forces: Ground reaction forces at each contact (N x 3)
                   Each row is [fx, fy, fz] in world frame
            positions: Contact point positions in world frame (N x 3)
                      Each row is [x, y, z]
            
        Returns:
            zmp: Zero Moment Point position (2,) as [x, y]
        """
        # Extract vertical forces
        fz = forces[:, 2]
        total_force = np.sum(fz)
        
        if total_force < 1e-6:
            # No ground contact, return origin
            return np.array([0.0, 0.0])
        
        # ZMP calculation: ZMP = Σ(f_zi * r_i) / Σ(f_zi)
        zmp_x = np.sum(fz * positions[:, 0]) / total_force
        zmp_y = np.sum(fz * positions[:, 1]) / total_force
        
        return np.array([zmp_x, zmp_y])
    
    def compute_zmp_from_com(self,
                            com_pos: np.ndarray,
                            com_acc: np.ndarray,
                            height: float) -> np.ndarray:
        """
        Compute ZMP from center of mass dynamics.
        
        Using the simplified model: ZMP = CoM_xy - (h/g) * CoM_acc_xy
        
        Args:
            com_pos: Center of mass position (3,) [x, y, z]
            com_acc: Center of mass acceleration (3,) [ax, ay, az]
            height: Height of CoM above ground
            
        Returns:
            zmp: ZMP position (2,) as [x, y]
        """
        zmp_x = com_pos[0] - (height / self.gravity) * com_acc[0]
        zmp_y = com_pos[1] - (height / self.gravity) * com_acc[1]
        
        return np.array([zmp_x, zmp_y])
    
    def is_stable(self, 
                  zmp: np.ndarray, 
                  support_polygon: np.ndarray) -> bool:
        """
        Check if ZMP is within support polygon.
        
        Args:
            zmp: Zero Moment Point (2,) [x, y]
            support_polygon: Vertices of support polygon (N x 2)
                           Vertices should be in counter-clockwise order
            
        Returns:
            stable: True if ZMP is inside polygon with safety margin
        """
        if len(support_polygon) < 3:
            # Degenerate polygon
            return False
        
        # Shrink polygon by margin for safety
        center = np.mean(support_polygon, axis=0)
        shrunk_polygon = center + (support_polygon - center) * (1 - self.margin / np.linalg.norm(support_polygon[0] - center))
        
        # Point-in-polygon test using winding number
        return self._point_in_polygon(zmp, shrunk_polygon)
    
    def _point_in_polygon(self, point: np.ndarray, polygon: np.ndarray) -> bool:
        """
        Test if a point is inside a polygon using ray casting algorithm.
        
        Args:
            point: Test point (2,)
            polygon: Polygon vertices (N x 2)
            
        Returns:
            inside: True if point is inside polygon
        """
        x, y = point
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def compute_stability_margin(self,
                                 zmp: np.ndarray,
                                 support_polygon: np.ndarray) -> float:
        """
        Compute distance from ZMP to nearest edge of support polygon.
        
        A positive value indicates stable configuration.
        
        Args:
            zmp: Zero Moment Point (2,)
            support_polygon: Polygon vertices (N x 2)
            
        Returns:
            margin: Distance to nearest edge (positive = inside, negative = outside)
        """
        if not self.is_stable(zmp, support_polygon):
            # Point is outside, return negative distance
            min_dist = float('inf')
            for i in range(len(support_polygon)):
                p1 = support_polygon[i]
                p2 = support_polygon[(i + 1) % len(support_polygon)]
                dist = self._point_to_line_distance(zmp, p1, p2)
                min_dist = min(min_dist, dist)
            return -min_dist
        
        # Point is inside, find minimum distance to edges
        min_dist = float('inf')
        for i in range(len(support_polygon)):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % len(support_polygon)]
            dist = self._point_to_line_distance(zmp, p1, p2)
            min_dist = min(min_dist, dist)
        
        return min_dist
    
    def _point_to_line_distance(self,
                                point: np.ndarray,
                                line_start: np.ndarray,
                                line_end: np.ndarray) -> float:
        """
        Compute perpendicular distance from point to line segment.
        
        Args:
            point: Test point (2,)
            line_start: Start of line segment (2,)
            line_end: End of line segment (2,)
            
        Returns:
            distance: Perpendicular distance to line segment
        """
        # Vector from line_start to line_end
        line_vec = line_end - line_start
        line_len = np.linalg.norm(line_vec)
        
        if line_len < 1e-10:
            # Degenerate line segment
            return np.linalg.norm(point - line_start)
        
        # Vector from line_start to point
        point_vec = point - line_start
        
        # Project point onto line
        t = np.dot(point_vec, line_vec) / (line_len ** 2)
        t = np.clip(t, 0.0, 1.0)
        
        # Find closest point on line segment
        closest = line_start + t * line_vec
        
        return np.linalg.norm(point - closest)
    
    def generate_constraint_matrix(self,
                                   support_polygon: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate linear inequality constraints: A @ zmp <= b
        
        These constraints ensure ZMP stays within support polygon.
        
        Args:
            support_polygon: Polygon vertices (N x 2)
            
        Returns:
            A: Constraint matrix (N x 2)
            b: Constraint vector (N,)
        """
        n = len(support_polygon)
        A = np.zeros((n, 2))
        b = np.zeros(n)
        
        for i in range(n):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % n]
            
            # Normal vector pointing inward
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])
            normal = normal / np.linalg.norm(normal)
            
            # Check if normal points inward
            center = np.mean(support_polygon, axis=0)
            if np.dot(normal, center - p1) < 0:
                normal = -normal
            
            A[i] = -normal
            b[i] = -np.dot(normal, p1) - self.margin
        
        return A, b
