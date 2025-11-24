"""
Support Polygon Calculation Module

Computes support polygons from foot contact points for stability analysis.
"""

import numpy as np
from scipy.spatial import ConvexHull
from typing import List, Optional


class SupportPolygonCalculator:
    """Computes support polygon from foot contact points."""
    
    def __init__(self, foot_size: tuple = (0.2, 0.1)):
        """
        Initialize support polygon calculator.
        
        Args:
            foot_size: (length, width) of rectangular foot in meters
        """
        self.foot_length = foot_size[0]
        self.foot_width = foot_size[1]
    
    def compute_support_polygon(self, foot_contacts: np.ndarray) -> np.ndarray:
        """
        Compute support polygon as convex hull of foot contacts.
        
        Args:
            foot_contacts: Contact points on ground plane (N x 3)
                          Each row is [x, y, z] in world frame
            
        Returns:
            vertices: Ordered vertices of support polygon (M x 2)
                     Vertices in counter-clockwise order
        """
        if len(foot_contacts) < 3:
            # Degenerate case - return points directly
            return foot_contacts[:, :2]
        
        # Project onto ground plane (x-y)
        projected = foot_contacts[:, :2]
        
        try:
            # Compute convex hull
            hull = ConvexHull(projected)
            
            # Return vertices in counter-clockwise order
            vertices = projected[hull.vertices]
            
            # Ensure counter-clockwise ordering
            vertices = self._ensure_ccw(vertices)
            
            return vertices
            
        except Exception as e:
            # If convex hull fails, return original points
            print(f"Warning: ConvexHull failed: {e}")
            return projected
    
    def _ensure_ccw(self, vertices: np.ndarray) -> np.ndarray:
        """
        Ensure vertices are in counter-clockwise order.
        
        Args:
            vertices: Polygon vertices (N x 2)
            
        Returns:
            vertices: Reordered vertices in CCW order
        """
        # Compute signed area
        area = 0.0
        n = len(vertices)
        for i in range(n):
            j = (i + 1) % n
            area += vertices[i, 0] * vertices[j, 1]
            area -= vertices[j, 0] * vertices[i, 1]
        
        if area < 0:
            # Clockwise, reverse order
            return vertices[::-1]
        return vertices
    
    def get_foot_corners(self, foot_pose: np.ndarray) -> np.ndarray:
        """
        Get corner points of a rectangular foot.
        
        Args:
            foot_pose: Foot pose as [x, y, z, roll, pitch, yaw]
            
        Returns:
            corners: Four corner points (4 x 3)
        """
        x, y, z, roll, pitch, yaw = foot_pose
        
        # Define corners in foot frame
        half_l = self.foot_length / 2
        half_w = self.foot_width / 2
        
        corners_local = np.array([
            [half_l, half_w, 0],
            [half_l, -half_w, 0],
            [-half_l, half_w, 0],
            [-half_l, -half_w, 0]
        ])
        
        # Rotation matrix (simplified - only yaw)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        R = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])
        
        # Transform to world frame
        corners_world = corners_local @ R.T + np.array([x, y, z])
        
        return corners_world
    
    def compute_double_support_polygon(self,
                                      left_foot_pose: np.ndarray,
                                      right_foot_pose: np.ndarray) -> np.ndarray:
        """
        Compute support polygon for double support phase.
        
        Args:
            left_foot_pose: Left foot pose [x, y, z, roll, pitch, yaw]
            right_foot_pose: Right foot pose [x, y, z, roll, pitch, yaw]
            
        Returns:
            vertices: Support polygon vertices (N x 2)
        """
        # Get corners of both feet
        left_corners = self.get_foot_corners(left_foot_pose)
        right_corners = self.get_foot_corners(right_foot_pose)
        
        # Combine all contact points
        all_contacts = np.vstack([left_corners, right_corners])
        
        # Compute convex hull
        return self.compute_support_polygon(all_contacts)
    
    def compute_single_support_polygon(self, foot_pose: np.ndarray) -> np.ndarray:
        """
        Compute support polygon for single support phase.
        
        Args:
            foot_pose: Foot pose [x, y, z, roll, pitch, yaw]
            
        Returns:
            vertices: Support polygon vertices (4 x 2) for rectangular foot
        """
        corners = self.get_foot_corners(foot_pose)
        return corners[:, :2]
    
    def visualize_polygon(self, polygon: np.ndarray, zmp: Optional[np.ndarray] = None):
        """
        Visualize support polygon and optionally ZMP.
        
        Args:
            polygon: Polygon vertices (N x 2)
            zmp: Optional ZMP point (2,)
        """
        import matplotlib.pyplot as plt
        
        # Close the polygon
        polygon_closed = np.vstack([polygon, polygon[0]])
        
        plt.figure(figsize=(8, 8))
        plt.plot(polygon_closed[:, 0], polygon_closed[:, 1], 'b-', linewidth=2, label='Support Polygon')
        plt.fill(polygon[:, 0], polygon[:, 1], alpha=0.3, color='blue')
        
        if zmp is not None:
            plt.plot(zmp[0], zmp[1], 'ro', markersize=10, label='ZMP')
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Support Polygon')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
    
    def compute_polygon_area(self, polygon: np.ndarray) -> float:
        """
        Compute area of polygon using shoelace formula.
        
        Args:
            polygon: Polygon vertices (N x 2)
            
        Returns:
            area: Area in square meters
        """
        n = len(polygon)
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += polygon[i, 0] * polygon[j, 1]
            area -= polygon[j, 0] * polygon[i, 1]
        return abs(area) / 2.0
    
    def compute_polygon_centroid(self, polygon: np.ndarray) -> np.ndarray:
        """
        Compute centroid of polygon.
        
        Args:
            polygon: Polygon vertices (N x 2)
            
        Returns:
            centroid: Centroid position (2,)
        """
        return np.mean(polygon, axis=0)
