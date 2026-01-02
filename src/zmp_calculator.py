"""
Zero Moment Point (ZMP) Calculator

The ZMP is the point on the ground where the sum of horizontal moments
due to gravity and inertia equals zero. For stable standing/walking,
the ZMP must remain inside the support polygon.

Physics:
    For a robot with CoM at position p_c and acceleration a_c:
    
    ZMP_x = p_c_x - (p_c_z * a_c_x) / (a_c_z + g)
    ZMP_y = p_c_y - (p_c_z * a_c_y) / (a_c_z + g)
    
    Where g = 9.81 m/s² (gravity)
    
For quasi-static motion (slow movements where accelerations are small):
    ZMP ≈ projection of CoM onto ground plane
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class ZMPState:
    """ZMP computation result"""
    zmp: np.ndarray              # ZMP position [x, y] on ground
    com: np.ndarray              # CoM position [x, y, z]
    com_acceleration: np.ndarray # CoM acceleration [ax, ay, az]
    is_stable: bool              # Whether ZMP is inside support polygon
    stability_margin: float      # Distance from ZMP to nearest polygon edge (negative if outside)
    nearest_edge_point: np.ndarray  # Closest point on polygon boundary


class ZMPCalculator:
    """
    Computes Zero Moment Point and stability metrics.
    """
    
    GRAVITY = 9.81  # m/s²
    
    def __init__(self, safety_margin: float = 0.02):
        """
        Args:
            safety_margin: Minimum distance (m) ZMP should be from polygon edge
        """
        self.safety_margin = safety_margin
    
    def compute_zmp(
        self,
        com: np.ndarray,
        com_acceleration: np.ndarray = None
    ) -> np.ndarray:
        """
        Compute ZMP position.
        
        Args:
            com: Center of mass position [x, y, z] in meters
            com_acceleration: CoM acceleration [ax, ay, az] in m/s². 
                            If None, assumes quasi-static (zero acceleration).
        
        Returns:
            zmp: [x, y] position on ground plane
        """
        if com_acceleration is None:
            # Quasi-static: ZMP is directly below CoM
            return com[:2].copy()
        
        ax, ay, az = com_acceleration
        px, py, pz = com
        
        # ZMP formula from Newton-Euler equations
        # Denominator is vertical acceleration plus gravity
        denom = az + self.GRAVITY
        
        # Avoid division by zero (shouldn't happen in normal operation)
        if abs(denom) < 1e-6:
            denom = 1e-6
        
        zmp_x = px - (pz * ax) / denom
        zmp_y = py - (pz * ay) / denom
        
        return np.array([zmp_x, zmp_y])
    
    def point_in_polygon(self, point: np.ndarray, polygon: np.ndarray) -> bool:
        """
        Check if a point is inside a convex polygon using cross product method.
        
        Args:
            point: [x, y] point to test
            polygon: (N x 2) array of polygon vertices in order
            
        Returns:
            True if point is inside polygon
        """
        n = len(polygon)
        
        for i in range(n):
            # Edge from vertex i to vertex (i+1) % n
            v1 = polygon[i]
            v2 = polygon[(i + 1) % n]
            
            # Vector along edge
            edge = v2 - v1
            # Vector from v1 to point
            to_point = point - v1
            
            # Cross product (z-component of 3D cross product)
            cross = edge[0] * to_point[1] - edge[1] * to_point[0]
            
            # All cross products should have same sign for point inside convex polygon
            if i == 0:
                sign = cross > 0
            elif (cross > 0) != sign:
                return False
        
        return True
    
    def distance_to_polygon_edge(
        self, 
        point: np.ndarray, 
        polygon: np.ndarray
    ) -> Tuple[float, np.ndarray]:
        """
        Compute signed distance from point to nearest polygon edge.
        
        Args:
            point: [x, y] point
            polygon: (N x 2) polygon vertices
            
        Returns:
            distance: Positive if inside, negative if outside
            nearest_point: Closest point on polygon boundary
        """
        n = len(polygon)
        min_dist = float('inf')
        nearest_point = None
        
        for i in range(n):
            v1 = polygon[i]
            v2 = polygon[(i + 1) % n]
            
            # Project point onto line segment
            edge = v2 - v1
            edge_length = np.linalg.norm(edge)
            
            if edge_length < 1e-10:
                continue
                
            edge_unit = edge / edge_length
            
            # Parameter along edge (0 = v1, 1 = v2)
            t = np.dot(point - v1, edge_unit)
            t = np.clip(t, 0, edge_length)
            
            # Closest point on edge
            closest = v1 + t * edge_unit
            dist = np.linalg.norm(point - closest)
            
            if dist < min_dist:
                min_dist = dist
                nearest_point = closest
        
        # Sign: positive if inside, negative if outside
        if self.point_in_polygon(point, polygon):
            return min_dist, nearest_point
        else:
            return -min_dist, nearest_point
    
    def compute_stability(
        self,
        com: np.ndarray,
        support_polygon: np.ndarray,
        com_acceleration: np.ndarray = None
    ) -> ZMPState:
        """
        Full ZMP stability analysis.
        
        Args:
            com: Center of mass [x, y, z]
            support_polygon: (N x 2) polygon vertices
            com_acceleration: Optional CoM acceleration
            
        Returns:
            ZMPState with all stability information
        """
        zmp = self.compute_zmp(com, com_acceleration)
        margin, nearest = self.distance_to_polygon_edge(zmp, support_polygon)
        is_stable = margin >= self.safety_margin
        
        return ZMPState(
            zmp=zmp,
            com=com,
            com_acceleration=com_acceleration if com_acceleration is not None else np.zeros(3),
            is_stable=is_stable,
            stability_margin=margin,
            nearest_edge_point=nearest
        )
    
    def check_trajectory_stability(
        self,
        com_trajectory: np.ndarray,
        support_polygon: np.ndarray,
        com_velocities: np.ndarray = None,
        dt: float = 0.01
    ) -> Tuple[bool, np.ndarray, int]:
        """
        Check stability along entire trajectory.
        
        Args:
            com_trajectory: (T x 3) CoM positions over time
            support_polygon: Support polygon (assumed constant)
            com_velocities: (T x 3) CoM velocities. If None, computed via finite diff.
            dt: Time step for acceleration computation
            
        Returns:
            all_stable: True if entire trajectory is stable
            margins: (T,) stability margins at each timestep
            first_violation_idx: Index of first instability (-1 if none)
        """
        T = len(com_trajectory)
        
        # Compute velocities if not provided
        if com_velocities is None:
            com_velocities = np.zeros_like(com_trajectory)
            com_velocities[1:] = (com_trajectory[1:] - com_trajectory[:-1]) / dt
            com_velocities[0] = com_velocities[1]
        
        # Compute accelerations via finite differences
        com_accelerations = np.zeros_like(com_trajectory)
        com_accelerations[1:] = (com_velocities[1:] - com_velocities[:-1]) / dt
        com_accelerations[0] = com_accelerations[1]
        
        margins = np.zeros(T)
        first_violation = -1
        
        for i in range(T):
            state = self.compute_stability(
                com_trajectory[i],
                support_polygon,
                com_accelerations[i]
            )
            margins[i] = state.stability_margin
            
            if not state.is_stable and first_violation == -1:
                first_violation = i
        
        all_stable = first_violation == -1
        return all_stable, margins, first_violation


# Visualization helper
def plot_zmp_state(
    zmp_state: ZMPState,
    support_polygon: np.ndarray,
    ax=None,
    title: str = "ZMP Stability"
):
    """Plot ZMP and support polygon"""
    import matplotlib.pyplot as plt
    
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))
    
    # Plot support polygon
    poly_closed = np.vstack([support_polygon, support_polygon[0]])
    ax.plot(poly_closed[:, 0], poly_closed[:, 1], 'b-', linewidth=2, label='Support Polygon')
    ax.fill(support_polygon[:, 0], support_polygon[:, 1], 'b', alpha=0.1)
    
    # Plot safety margin polygon (shrunk)
    if hasattr(zmp_state, 'safety_margin'):
        # Simple approximation: offset vertices inward
        center = np.mean(support_polygon, axis=0)
        margin = 0.02  # safety margin
        shrunk = center + (support_polygon - center) * 0.85  # rough approximation
        shrunk_closed = np.vstack([shrunk, shrunk[0]])
        ax.plot(shrunk_closed[:, 0], shrunk_closed[:, 1], 'g--', linewidth=1, label='Safety Boundary')
    
    # Plot CoM projection
    ax.plot(zmp_state.com[0], zmp_state.com[1], 'ko', markersize=10, label='CoM (projected)')
    
    # Plot ZMP
    color = 'g' if zmp_state.is_stable else 'r'
    ax.plot(zmp_state.zmp[0], zmp_state.zmp[1], color + 's', markersize=12, label=f'ZMP (margin={zmp_state.stability_margin:.3f}m)')
    
    # Draw line from ZMP to nearest edge
    ax.plot(
        [zmp_state.zmp[0], zmp_state.nearest_edge_point[0]],
        [zmp_state.zmp[1], zmp_state.nearest_edge_point[1]],
        'r--', linewidth=1
    )
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    return ax


# Test
if __name__ == '__main__':
    print("Testing ZMP Calculator...")
    
    calc = ZMPCalculator(safety_margin=0.02)
    
    # Test support polygon (from G1 standing)
    support_polygon = np.array([
        [0.05, -0.14],
        [0.05, 0.14],
        [-0.05, 0.14],
        [-0.05, -0.14]
    ])
    
    # Test 1: CoM directly above center (stable)
    print("\nTest 1: Centered CoM")
    com1 = np.array([0.0, 0.0, 0.7])
    state1 = calc.compute_stability(com1, support_polygon)
    print(f"  ZMP: [{state1.zmp[0]:.4f}, {state1.zmp[1]:.4f}]")
    print(f"  Stable: {state1.is_stable}")
    print(f"  Margin: {state1.stability_margin:.4f} m")
    
    # Test 2: CoM shifted forward (still stable but less margin)
    print("\nTest 2: Forward-shifted CoM")
    com2 = np.array([0.03, 0.0, 0.7])
    state2 = calc.compute_stability(com2, support_polygon)
    print(f"  ZMP: [{state2.zmp[0]:.4f}, {state2.zmp[1]:.4f}]")
    print(f"  Stable: {state2.is_stable}")
    print(f"  Margin: {state2.stability_margin:.4f} m")
    
    # Test 3: CoM with acceleration (simulating arm movement)
    print("\nTest 3: CoM with lateral acceleration")
    com3 = np.array([0.0, 0.0, 0.7])
    acc3 = np.array([0.0, 2.0, 0.0])  # Accelerating sideways
    state3 = calc.compute_stability(com3, support_polygon, acc3)
    print(f"  ZMP: [{state3.zmp[0]:.4f}, {state3.zmp[1]:.4f}]")
    print(f"  Stable: {state3.is_stable}")
    print(f"  Margin: {state3.stability_margin:.4f} m")
    
    # Test 4: Unstable case
    print("\nTest 4: Unstable (CoM outside polygon)")
    com4 = np.array([0.1, 0.0, 0.7])  # Too far forward
    state4 = calc.compute_stability(com4, support_polygon)
    print(f"  ZMP: [{state4.zmp[0]:.4f}, {state4.zmp[1]:.4f}]")
    print(f"  Stable: {state4.is_stable}")
    print(f"  Margin: {state4.stability_margin:.4f} m")  # Should be negative
    
    # Plot results
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    
    plot_zmp_state(state1, support_polygon, axes[0, 0], "Test 1: Centered")
    plot_zmp_state(state2, support_polygon, axes[0, 1], "Test 2: Forward Shifted")
    plot_zmp_state(state3, support_polygon, axes[1, 0], "Test 3: With Acceleration")
    plot_zmp_state(state4, support_polygon, axes[1, 1], "Test 4: Unstable")
    
    plt.tight_layout()
    plt.savefig('zmp_test_results.png', dpi=150)
    print("\nPlot saved to zmp_test_results.png")
    
    print("\nZMP Calculator tests complete!")
