"""
Dynamic ZMP Calculator

Extends the basic ZMP calculator to handle dynamic motions:
- Computes ZMP from CoM trajectory with accelerations
- Predicts ZMP trajectory for fast motions
- Provides stability margins over entire trajectory

For quasi-static motion: ZMP ≈ CoM projection
For dynamic motion: ZMP = CoM - (h/g) * CoM_acceleration

where h is CoM height and g is gravity.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from scipy.signal import savgol_filter


@dataclass
class DynamicZMPState:
    """State of ZMP at a single time instant"""
    time: float
    com: np.ndarray           # [x, y, z] CoM position
    com_vel: np.ndarray       # [x, y, z] CoM velocity
    com_acc: np.ndarray       # [x, y, z] CoM acceleration
    zmp: np.ndarray           # [x, y] ZMP position
    stability_margin: float   # Distance to support polygon edge
    is_stable: bool


@dataclass
class DynamicZMPTrajectory:
    """ZMP analysis over entire trajectory"""
    times: np.ndarray
    com_positions: np.ndarray      # (N, 3)
    com_velocities: np.ndarray     # (N, 3)
    com_accelerations: np.ndarray  # (N, 3)
    zmp_positions: np.ndarray      # (N, 2)
    stability_margins: np.ndarray  # (N,)
    is_stable: np.ndarray          # (N,) boolean
    min_margin: float
    min_margin_time: float
    all_stable: bool


class DynamicZMPCalculator:
    """
    Computes Zero Moment Point for dynamic humanoid motions.
    
    The ZMP is the point on the ground where the total moment of
    all forces (gravity + inertia) is zero. For a humanoid to be
    stable, ZMP must stay inside the support polygon.
    
    ZMP equations:
        ZMP_x = CoM_x - (CoM_z * acc_x) / (acc_z + g)
        ZMP_y = CoM_y - (CoM_z * acc_y) / (acc_z + g)
    
    For quasi-static (slow) motion where acc ≈ 0:
        ZMP ≈ CoM projection onto ground
    """
    
    def __init__(self, gravity: float = 9.81):
        self.g = gravity
    
    def compute_zmp(
        self,
        com: np.ndarray,
        com_acceleration: np.ndarray = None
    ) -> np.ndarray:
        """
        Compute ZMP from CoM position and acceleration.
        
        Args:
            com: [x, y, z] Center of mass position
            com_acceleration: [ax, ay, az] CoM acceleration (optional)
            
        Returns:
            [zmp_x, zmp_y] ZMP position on ground plane
        """
        if com_acceleration is None:
            # Quasi-static: ZMP = CoM projection
            return com[:2].copy()
        
        ax, ay, az = com_acceleration
        px, py, pz = com
        
        # Avoid division by zero
        denominator = az + self.g
        if abs(denominator) < 1e-6:
            denominator = 1e-6 * np.sign(denominator) if denominator != 0 else 1e-6
        
        zmp_x = px - (pz * ax) / denominator
        zmp_y = py - (pz * ay) / denominator
        
        return np.array([zmp_x, zmp_y])
    
    def compute_com_derivatives(
        self,
        com_trajectory: np.ndarray,
        times: np.ndarray,
        smooth: bool = True
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute CoM velocity and acceleration from position trajectory.
        
        Args:
            com_trajectory: (N, 3) CoM positions over time
            times: (N,) time stamps
            smooth: Apply Savitzky-Golay filter for smoother derivatives
            
        Returns:
            velocities (N, 3), accelerations (N, 3)
        """
        N = len(times)
        
        if N < 3:
            return np.zeros_like(com_trajectory), np.zeros_like(com_trajectory)
        
        # Compute dt
        dt = np.diff(times)
        
        # Velocity via central differences
        velocities = np.zeros_like(com_trajectory)
        for i in range(3):
            velocities[1:-1, i] = (com_trajectory[2:, i] - com_trajectory[:-2, i]) / (times[2:] - times[:-2])
            velocities[0, i] = (com_trajectory[1, i] - com_trajectory[0, i]) / dt[0]
            velocities[-1, i] = (com_trajectory[-1, i] - com_trajectory[-2, i]) / dt[-1]
        
        # Acceleration via central differences on velocity
        accelerations = np.zeros_like(com_trajectory)
        for i in range(3):
            accelerations[1:-1, i] = (velocities[2:, i] - velocities[:-2, i]) / (times[2:] - times[:-2])
            accelerations[0, i] = accelerations[1, i]
            accelerations[-1, i] = accelerations[-2, i]
        
        # Smooth if requested (reduces noise in numerical derivatives)
        if smooth and N >= 7:
            window = min(7, N if N % 2 == 1 else N - 1)
            for i in range(3):
                velocities[:, i] = savgol_filter(velocities[:, i], window, 3)
                accelerations[:, i] = savgol_filter(accelerations[:, i], window, 3)
        
        return velocities, accelerations
    
    def analyze_trajectory(
        self,
        com_trajectory: np.ndarray,
        times: np.ndarray,
        support_polygon: np.ndarray,
        safety_margin: float = 0.0
    ) -> DynamicZMPTrajectory:
        """
        Analyze ZMP stability over entire trajectory.
        
        Args:
            com_trajectory: (N, 3) CoM positions
            times: (N,) timestamps
            support_polygon: (M, 2) vertices of support polygon
            safety_margin: Required margin inside polygon
            
        Returns:
            DynamicZMPTrajectory with full analysis
        """
        N = len(times)
        
        # Compute derivatives
        velocities, accelerations = self.compute_com_derivatives(com_trajectory, times)
        
        # Compute ZMP at each point
        zmp_positions = np.zeros((N, 2))
        stability_margins = np.zeros(N)
        is_stable = np.zeros(N, dtype=bool)
        
        for i in range(N):
            zmp = self.compute_zmp(com_trajectory[i], accelerations[i])
            zmp_positions[i] = zmp
            
            margin = self._distance_to_polygon_edge(zmp, support_polygon)
            stability_margins[i] = margin
            is_stable[i] = margin >= safety_margin
        
        # Find minimum margin
        min_idx = np.argmin(stability_margins)
        
        return DynamicZMPTrajectory(
            times=times,
            com_positions=com_trajectory,
            com_velocities=velocities,
            com_accelerations=accelerations,
            zmp_positions=zmp_positions,
            stability_margins=stability_margins,
            is_stable=is_stable,
            min_margin=stability_margins[min_idx],
            min_margin_time=times[min_idx],
            all_stable=np.all(is_stable)
        )
    
    def predict_zmp_for_trajectory(
        self,
        planned_com: np.ndarray,
        planned_times: np.ndarray
    ) -> np.ndarray:
        """
        Predict ZMP trajectory for a planned CoM motion.
        
        Useful for checking if a planned trajectory will be stable
        before executing it.
        
        Args:
            planned_com: (N, 3) planned CoM positions
            planned_times: (N,) time stamps
            
        Returns:
            (N, 2) predicted ZMP positions
        """
        velocities, accelerations = self.compute_com_derivatives(planned_com, planned_times)
        
        N = len(planned_times)
        zmp_trajectory = np.zeros((N, 2))
        
        for i in range(N):
            zmp_trajectory[i] = self.compute_zmp(planned_com[i], accelerations[i])
        
        return zmp_trajectory
    
    def _point_in_polygon(self, point: np.ndarray, polygon: np.ndarray) -> bool:
        """Check if point is inside polygon using cross product method."""
        n = len(polygon)
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]
            edge = p2 - p1
            to_point = point - p1
            cross = edge[0] * to_point[1] - edge[1] * to_point[0]
            if cross < 0:
                return False
        return True
    
    def _distance_to_polygon_edge(self, point: np.ndarray, polygon: np.ndarray) -> float:
        """
        Compute signed distance from point to nearest polygon edge.
        Positive = inside, Negative = outside
        """
        n = len(polygon)
        min_dist = float('inf')
        
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]
            
            # Distance to line segment
            edge = p2 - p1
            edge_len_sq = np.dot(edge, edge)
            
            if edge_len_sq < 1e-10:
                dist = np.linalg.norm(point - p1)
            else:
                t = max(0, min(1, np.dot(point - p1, edge) / edge_len_sq))
                projection = p1 + t * edge
                dist = np.linalg.norm(point - projection)
            
            min_dist = min(min_dist, dist)
        
        # Sign: positive if inside
        if self._point_in_polygon(point, polygon):
            return min_dist
        else:
            return -min_dist


def compare_static_vs_dynamic():
    """Compare quasi-static vs dynamic ZMP calculation."""
    import matplotlib.pyplot as plt
    
    print("=" * 60)
    print("Comparing Quasi-Static vs Dynamic ZMP")
    print("=" * 60)
    
    calc = DynamicZMPCalculator()
    
    # Create a fast reaching motion (CoM moves forward quickly)
    duration = 1.0  # Fast 1-second motion
    N = 100
    times = np.linspace(0, duration, N)
    
    # Simulate CoM moving forward 5cm with acceleration
    com_trajectory = np.zeros((N, 3))
    com_trajectory[:, 2] = 0.7  # Height = 0.7m (constant)
    
    # S-curve motion profile (smooth acceleration)
    for i, t in enumerate(times):
        # Quintic profile for x motion
        tau = t / duration
        s = 10*tau**3 - 15*tau**4 + 6*tau**5  # Normalized position
        com_trajectory[i, 0] = 0.02 + 0.05 * s  # 2cm to 7cm forward
        com_trajectory[i, 1] = 0.0
    
    # Compute velocities and accelerations
    velocities, accelerations = calc.compute_com_derivatives(com_trajectory, times, smooth=True)
    
    # Compute ZMP both ways
    zmp_static = np.zeros((N, 2))
    zmp_dynamic = np.zeros((N, 2))
    
    for i in range(N):
        zmp_static[i] = calc.compute_zmp(com_trajectory[i], None)  # Quasi-static
        zmp_dynamic[i] = calc.compute_zmp(com_trajectory[i], accelerations[i])  # Dynamic
    
    # Support polygon (typical humanoid feet)
    support_polygon = np.array([
        [-0.05, -0.12],
        [0.05, -0.12],
        [0.05, 0.12],
        [-0.05, 0.12]
    ])
    
    print(f"Motion duration: {duration}s")
    print(f"CoM displacement: {com_trajectory[-1, 0] - com_trajectory[0, 0]:.3f}m")
    print(f"Max CoM velocity: {np.abs(velocities[:, 0]).max():.3f} m/s")
    print(f"Max CoM acceleration: {np.abs(accelerations[:, 0]).max():.3f} m/s²")
    print(f"\nMax ZMP difference (static vs dynamic): {np.abs(zmp_static - zmp_dynamic).max()*1000:.1f} mm")
    
    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # CoM trajectory
    axes[0, 0].plot(times, com_trajectory[:, 0] * 100, 'b-', linewidth=2)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('CoM X (cm)')
    axes[0, 0].set_title('CoM Forward Motion')
    axes[0, 0].grid(True, alpha=0.3)
    
    # CoM acceleration
    axes[0, 1].plot(times, accelerations[:, 0], 'r-', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Acceleration (m/s²)')
    axes[0, 1].set_title('CoM X Acceleration')
    axes[0, 1].grid(True, alpha=0.3)
    
    # ZMP comparison
    axes[1, 0].plot(times, zmp_static[:, 0] * 100, 'b--', linewidth=2, label='Quasi-static')
    axes[1, 0].plot(times, zmp_dynamic[:, 0] * 100, 'r-', linewidth=2, label='Dynamic')
    axes[1, 0].axhline(y=5, color='k', linestyle=':', alpha=0.5, label='Support polygon edge')
    axes[1, 0].axhline(y=-5, color='k', linestyle=':', alpha=0.5)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('ZMP X (cm)')
    axes[1, 0].set_title('ZMP: Static vs Dynamic')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # ZMP on support polygon
    poly_closed = np.vstack([support_polygon, support_polygon[0]])
    axes[1, 1].plot(poly_closed[:, 0] * 100, poly_closed[:, 1] * 100, 'b-', linewidth=2, label='Support polygon')
    axes[1, 1].plot(zmp_static[:, 0] * 100, zmp_static[:, 1] * 100, 'g--', linewidth=2, label='Static ZMP')
    axes[1, 1].plot(zmp_dynamic[:, 0] * 100, zmp_dynamic[:, 1] * 100, 'r-', linewidth=2, label='Dynamic ZMP')
    axes[1, 1].scatter([zmp_dynamic[0, 0] * 100], [zmp_dynamic[0, 1] * 100], c='green', s=100, marker='o', zorder=5, label='Start')
    axes[1, 1].scatter([zmp_dynamic[-1, 0] * 100], [zmp_dynamic[-1, 1] * 100], c='red', s=100, marker='s', zorder=5, label='End')
    axes[1, 1].set_xlabel('X (cm)')
    axes[1, 1].set_ylabel('Y (cm)')
    axes[1, 1].set_title('ZMP Trajectory on Support Polygon')
    axes[1, 1].legend(loc='upper right')
    axes[1, 1].axis('equal')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('dynamic_zmp_comparison.png', dpi=150)
    print("\nPlot saved to dynamic_zmp_comparison.png")
    
    # Analyze with support polygon
    print("\n--- Full Trajectory Analysis ---")
    result = calc.analyze_trajectory(com_trajectory, times, support_polygon, safety_margin=0.01)
    print(f"All points stable: {result.all_stable}")
    print(f"Minimum stability margin: {result.min_margin*1000:.1f} mm at t={result.min_margin_time:.2f}s")


if __name__ == '__main__':
    compare_static_vs_dynamic()
