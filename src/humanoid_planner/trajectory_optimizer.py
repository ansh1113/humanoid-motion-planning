"""
Trajectory Optimization Module

Optimizes robot trajectories for smooth, energy-efficient motion while
satisfying kinematic and dynamic constraints.
"""

import numpy as np
from scipy.optimize import minimize
from typing import Optional, Callable, Tuple


class TrajectoryOptimizer:
    """Optimizes trajectories for humanoid reaching tasks."""
    
    def __init__(self,
                 num_joints: int,
                 dt: float = 0.1,
                 weight_smoothness: float = 1.0,
                 weight_time: float = 0.1,
                 weight_energy: float = 0.01):
        """
        Initialize trajectory optimizer.
        
        Args:
            num_joints: Number of robot joints
            dt: Time step duration (seconds)
            weight_smoothness: Weight for smoothness cost
            weight_time: Weight for trajectory duration
            weight_energy: Weight for energy consumption
        """
        self.num_joints = num_joints
        self.dt = dt
        self.w_smooth = weight_smoothness
        self.w_time = weight_time
        self.w_energy = weight_energy
    
    def optimize_trajectory(self,
                           q_start: np.ndarray,
                           q_goal: np.ndarray,
                           num_waypoints: int = 20,
                           q_limits: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                           velocity_limit: float = 1.0) -> np.ndarray:
        """
        Optimize a smooth trajectory from start to goal configuration.
        
        Args:
            q_start: Starting joint configuration (num_joints,)
            q_goal: Goal joint configuration (num_joints,)
            num_waypoints: Number of waypoints in trajectory
            q_limits: Joint limits as (q_min, q_max)
            velocity_limit: Maximum joint velocity (rad/s)
            
        Returns:
            trajectory: Optimized trajectory (num_waypoints x num_joints)
        """
        # Initialize with linear interpolation
        t = np.linspace(0, 1, num_waypoints)
        q_init = np.outer(t, q_goal) + np.outer(1 - t, q_start)
        
        # Flatten for optimization
        x0 = q_init.flatten()
        
        # Define cost function
        def cost(x):
            q = x.reshape(num_waypoints, self.num_joints)
            return self._compute_cost(q)
        
        # Define constraints
        constraints = []
        
        # Start and end constraints
        def start_constraint(x):
            q = x.reshape(num_waypoints, self.num_joints)
            return q[0] - q_start
        
        def goal_constraint(x):
            q = x.reshape(num_waypoints, self.num_joints)
            return q[-1] - q_goal
        
        constraints.append({'type': 'eq', 'fun': start_constraint})
        constraints.append({'type': 'eq', 'fun': goal_constraint})
        
        # Velocity constraints
        if velocity_limit is not None:
            def velocity_constraint(x):
                q = x.reshape(num_waypoints, self.num_joints)
                velocities = np.diff(q, axis=0) / self.dt
                max_vel = np.max(np.abs(velocities))
                return velocity_limit - max_vel
            
            constraints.append({'type': 'ineq', 'fun': velocity_constraint})
        
        # Joint limit bounds
        if q_limits is not None:
            q_min, q_max = q_limits
            bounds = []
            for _ in range(num_waypoints):
                for j in range(self.num_joints):
                    bounds.append((q_min[j], q_max[j]))
        else:
            bounds = None
        
        # Optimize
        result = minimize(
            cost,
            x0,
            method='SLSQP',
            constraints=constraints,
            bounds=bounds,
            options={'maxiter': 500, 'ftol': 1e-6}
        )
        
        if not result.success:
            print(f"Warning: Optimization did not converge: {result.message}")
        
        # Reshape result
        trajectory = result.x.reshape(num_waypoints, self.num_joints)
        
        return trajectory
    
    def _compute_cost(self, q: np.ndarray) -> float:
        """
        Compute total trajectory cost.
        
        Args:
            q: Trajectory (num_waypoints x num_joints)
            
        Returns:
            cost: Total cost value
        """
        cost = 0.0
        
        # Smoothness cost (minimize acceleration)
        if len(q) >= 3:
            accel = np.diff(q, n=2, axis=0) / (self.dt ** 2)
            cost += self.w_smooth * np.sum(accel ** 2)
        
        # Time cost (encourage faster trajectories)
        cost += self.w_time * len(q) * self.dt
        
        # Energy cost (minimize large joint angles)
        cost += self.w_energy * np.sum(np.abs(q))
        
        return cost
    
    def compute_velocity_profile(self, trajectory: np.ndarray) -> np.ndarray:
        """
        Compute velocity profile from trajectory.
        
        Args:
            trajectory: Joint trajectory (T x num_joints)
            
        Returns:
            velocities: Joint velocities (T-1 x num_joints)
        """
        return np.diff(trajectory, axis=0) / self.dt
    
    def compute_acceleration_profile(self, trajectory: np.ndarray) -> np.ndarray:
        """
        Compute acceleration profile from trajectory.
        
        Args:
            trajectory: Joint trajectory (T x num_joints)
            
        Returns:
            accelerations: Joint accelerations (T-2 x num_joints)
        """
        return np.diff(trajectory, n=2, axis=0) / (self.dt ** 2)
    
    def smooth_trajectory(self,
                         trajectory: np.ndarray,
                         window_size: int = 5) -> np.ndarray:
        """
        Smooth trajectory using moving average filter.
        
        Args:
            trajectory: Input trajectory (T x num_joints)
            window_size: Size of smoothing window
            
        Returns:
            smoothed: Smoothed trajectory (T x num_joints)
        """
        from scipy.ndimage import uniform_filter1d
        
        smoothed = uniform_filter1d(trajectory, size=window_size, axis=0, mode='nearest')
        
        # Preserve start and end points
        smoothed[0] = trajectory[0]
        smoothed[-1] = trajectory[-1]
        
        return smoothed
    
    def interpolate_trajectory(self,
                               waypoints: np.ndarray,
                               num_points: int) -> np.ndarray:
        """
        Interpolate between waypoints to create dense trajectory.
        
        Args:
            waypoints: Sparse waypoints (N x num_joints)
            num_points: Desired number of points in trajectory
            
        Returns:
            trajectory: Interpolated trajectory (num_points x num_joints)
        """
        from scipy.interpolate import interp1d
        
        t_waypoints = np.linspace(0, 1, len(waypoints))
        t_dense = np.linspace(0, 1, num_points)
        
        interpolator = interp1d(t_waypoints, waypoints, axis=0, kind='cubic')
        trajectory = interpolator(t_dense)
        
        return trajectory
    
    def check_trajectory_feasibility(self,
                                     trajectory: np.ndarray,
                                     q_limits: Tuple[np.ndarray, np.ndarray],
                                     velocity_limit: float,
                                     acceleration_limit: float) -> bool:
        """
        Check if trajectory satisfies all constraints.
        
        Args:
            trajectory: Joint trajectory (T x num_joints)
            q_limits: Joint limits as (q_min, q_max)
            velocity_limit: Maximum joint velocity (rad/s)
            acceleration_limit: Maximum joint acceleration (rad/s^2)
            
        Returns:
            feasible: True if trajectory is feasible
        """
        q_min, q_max = q_limits
        
        # Check joint limits
        if np.any(trajectory < q_min) or np.any(trajectory > q_max):
            print("Joint limits violated")
            return False
        
        # Check velocity limits
        velocities = self.compute_velocity_profile(trajectory)
        if np.any(np.abs(velocities) > velocity_limit):
            print(f"Velocity limit violated: max = {np.max(np.abs(velocities)):.3f}")
            return False
        
        # Check acceleration limits
        if len(trajectory) >= 3:
            accelerations = self.compute_acceleration_profile(trajectory)
            if np.any(np.abs(accelerations) > acceleration_limit):
                print(f"Acceleration limit violated: max = {np.max(np.abs(accelerations)):.3f}")
                return False
        
        return True
