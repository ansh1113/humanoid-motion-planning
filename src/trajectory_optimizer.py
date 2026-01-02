"""
Trajectory Optimization using Drake

Generates smooth, time-optimal trajectories using:
- Direct Collocation for trajectory optimization
- Minimum-jerk cost functions
- Joint limit constraints
- Velocity and acceleration limits

This replaces simple linear interpolation with properly optimized trajectories.
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass

from pydrake.all import (
    MathematicalProgram,
    Solve,
    SolverOptions,
    SnoptSolver,
    IpoptSolver,
    eq,
    le,
    ge,
)


@dataclass
class OptimizedTrajectory:
    """Result of trajectory optimization"""
    success: bool
    times: np.ndarray           # (N,) time points
    positions: np.ndarray       # (N, n_joints) joint positions
    velocities: np.ndarray      # (N, n_joints) joint velocities
    accelerations: np.ndarray   # (N, n_joints) joint accelerations
    jerks: np.ndarray           # (N, n_joints) joint jerks
    cost: float
    message: str


class TrajectoryOptimizer:
    """
    Optimizes trajectories for smooth, efficient motion.
    
    Uses Drake's mathematical programming to solve:
    
    minimize: ∫ ||q̈||² dt  (minimum acceleration)
           or ∫ ||q⃛||² dt  (minimum jerk)
    
    subject to:
        q(0) = q_start
        q(T) = q_end
        q̇(0) = 0, q̇(T) = 0  (start/end at rest)
        q_min ≤ q ≤ q_max   (joint limits)
        |q̇| ≤ v_max         (velocity limits)
        |q̈| ≤ a_max         (acceleration limits)
    """
    
    def __init__(
        self,
        n_joints: int,
        joint_limits_lower: np.ndarray,
        joint_limits_upper: np.ndarray,
        velocity_limits: np.ndarray = None,
        acceleration_limits: np.ndarray = None
    ):
        """
        Args:
            n_joints: Number of joints
            joint_limits_lower: Lower joint limits (rad)
            joint_limits_upper: Upper joint limits (rad)
            velocity_limits: Max joint velocities (rad/s). Default: 2.0
            acceleration_limits: Max joint accelerations (rad/s²). Default: 10.0
        """
        self.n_joints = n_joints
        self.q_min = joint_limits_lower
        self.q_max = joint_limits_upper
        
        self.v_max = velocity_limits if velocity_limits is not None else np.full(n_joints, 2.0)
        self.a_max = acceleration_limits if acceleration_limits is not None else np.full(n_joints, 10.0)
    
    def optimize_min_jerk(
        self,
        q_start: np.ndarray,
        q_end: np.ndarray,
        duration: float,
        n_segments: int = 20
    ) -> OptimizedTrajectory:
        """
        Generate minimum-jerk trajectory.
        
        Minimum jerk produces the smoothest motion (used in human arm movements).
        
        Args:
            q_start: Starting joint configuration
            q_end: Ending joint configuration
            duration: Total motion duration (seconds)
            n_segments: Number of trajectory segments
            
        Returns:
            OptimizedTrajectory with optimized path
        """
        n = self.n_joints
        N = n_segments + 1  # Number of knot points
        dt = duration / n_segments
        
        prog = MathematicalProgram()
        
        # Decision variables at each knot point
        # q[i] = positions, v[i] = velocities, a[i] = accelerations
        q = prog.NewContinuousVariables(N, n, "q")
        v = prog.NewContinuousVariables(N, n, "v")
        a = prog.NewContinuousVariables(N, n, "a")
        
        # Jerk variables (between knot points)
        jerk = prog.NewContinuousVariables(n_segments, n, "jerk")
        
        # Boundary conditions
        for j in range(n):
            prog.AddConstraint(q[0, j] == q_start[j])
            prog.AddConstraint(q[-1, j] == q_end[j])
            prog.AddConstraint(v[0, j] == 0)  # Start at rest
            prog.AddConstraint(v[-1, j] == 0)  # End at rest
            prog.AddConstraint(a[0, j] == 0)  # Zero initial acceleration
            prog.AddConstraint(a[-1, j] == 0)  # Zero final acceleration
        
        # Dynamics constraints (trapezoidal collocation)
        for i in range(n_segments):
            for j in range(n):
                # Position: q[i+1] = q[i] + dt * (v[i] + v[i+1]) / 2
                prog.AddConstraint(
                    q[i+1, j] == q[i, j] + dt * (v[i, j] + v[i+1, j]) / 2
                )
                # Velocity: v[i+1] = v[i] + dt * (a[i] + a[i+1]) / 2
                prog.AddConstraint(
                    v[i+1, j] == v[i, j] + dt * (a[i, j] + a[i+1, j]) / 2
                )
                # Acceleration: a[i+1] = a[i] + dt * jerk[i]
                prog.AddConstraint(
                    a[i+1, j] == a[i, j] + dt * jerk[i, j]
                )
        
        # Joint limits
        for i in range(N):
            for j in range(n):
                prog.AddConstraint(q[i, j] >= self.q_min[j])
                prog.AddConstraint(q[i, j] <= self.q_max[j])
                prog.AddConstraint(v[i, j] >= -self.v_max[j])
                prog.AddConstraint(v[i, j] <= self.v_max[j])
                prog.AddConstraint(a[i, j] >= -self.a_max[j])
                prog.AddConstraint(a[i, j] <= self.a_max[j])
        
        # Cost: minimize squared jerk (minimum jerk trajectory)
        for i in range(n_segments):
            for j in range(n):
                prog.AddCost(jerk[i, j] ** 2)
        
        # Initial guess: linear interpolation
        for i in range(N):
            alpha = i / (N - 1)
            q_guess = (1 - alpha) * q_start + alpha * q_end
            prog.SetInitialGuess(q[i], q_guess)
            prog.SetInitialGuess(v[i], np.zeros(n))
            prog.SetInitialGuess(a[i], np.zeros(n))
        
        # Solve
        result = Solve(prog)
        
        if result.is_success():
            times = np.linspace(0, duration, N)
            positions = result.GetSolution(q)
            velocities = result.GetSolution(v)
            accelerations = result.GetSolution(a)
            jerks_sol = result.GetSolution(jerk)
            
            # Extend jerks to match time points
            jerks_full = np.vstack([jerks_sol, jerks_sol[-1:]])
            
            return OptimizedTrajectory(
                success=True,
                times=times,
                positions=positions,
                velocities=velocities,
                accelerations=accelerations,
                jerks=jerks_full,
                cost=result.get_optimal_cost(),
                message="Optimization successful"
            )
        else:
            return OptimizedTrajectory(
                success=False,
                times=np.array([]),
                positions=np.array([]),
                velocities=np.array([]),
                accelerations=np.array([]),
                jerks=np.array([]),
                cost=float('inf'),
                message=f"Optimization failed: {result.get_solution_result()}"
            )
    
    def optimize_min_time(
        self,
        q_start: np.ndarray,
        q_end: np.ndarray,
        initial_duration: float = 2.0,
        n_segments: int = 20
    ) -> Tuple[OptimizedTrajectory, float]:
        """
        Generate time-optimal trajectory (fastest motion within limits).
        
        Uses binary search on duration to find minimum feasible time.
        
        Args:
            q_start: Starting configuration
            q_end: Ending configuration
            initial_duration: Initial duration guess
            n_segments: Number of segments
            
        Returns:
            Tuple of (OptimizedTrajectory, optimal_duration)
        """
        # Binary search for minimum duration
        t_min = 0.1
        t_max = initial_duration * 2
        
        best_trajectory = None
        best_duration = t_max
        
        for _ in range(10):  # 10 iterations of binary search
            t_mid = (t_min + t_max) / 2
            
            result = self.optimize_min_jerk(q_start, q_end, t_mid, n_segments)
            
            if result.success:
                best_trajectory = result
                best_duration = t_mid
                t_max = t_mid
            else:
                t_min = t_mid
        
        if best_trajectory is None:
            # Fall back to longer duration
            best_trajectory = self.optimize_min_jerk(
                q_start, q_end, initial_duration * 2, n_segments
            )
            best_duration = initial_duration * 2
        
        return best_trajectory, best_duration


def generate_quintic_spline(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    n_points: int = 50
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate a quintic (5th order) polynomial trajectory.
    
    This is a closed-form minimum-jerk solution for point-to-point motion
    with zero velocity and acceleration at endpoints.
    
    The quintic polynomial is:
        q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    
    With boundary conditions:
        q(0) = q_start, q(T) = q_end
        q̇(0) = q̇(T) = 0
        q̈(0) = q̈(T) = 0
    
    Returns:
        times, positions, velocities, accelerations
    """
    T = duration
    times = np.linspace(0, T, n_points)
    
    n_joints = len(q_start)
    positions = np.zeros((n_points, n_joints))
    velocities = np.zeros((n_points, n_joints))
    accelerations = np.zeros((n_points, n_joints))
    
    for j in range(n_joints):
        q0 = q_start[j]
        qf = q_end[j]
        
        # Quintic coefficients for boundary conditions
        a0 = q0
        a1 = 0
        a2 = 0
        a3 = 10 * (qf - q0) / T**3
        a4 = -15 * (qf - q0) / T**4
        a5 = 6 * (qf - q0) / T**5
        
        for i, t in enumerate(times):
            positions[i, j] = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
            velocities[i, j] = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            accelerations[i, j] = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
    
    return times, positions, velocities, accelerations


# Test
if __name__ == '__main__':
    print("Testing Trajectory Optimizer...")
    
    # Simple 3-joint test
    n_joints = 3
    q_min = np.array([-2.0, -2.0, -2.0])
    q_max = np.array([2.0, 2.0, 2.0])
    
    optimizer = TrajectoryOptimizer(n_joints, q_min, q_max)
    
    q_start = np.array([0.0, 0.0, 0.0])
    q_end = np.array([1.0, 0.5, -0.5])
    
    print("\n--- Minimum Jerk Trajectory ---")
    result = optimizer.optimize_min_jerk(q_start, q_end, duration=2.0, n_segments=15)
    
    print(f"Success: {result.success}")
    print(f"Cost: {result.cost:.6f}")
    
    if result.success:
        print(f"Time points: {len(result.times)}")
        print(f"Start pos: {result.positions[0]}")
        print(f"End pos:   {result.positions[-1]}")
        print(f"Max velocity: {np.abs(result.velocities).max():.4f} rad/s")
        print(f"Max acceleration: {np.abs(result.accelerations).max():.4f} rad/s²")
    
    print("\n--- Quintic Spline (Closed-form) ---")
    t, pos, vel, acc = generate_quintic_spline(q_start, q_end, 2.0, 50)
    print(f"Time points: {len(t)}")
    print(f"Max velocity: {np.abs(vel).max():.4f} rad/s")
    print(f"Max acceleration: {np.abs(acc).max():.4f} rad/s²")
    
    # Plot comparison
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    
    # Position
    axes[0].plot(result.times, result.positions[:, 0], 'b-', label='Drake optimized', linewidth=2)
    axes[0].plot(t, pos[:, 0], 'r--', label='Quintic spline', linewidth=2)
    axes[0].set_ylabel('Position (rad)')
    axes[0].legend()
    axes[0].set_title('Trajectory Comparison (Joint 1)')
    axes[0].grid(True, alpha=0.3)
    
    # Velocity
    axes[1].plot(result.times, result.velocities[:, 0], 'b-', linewidth=2)
    axes[1].plot(t, vel[:, 0], 'r--', linewidth=2)
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].grid(True, alpha=0.3)
    
    # Acceleration
    axes[2].plot(result.times, result.accelerations[:, 0], 'b-', linewidth=2)
    axes[2].plot(t, acc[:, 0], 'r--', linewidth=2)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Acceleration (rad/s²)')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_optimization_test.png', dpi=150)
    print("\nPlot saved to trajectory_optimization_test.png")
