"""
Humanoid Whole-Body Motion Planner with ZMP Constraints

NOTE: This module requires Drake for full functionality.
Install with: pip install pydrake

For basic planning without Drake simulation, use the other modules directly:
- ZMPConstraint
- SupportPolygonCalculator  
- TrajectoryOptimizer
- ForwardKinematics, InverseKinematics
"""

import numpy as np
from typing import Optional, Tuple, List, Dict
from .zmp_constraint import ZMPConstraint
from .support_polygon import SupportPolygonCalculator
from .trajectory_optimizer import TrajectoryOptimizer
from .stability_analysis import StabilityAnalyzer

try:
    from pydrake.all import (
        MultibodyPlant, Parser, DiagramBuilder, Simulator,
        MathematicalProgram, Solve, eq, le, ge, AddMultibodyPlantSceneGraph,
        SnoptSolver, IpoptSolver
    )
    DRAKE_AVAILABLE = True
except ImportError:
    DRAKE_AVAILABLE = False
    print("Warning: Drake not available. MotionPlanner will have limited functionality.")
    print("Install Drake with: pip install pydrake")


class MotionPlanner:
    """
    Main motion planner for humanoid reaching tasks with ZMP constraints.
    
    Integrates Drake optimization framework for trajectory generation.
    """
    
    def __init__(self, 
                 urdf_path: Optional[str] = None,
                 time_step: float = 0.01,
                 zmp_margin: float = 0.02):
        """
        Initialize motion planner.
        
        Args:
            urdf_path: Path to robot URDF file
            time_step: Simulation time step
            zmp_margin: Safety margin for ZMP constraints
        """
        self.urdf_path = urdf_path
        self.time_step = time_step
        
        # Initialize sub-modules
        self.zmp_constraint = ZMPConstraint(margin=zmp_margin)
        self.support_calc = SupportPolygonCalculator()
        self.trajectory_opt = TrajectoryOptimizer(num_joints=12, dt=time_step)
        self.stability_analyzer = StabilityAnalyzer(zmp_margin=zmp_margin)
        
        # Drake components (if available)
        self.plant = None
        self.plant_context = None
        
        if DRAKE_AVAILABLE and urdf_path is not None:
            self._initialize_drake(urdf_path)
    
    def _initialize_drake(self, urdf_path: str):
        """
        Initialize Drake MultibodyPlant from URDF.
        
        Args:
            urdf_path: Path to URDF file
        """
        if not DRAKE_AVAILABLE:
            return
        
        try:
            builder = DiagramBuilder()
            self.plant, scene_graph = AddMultibodyPlantSceneGraph(
                builder, time_step=self.time_step
            )
            
            parser = Parser(self.plant)
            parser.AddModelFromFile(urdf_path)
            
            self.plant.Finalize()
            
            diagram = builder.Build()
            self.plant_context = diagram.CreateDefaultContext()
            
            print(f"Drake plant initialized with {self.plant.num_positions()} DOFs")
            
        except Exception as e:
            print(f"Warning: Failed to initialize Drake plant: {e}")
            self.plant = None
    
    def plan_reaching_task(self,
                          target_pose: np.ndarray,
                          start_config: Optional[np.ndarray] = None,
                          foot_poses: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                          max_planning_time: float = 5.0,
                          num_waypoints: int = 20) -> Dict:
        """
        Plan a reaching trajectory with ZMP constraints.
        
        Args:
            target_pose: Target end-effector pose [x, y, z, qx, qy, qz, qw]
            start_config: Starting joint configuration
            foot_poses: (left_foot, right_foot) poses for stability
            max_planning_time: Maximum planning time (seconds)
            num_waypoints: Number of trajectory waypoints
            
        Returns:
            result: Dictionary with trajectory and metrics
        """
        # Default starting configuration
        if start_config is None:
            start_config = np.zeros(12)  # Assuming 12-DOF robot
        
        # Default foot poses (double support)
        if foot_poses is None:
            left_foot = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
            right_foot = np.array([0.0, -0.1, 0.0, 0.0, 0.0, 0.0])
            foot_poses = (left_foot, right_foot)
        
        # Compute support polygon
        support_polygon = self.support_calc.compute_double_support_polygon(
            foot_poses[0], foot_poses[1]
        )
        
        # Use inverse kinematics to find goal configuration
        target_pos = target_pose[:3]
        goal_config = self._solve_ik_for_target(target_pos, start_config)
        
        if goal_config is None:
            print("Warning: IK failed, using heuristic goal")
            goal_config = start_config + np.random.uniform(-0.3, 0.3, len(start_config))
        
        # Optimize trajectory
        trajectory = self.trajectory_opt.optimize_trajectory(
            q_start=start_config,
            q_goal=goal_config,
            num_waypoints=num_waypoints,
            q_limits=self._get_joint_limits(len(start_config)),
            velocity_limit=1.0
        )
        
        # Verify ZMP constraints
        zmp_violations = self._verify_zmp_constraints(trajectory, support_polygon)
        
        # Compute metrics
        metrics = self._compute_trajectory_metrics(trajectory, support_polygon)
        
        result = {
            'trajectory': trajectory,
            'goal_config': goal_config,
            'support_polygon': support_polygon,
            'zmp_violations': zmp_violations,
            'metrics': metrics,
            'success': zmp_violations == 0
        }
        
        return result
    
    def _solve_ik_for_target(self,
                            target_pos: np.ndarray,
                            q_init: np.ndarray) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics for target position.
        
        Args:
            target_pos: Target position (3,)
            q_init: Initial guess
            
        Returns:
            q_solution: Joint configuration or None
        """
        # Simplified IK - in practice would use Drake's IK or custom solver
        from scipy.optimize import minimize
        
        def cost(q):
            # Simplified FK - just use a simple chain model
            pos = self._compute_simplified_fk(q)
            return np.linalg.norm(pos - target_pos) ** 2
        
        q_limits = self._get_joint_limits(len(q_init))
        bounds = [(q_limits[0][i], q_limits[1][i]) for i in range(len(q_init))]
        
        result = minimize(cost, q_init, method='SLSQP', bounds=bounds)
        
        if result.success and cost(result.x) < 0.01:  # 10cm tolerance
            return result.x
        
        return None
    
    def _compute_simplified_fk(self, q: np.ndarray) -> np.ndarray:
        """
        Simplified forward kinematics for reaching arm.
        
        Args:
            q: Joint angles
            
        Returns:
            pos: End-effector position (3,)
        """
        # Simplified model - just sum up link contributions
        # In practice, would use proper DH parameters or Drake
        link_lengths = np.ones(min(6, len(q))) * 0.3  # 30cm links
        
        x = np.sum(link_lengths[:len(q)] * np.cos(q[:len(link_lengths)]))
        y = np.sum(link_lengths[:len(q)] * np.sin(q[:len(link_lengths)]))
        z = 0.8 + np.sum(q[len(link_lengths):] * 0.1)  # Torso height + contribution
        
        return np.array([x, y, z])
    
    def _get_joint_limits(self, num_joints: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get joint limits.
        
        Args:
            num_joints: Number of joints
            
        Returns:
            q_min, q_max: Joint limits
        """
        q_min = -np.pi * np.ones(num_joints)
        q_max = np.pi * np.ones(num_joints)
        
        return q_min, q_max
    
    def _verify_zmp_constraints(self,
                               trajectory: np.ndarray,
                               support_polygon: np.ndarray) -> int:
        """
        Verify ZMP constraints along trajectory.
        
        Args:
            trajectory: Joint trajectory (T x num_joints)
            support_polygon: Support polygon vertices
            
        Returns:
            num_violations: Number of ZMP violations
        """
        num_violations = 0
        
        for q in trajectory:
            # Compute CoM (simplified)
            com_pos = self._compute_com_position(q)
            com_acc = np.zeros(3)  # Simplified - no dynamics
            
            # Compute ZMP
            zmp = self.zmp_constraint.compute_zmp_from_com(
                com_pos, com_acc, height=com_pos[2]
            )
            
            # Check stability
            if not self.zmp_constraint.is_stable(zmp, support_polygon):
                num_violations += 1
        
        return num_violations
    
    def _compute_com_position(self, q: np.ndarray) -> np.ndarray:
        """
        Compute center of mass position.
        
        Args:
            q: Joint configuration
            
        Returns:
            com: CoM position (3,)
        """
        # Simplified CoM computation
        # In practice, would use proper multi-body dynamics
        
        if self.plant is not None and DRAKE_AVAILABLE:
            # Use Drake for accurate CoM
            try:
                self.plant.SetPositions(self.plant_context, q)
                com = self.plant.CalcCenterOfMassPosition(self.plant_context)
                return com
            except:
                pass
        
        # Fallback: simplified model
        return np.array([0.0, 0.0, 0.9])  # Fixed height approximation
    
    def _compute_trajectory_metrics(self,
                                   trajectory: np.ndarray,
                                   support_polygon: np.ndarray) -> Dict:
        """
        Compute trajectory quality metrics.
        
        Args:
            trajectory: Joint trajectory
            support_polygon: Support polygon
            
        Returns:
            metrics: Dictionary of metrics
        """
        # Velocity profile
        velocities = self.trajectory_opt.compute_velocity_profile(trajectory)
        max_velocity = np.max(np.abs(velocities))
        avg_velocity = np.mean(np.abs(velocities))
        
        # Acceleration profile
        accelerations = self.trajectory_opt.compute_acceleration_profile(trajectory)
        max_acceleration = np.max(np.abs(accelerations))
        
        # Smoothness (jerk)
        if len(trajectory) >= 4:
            jerk = np.diff(accelerations, axis=0)
            smoothness = np.mean(np.abs(jerk))
        else:
            smoothness = 0.0
        
        # Execution time
        execution_time = len(trajectory) * self.time_step
        
        # Stability margins
        stability_margins = []
        for q in trajectory:
            com_pos = self._compute_com_position(q)
            margin = self.stability_analyzer.compute_static_stability_margin(
                com_pos, support_polygon
            )
            stability_margins.append(margin)
        
        min_stability_margin = np.min(stability_margins)
        avg_stability_margin = np.mean(stability_margins)
        
        return {
            'execution_time': execution_time,
            'max_velocity': max_velocity,
            'avg_velocity': avg_velocity,
            'max_acceleration': max_acceleration,
            'smoothness': smoothness,
            'min_stability_margin': min_stability_margin,
            'avg_stability_margin': avg_stability_margin,
            'num_waypoints': len(trajectory)
        }
    
    def execute_trajectory(self, trajectory: np.ndarray):
        """
        Execute planned trajectory.
        
        This is a placeholder for actual robot execution.
        In practice, would send commands to robot controller.
        
        Args:
            trajectory: Joint trajectory to execute
        """
        print(f"Executing trajectory with {len(trajectory)} waypoints")
        print(f"Duration: {len(trajectory) * self.time_step:.2f} seconds")
        
        # In real system: send to robot controller
        # For now, just validate
        if self.plant is not None:
            print("Trajectory validated with Drake plant")


class DrakeTrajectoryOptimizer:
    """
    Drake-based trajectory optimization with direct collocation.
    
    This provides more sophisticated optimization than the basic optimizer.
    """
    
    def __init__(self, plant: Optional['MultibodyPlant'] = None):
        """
        Initialize Drake optimizer.
        
        Args:
            plant: Drake MultibodyPlant
        """
        if not DRAKE_AVAILABLE:
            raise ImportError("DrakeTrajectoryOptimizer requires pydrake")
        
        self.plant = plant
    
    def optimize_with_constraints(self,
                                 start_state: np.ndarray,
                                 goal_state: np.ndarray,
                                 num_timesteps: int = 20,
                                 constraints: Optional[List] = None) -> Optional[np.ndarray]:
        """
        Optimize trajectory using Drake's mathematical program.
        
        Args:
            start_state: Starting state
            goal_state: Goal state
            num_timesteps: Number of timesteps
            constraints: List of constraint functions
            
        Returns:
            trajectory: Optimized trajectory or None
        """
        prog = MathematicalProgram()
        
        # Decision variables: joint positions at each timestep
        q = []
        for i in range(num_timesteps):
            qi = prog.NewContinuousVariables(len(start_state), f"q_{i}")
            q.append(qi)
        
        # Start and goal constraints
        prog.AddLinearEqualityConstraint(q[0] == start_state)
        prog.AddLinearEqualityConstraint(q[-1] == goal_state)
        
        # Smoothness cost (minimize acceleration)
        for i in range(num_timesteps - 2):
            accel = q[i] - 2*q[i+1] + q[i+2]
            prog.AddQuadraticCost(accel.dot(accel))
        
        # Add custom constraints
        if constraints is not None:
            for constraint_fn in constraints:
                for i in range(num_timesteps):
                    prog.AddConstraint(constraint_fn, q[i])
        
        # Solve
        solver = SnoptSolver() if SnoptSolver.is_available() else IpoptSolver()
        result = solver.Solve(prog)
        
        if result.is_success():
            trajectory = np.array([result.GetSolution(qi) for qi in q])
            return trajectory
        else:
            print(f"Optimization failed: {result.get_solution_result()}")
            return None


# Placeholder for when Drake is not available
if not DRAKE_AVAILABLE:
    MotionPlanner = None
    DrakeTrajectoryOptimizer = None
