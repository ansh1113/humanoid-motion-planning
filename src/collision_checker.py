"""
Collision Checker using MuJoCo

Provides collision detection between robot and environment:
- Self-collision detection  
- Environment obstacle collision
- Safety distance computation
- Collision-free trajectory verification
"""

import numpy as np
import mujoco
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class CollisionInfo:
    """Information about a detected collision"""
    body1: str
    body2: str
    geom1: str
    geom2: str
    distance: float        # Negative = penetration
    contact_point: np.ndarray
    contact_normal: np.ndarray


@dataclass 
class CollisionCheckResult:
    """Result of collision check"""
    has_collision: bool
    min_distance: float
    collisions: List[CollisionInfo]
    safe: bool  # True if min_distance > safety_margin
    n_contacts: int


class CollisionChecker:
    """
    Collision detection using MuJoCo's built-in collision engine.
    """
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        
        # Build body and geom name lookups
        self.body_names = {}
        self.geom_names = {}
        
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name:
                self.body_names[i] = name
        
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name:
                self.geom_names[i] = name
        
        # Pairs to ignore (adjacent links, foot-ground)
        self.ignored_pairs = set()
        self._setup_ignored_pairs()
    
    def _setup_ignored_pairs(self):
        """Setup pairs of bodies that should not be checked for collision."""
        # Ignore parent-child (they naturally touch at joints)
        for i in range(self.model.nbody):
            parent = self.model.body_parentid[i]
            if parent >= 0:
                self.ignored_pairs.add((min(parent, i), max(parent, i)))
        
        # Ignore ground contact with feet (that's expected)
        ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "world")
        for name in ["left_ankle_roll", "right_ankle_roll"]:
            foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if foot_id >= 0 and ground_id >= 0:
                self.ignored_pairs.add((min(ground_id, foot_id), max(ground_id, foot_id)))
    
    def check_collisions(self, safety_margin: float = 0.01) -> CollisionCheckResult:
        """Check for collisions in current robot configuration."""
        mujoco.mj_forward(self.model, self.data)
        
        collisions = []
        min_distance = float('inf')
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            
            geom1 = contact.geom1
            geom2 = contact.geom2
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
            # Skip ignored pairs
            pair = (min(body1, body2), max(body1, body2))
            if pair in self.ignored_pairs:
                continue
            
            body1_name = self.body_names.get(body1, f"body_{body1}")
            body2_name = self.body_names.get(body2, f"body_{body2}")
            geom1_name = self.geom_names.get(geom1, f"geom_{geom1}")
            geom2_name = self.geom_names.get(geom2, f"geom_{geom2}")
            
            dist = contact.dist
            min_distance = min(min_distance, dist)
            
            if dist < safety_margin:
                collisions.append(CollisionInfo(
                    body1=body1_name,
                    body2=body2_name,
                    geom1=geom1_name,
                    geom2=geom2_name,
                    distance=dist,
                    contact_point=contact.pos.copy(),
                    contact_normal=contact.frame[:3].copy()
                ))
        
        # If no contacts at all, report as safe with large distance
        if min_distance == float('inf'):
            min_distance = 1.0  # Default safe distance
        
        return CollisionCheckResult(
            has_collision=len(collisions) > 0,
            min_distance=min_distance,
            collisions=collisions,
            safe=min_distance >= safety_margin,
            n_contacts=self.data.ncon
        )
    
    def check_configuration(
        self,
        qpos_indices: List[int],
        joint_values: np.ndarray,
        safety_margin: float = 0.01
    ) -> CollisionCheckResult:
        """
        Check a specific joint configuration for collisions.
        
        Args:
            qpos_indices: Indices into qpos array
            joint_values: Joint values to set
            safety_margin: Required clearance
            
        Returns:
            CollisionCheckResult
        """
        # Save current state
        original_qpos = self.data.qpos.copy()
        
        # Set new configuration
        for i, idx in enumerate(qpos_indices):
            self.data.qpos[idx] = joint_values[i]
        
        # Check collisions
        result = self.check_collisions(safety_margin)
        
        # Restore state
        self.data.qpos[:] = original_qpos
        mujoco.mj_forward(self.model, self.data)
        
        return result
    
    def check_trajectory(
        self,
        qpos_indices: List[int],
        trajectory: np.ndarray,
        safety_margin: float = 0.01
    ) -> Tuple[bool, int, List[CollisionCheckResult]]:
        """
        Check entire trajectory for collisions.
        
        Args:
            qpos_indices: Indices into qpos array
            trajectory: (N, n_joints) joint positions over time
            safety_margin: Required clearance
            
        Returns:
            (all_safe, first_collision_index, all_results)
        """
        original_qpos = self.data.qpos.copy()
        
        results = []
        first_collision_idx = -1
        
        for i, q in enumerate(trajectory):
            # Set configuration
            for j, idx in enumerate(qpos_indices):
                self.data.qpos[idx] = q[j]
            
            result = self.check_collisions(safety_margin)
            results.append(result)
            
            if result.has_collision and first_collision_idx < 0:
                first_collision_idx = i
        
        # Restore state
        self.data.qpos[:] = original_qpos
        mujoco.mj_forward(self.model, self.data)
        
        all_safe = first_collision_idx < 0
        return all_safe, first_collision_idx, results


def test_collision_checker():
    """Test collision detection with actual self-collisions."""
    import sys
    sys.path.insert(0, 'src')
    from g1_model import G1Model
    
    print("=" * 60)
    print("Testing Collision Checker with Self-Collision Scenarios")
    print("=" * 60)
    
    robot = G1Model()
    checker = CollisionChecker(robot.model, robot.data)
    
    # Get arm joint indices
    arm_info = robot.joint_groups['right_arm']
    qpos_indices = [int(idx) for idx in arm_info['qpos_indices']]
    n_joints = len(qpos_indices)
    
    print(f"\nRight arm has {n_joints} joints")
    print(f"Joint indices: {qpos_indices}")
    
    # Test 1: Initial safe configuration
    print("\n--- Test 1: Initial Configuration ---")
    result = checker.check_collisions(safety_margin=0.01)
    print(f"Total contacts in scene: {result.n_contacts}")
    print(f"Has collision (non-ignored): {result.has_collision}")
    print(f"Min distance: {result.min_distance*1000:.1f} mm")
    
    # Test 2: Move arm to safe forward position
    print("\n--- Test 2: Arm Forward (Safe) ---")
    safe_config = np.zeros(n_joints)
    safe_config[0] = 0.5   # Shoulder pitch forward
    safe_config[3] = -0.5  # Elbow bent
    
    result = checker.check_configuration(qpos_indices, safe_config, safety_margin=0.01)
    print(f"Configuration safe: {result.safe}")
    print(f"Min distance: {result.min_distance*1000:.1f} mm")
    
    # Test 3: Move arm across body (potential self-collision)
    print("\n--- Test 3: Arm Across Body (Potential Collision) ---")
    cross_config = np.zeros(n_joints)
    cross_config[0] = 0.8   # Shoulder pitch forward
    cross_config[1] = 1.5   # Shoulder roll inward (across body)
    cross_config[3] = -1.0  # Elbow bent
    
    result = checker.check_configuration(qpos_indices, cross_config, safety_margin=0.01)
    print(f"Configuration safe: {result.safe}")
    print(f"Min distance: {result.min_distance*1000:.1f} mm")
    print(f"Has collision: {result.has_collision}")
    if result.collisions:
        for c in result.collisions[:3]:
            print(f"  Collision: {c.body1} <-> {c.body2}: {c.distance*1000:.1f} mm")
    
    # Test 4: Extreme arm position (definite collision)
    print("\n--- Test 4: Arm Behind Back (Likely Collision) ---")
    extreme_config = np.zeros(n_joints)
    extreme_config[0] = -1.5  # Shoulder pitch way back
    extreme_config[1] = 1.0   # Shoulder roll inward
    
    result = checker.check_configuration(qpos_indices, extreme_config, safety_margin=0.01)
    print(f"Configuration safe: {result.safe}")
    print(f"Min distance: {result.min_distance*1000:.1f} mm")
    print(f"Has collision: {result.has_collision}")
    if result.collisions:
        for c in result.collisions[:3]:
            print(f"  Collision: {c.body1} <-> {c.body2}: {c.distance*1000:.1f} mm")
    
    # Test 5: Trajectory check
    print("\n--- Test 5: Trajectory from Safe to Collision ---")
    n_points = 20
    trajectory = np.zeros((n_points, n_joints))
    
    # Interpolate from safe to cross-body
    for i in range(n_points):
        alpha = i / (n_points - 1)
        trajectory[i] = (1 - alpha) * safe_config + alpha * cross_config
    
    all_safe, first_col, results = checker.check_trajectory(
        qpos_indices, trajectory, safety_margin=0.01
    )
    
    print(f"Trajectory fully safe: {all_safe}")
    if not all_safe:
        print(f"First collision at waypoint: {first_col}")
    
    # Find min distance along trajectory
    min_dist = min(r.min_distance for r in results)
    min_idx = np.argmin([r.min_distance for r in results])
    print(f"Minimum distance: {min_dist*1000:.1f} mm at waypoint {min_idx}")
    
    # Visualize trajectory safety
    print("\nTrajectory safety profile:")
    for i in range(0, n_points, 4):
        status = "✓" if results[i].safe else "✗"
        dist = results[i].min_distance * 1000
        print(f"  Waypoint {i:2d}: {status} dist={dist:6.1f} mm")
    
    print("\n✓ Collision checker test complete!")
    
    return robot, checker


if __name__ == '__main__':
    test_collision_checker()
