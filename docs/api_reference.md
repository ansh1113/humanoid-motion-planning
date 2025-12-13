# API Reference

Complete API documentation for the Humanoid Motion Planning system.

## Table of Contents

- [Core Modules](#core-modules)
- [MotionPlanner](#motionplanner)
- [ZMPConstraint](#zmpconstraint)
- [SupportPolygonCalculator](#supportpolygoncalculator)
- [TrajectoryOptimizer](#trajectoryoptimizer)
- [Kinematics](#kinematics)
- [StabilityAnalysis](#stabilityanalysis)
- [Perception](#perception)
- [Configuration](#configuration)
- [Data Structures](#data-structures)
- [Utilities](#utilities)

---

## Core Modules

### Overview

```python
from humanoid_planner import (
    MotionPlanner,
    ZMPConstraint,
    SupportPolygonCalculator,
    TrajectoryOptimizer,
    Kinematics,
    StabilityAnalysis,
    Perception
)
```

---

## MotionPlanner

The main motion planning interface.

### Class: `MotionPlanner`

```python
class MotionPlanner:
    """
    Main motion planner for humanoid robots with ZMP constraints.
    
    Attributes:
        config (dict): Configuration parameters
        kinematics (Kinematics): Kinematics solver
        stability (StabilityAnalysis): Stability analyzer
        optimizer (TrajectoryOptimizer): Trajectory optimizer
    """
```

### Constructor

```python
def __init__(
    self,
    config_path: str = "config/planner_params.yaml",
    zmp_margin: float = None,
    max_velocity: float = None,
    max_acceleration: float = None,
    max_planning_time: float = None
)
```

**Parameters:**
- `config_path` (str): Path to YAML configuration file
- `zmp_margin` (float, optional): Safety margin from support polygon edge (meters)
- `max_velocity` (float, optional): Maximum joint velocity (rad/s)
- `max_acceleration` (float, optional): Maximum joint acceleration (rad/s²)
- `max_planning_time` (float, optional): Maximum planning time (seconds)

**Example:**
```python
# Using default config
planner = MotionPlanner()

# With custom config file
planner = MotionPlanner(config_path="my_config.yaml")

# Overriding specific parameters
planner = MotionPlanner(
    config_path="config/planner_params.yaml",
    zmp_margin=0.03,
    max_planning_time=10.0
)
```

### Methods

#### `plan_reaching_task`

```python
def plan_reaching_task(
    self,
    target_position: np.ndarray,
    initial_config: np.ndarray = None,
    zmp_margin: float = None,
    max_planning_time: float = None
) -> Optional[Dict]
```

Plan a trajectory to reach a target position while maintaining balance.

**Parameters:**
- `target_position` (np.ndarray): Target position [x, y, z] in meters
- `initial_config` (np.ndarray, optional): Initial robot configuration
- `zmp_margin` (float, optional): Override default ZMP margin
- `max_planning_time` (float, optional): Override default planning time

**Returns:**
- `dict` or `None`: Trajectory dictionary if successful, None if planning fails

**Return Structure:**
```python
{
    'waypoints': List[np.ndarray],      # Robot configurations
    'timestamps': List[float],          # Time at each waypoint
    'zmp_positions': List[np.ndarray],  # ZMP positions [x, y]
    'joint_angles': Dict[str, List],    # Joint angles by name
    'duration': float,                  # Total duration (seconds)
    'success': bool                     # Whether planning succeeded
}
```

**Example:**
```python
import numpy as np

planner = MotionPlanner()
target = np.array([0.5, 0.3, 1.2])

trajectory = planner.plan_reaching_task(
    target_position=target,
    zmp_margin=0.02,
    max_planning_time=5.0
)

if trajectory and trajectory['success']:
    print(f"Planning succeeded! Duration: {trajectory['duration']:.2f}s")
    print(f"Number of waypoints: {len(trajectory['waypoints'])}")
else:
    print("Planning failed")
```

#### `visualize_trajectory`

```python
def visualize_trajectory(
    self,
    trajectory: Dict,
    show_zmp: bool = True,
    show_joints: bool = True,
    save_path: str = None
) -> None
```

Visualize a planned trajectory.

**Parameters:**
- `trajectory` (dict): Trajectory dictionary from `plan_reaching_task`
- `show_zmp` (bool): Whether to show ZMP trajectory
- `show_joints` (bool): Whether to show joint angle profiles
- `save_path` (str, optional): Path to save figure

**Example:**
```python
planner = MotionPlanner()
trajectory = planner.plan_reaching_task(target)

# Show visualization
planner.visualize_trajectory(trajectory)

# Save to file
planner.visualize_trajectory(trajectory, save_path="output/trajectory.png")
```

#### `validate_trajectory`

```python
def validate_trajectory(
    self,
    trajectory: Dict
) -> Tuple[bool, List[str]]
```

Validate a trajectory for safety and feasibility.

**Parameters:**
- `trajectory` (dict): Trajectory to validate

**Returns:**
- `Tuple[bool, List[str]]`: (is_valid, list_of_issues)

**Example:**
```python
is_valid, issues = planner.validate_trajectory(trajectory)
if is_valid:
    print("Trajectory is valid!")
else:
    print("Issues found:")
    for issue in issues:
        print(f"  - {issue}")
```

---

## ZMPConstraint

Zero Moment Point constraint implementation.

### Class: `ZMPConstraint`

```python
class ZMPConstraint:
    """
    Enforces Zero Moment Point constraints for stability.
    
    The ZMP must remain within the support polygon during motion.
    """
```

### Constructor

```python
def __init__(
    self,
    margin: float = 0.02,
    support_polygon: np.ndarray = None
)
```

**Parameters:**
- `margin` (float): Safety margin from polygon edge (meters)
- `support_polygon` (np.ndarray, optional): Custom support polygon vertices

**Example:**
```python
from humanoid_planner import ZMPConstraint

# Default constraint
constraint = ZMPConstraint(margin=0.02)

# Custom support polygon
polygon = np.array([
    [0.1, 0.05],
    [0.1, -0.05],
    [-0.1, -0.05],
    [-0.1, 0.05]
])
constraint = ZMPConstraint(margin=0.03, support_polygon=polygon)
```

### Methods

#### `compute_zmp`

```python
def compute_zmp(
    self,
    forces: np.ndarray,
    positions: np.ndarray
) -> np.ndarray
```

Compute ZMP from ground reaction forces.

**Parameters:**
- `forces` (np.ndarray): Ground reaction forces at contact points
- `positions` (np.ndarray): Contact point positions

**Returns:**
- `np.ndarray`: ZMP position [x, y]

**Example:**
```python
forces = np.array([[0, 0, 100], [0, 0, 150]])  # Two contact points
positions = np.array([[0.1, 0.05, 0], [-0.1, -0.05, 0]])

zmp = constraint.compute_zmp(forces, positions)
print(f"ZMP position: {zmp}")
```

#### `is_stable`

```python
def is_stable(
    self,
    zmp_position: np.ndarray
) -> bool
```

Check if ZMP is within the support polygon.

**Parameters:**
- `zmp_position` (np.ndarray): ZMP position [x, y]

**Returns:**
- `bool`: True if stable (ZMP inside polygon with margin)

**Example:**
```python
zmp = np.array([0.05, 0.02])
if constraint.is_stable(zmp):
    print("Robot is stable")
else:
    print("Robot is unstable!")
```

---

## SupportPolygonCalculator

Computes the support polygon from foot contacts.

### Class: `SupportPolygonCalculator`

```python
class SupportPolygonCalculator:
    """Calculate support polygon from contact points."""
```

### Methods

#### `compute_support_polygon`

```python
def compute_support_polygon(
    self,
    contact_points: np.ndarray
) -> np.ndarray
```

Compute convex hull of contact points.

**Parameters:**
- `contact_points` (np.ndarray): 3D contact points on ground

**Returns:**
- `np.ndarray`: 2D polygon vertices in counter-clockwise order

**Example:**
```python
from humanoid_planner import SupportPolygonCalculator

calculator = SupportPolygonCalculator()

# Foot contact points
contacts = np.array([
    [0.1, 0.05, 0],   # Right foot front
    [0.1, -0.05, 0],  # Right foot back
    [-0.1, 0.05, 0],  # Left foot front
    [-0.1, -0.05, 0]  # Left foot back
])

polygon = calculator.compute_support_polygon(contacts)
print(f"Support polygon: {polygon}")
```

---

## TrajectoryOptimizer

Optimizes trajectories for smoothness and efficiency.

### Class: `TrajectoryOptimizer`

```python
class TrajectoryOptimizer:
    """Optimize trajectories subject to constraints."""
```

### Methods

#### `optimize`

```python
def optimize(
    self,
    waypoints: List[np.ndarray],
    constraints: List,
    objective: str = "time_optimal"
) -> Dict
```

Optimize a trajectory.

**Parameters:**
- `waypoints` (List[np.ndarray]): Initial waypoints
- `constraints` (List): List of constraint objects
- `objective` (str): Optimization objective ("time_optimal", "energy_optimal", "smooth")

**Returns:**
- `dict`: Optimized trajectory

**Example:**
```python
from humanoid_planner import TrajectoryOptimizer, ZMPConstraint

optimizer = TrajectoryOptimizer()
constraint = ZMPConstraint(margin=0.02)

optimized = optimizer.optimize(
    waypoints=initial_waypoints,
    constraints=[constraint],
    objective="time_optimal"
)
```

---

## Kinematics

Forward and inverse kinematics solvers.

### Class: `Kinematics`

```python
class Kinematics:
    """Kinematics solver for humanoid robots."""
```

### Methods

#### `forward_kinematics`

```python
def forward_kinematics(
    self,
    joint_angles: Dict[str, float]
) -> np.ndarray
```

Compute end-effector position from joint angles.

**Parameters:**
- `joint_angles` (Dict[str, float]): Joint angles by name

**Returns:**
- `np.ndarray`: End-effector position [x, y, z]

**Example:**
```python
from humanoid_planner import Kinematics

kin = Kinematics()
joints = {
    'shoulder_pitch': 0.5,
    'shoulder_roll': 0.2,
    'elbow': 1.0,
    'wrist_pitch': 0.3
}

ee_pos = kin.forward_kinematics(joints)
print(f"End-effector position: {ee_pos}")
```

#### `inverse_kinematics`

```python
def inverse_kinematics(
    self,
    target_position: np.ndarray,
    initial_guess: Dict[str, float] = None
) -> Optional[Dict[str, float]]
```

Compute joint angles to reach target position.

**Parameters:**
- `target_position` (np.ndarray): Target position [x, y, z]
- `initial_guess` (Dict[str, float], optional): Initial joint angles

**Returns:**
- `Dict[str, float]` or `None`: Joint angles if solution found

**Example:**
```python
target = np.array([0.5, 0.3, 1.2])
solution = kin.inverse_kinematics(target)

if solution:
    print("IK solution found:")
    for joint, angle in solution.items():
        print(f"  {joint}: {angle:.3f} rad")
else:
    print("No IK solution found")
```

#### `compute_jacobian`

```python
def compute_jacobian(
    self,
    joint_angles: Dict[str, float]
) -> np.ndarray
```

Compute Jacobian matrix for velocity control.

**Parameters:**
- `joint_angles` (Dict[str, float]): Current joint configuration

**Returns:**
- `np.ndarray`: Jacobian matrix (6×n for n joints)

---

## StabilityAnalysis

Analyze and verify robot stability.

### Class: `StabilityAnalysis`

```python
class StabilityAnalysis:
    """Stability analysis for humanoid robots."""
```

### Methods

#### `check_stability`

```python
def check_stability(
    self,
    configuration: np.ndarray,
    contact_points: np.ndarray
) -> Tuple[bool, Dict]
```

Check if configuration is stable.

**Parameters:**
- `configuration` (np.ndarray): Robot configuration
- `contact_points` (np.ndarray): Ground contact points

**Returns:**
- `Tuple[bool, Dict]`: (is_stable, stability_metrics)

**Example:**
```python
from humanoid_planner import StabilityAnalysis

analyzer = StabilityAnalysis()
is_stable, metrics = analyzer.check_stability(config, contacts)

print(f"Stable: {is_stable}")
print(f"ZMP margin: {metrics['zmp_margin']:.3f}m")
print(f"CoM height: {metrics['com_height']:.3f}m")
```

---

## Perception

Robot state and environment perception.

### Class: `Perception`

```python
class Perception:
    """Perception module for robot state and environment."""
```

### Methods

#### `get_robot_state`

```python
def get_robot_state(self) -> Dict
```

Get current robot state.

**Returns:**
- `dict`: Robot state including joint angles, velocities, and pose

#### `detect_obstacles`

```python
def detect_obstacles(self) -> List[Dict]
```

Detect obstacles in the environment.

**Returns:**
- `List[Dict]`: List of detected obstacles with positions and sizes

---

## Configuration

### Loading Configuration

```python
from humanoid_planner.config import load_config

config = load_config("config/planner_params.yaml")
```

### Configuration Structure

```yaml
planner:
  zmp_margin: 0.02
  max_velocity: 1.0
  max_acceleration: 2.0
  planning_time: 5.0
  
stability:
  com_height_threshold: 0.05
  support_polygon_margin: 0.01
  
kinematics:
  convergence_threshold: 0.001
  max_iterations: 100
```

---

## Data Structures

### Trajectory Dictionary

```python
{
    'waypoints': List[np.ndarray],      # Shape: (n_waypoints, n_dof)
    'timestamps': List[float],          # Shape: (n_waypoints,)
    'zmp_positions': List[np.ndarray],  # Shape: (n_waypoints, 2)
    'joint_angles': Dict[str, List],    # Joint name -> angles
    'duration': float,                  # Total duration in seconds
    'success': bool                     # Planning success flag
}
```

### Robot Configuration

```python
# Joint angles dictionary
configuration = {
    'shoulder_pitch': 0.5,   # radians
    'shoulder_roll': 0.2,
    'elbow': 1.0,
    'wrist_pitch': 0.3,
    # ... more joints
}
```

---

## Utilities

### Visualization Utilities

```python
from humanoid_planner.utils import (
    plot_trajectory,
    plot_zmp,
    plot_joint_profiles,
    save_trajectory
)
```

### File I/O

```python
from humanoid_planner.utils import save_trajectory, load_trajectory

# Save trajectory
save_trajectory(trajectory, "output/my_trajectory.pkl")

# Load trajectory
loaded = load_trajectory("output/my_trajectory.pkl")
```

---

## Error Handling

All functions that can fail return `None` or raise specific exceptions:

```python
from humanoid_planner.exceptions import (
    PlanningFailedError,
    KinematicsError,
    StabilityError,
    ConfigurationError
)

try:
    trajectory = planner.plan_reaching_task(target)
except PlanningFailedError as e:
    print(f"Planning failed: {e}")
except KinematicsError as e:
    print(f"Kinematics error: {e}")
```

---

## Complete Example

Here's a complete example using multiple API components:

```python
#!/usr/bin/env python3
"""Complete API usage example."""

import numpy as np
from humanoid_planner import (
    MotionPlanner,
    ZMPConstraint,
    SupportPolygonCalculator,
    Kinematics,
    StabilityAnalysis
)

# Initialize components
planner = MotionPlanner(config_path="config/planner_params.yaml")
constraint = ZMPConstraint(margin=0.02)
calculator = SupportPolygonCalculator()
kinematics = Kinematics()
stability = StabilityAnalysis()

# Define target
target = np.array([0.5, 0.3, 1.2])

# Check if target is kinematically reachable
ik_solution = kinematics.inverse_kinematics(target)
if ik_solution is None:
    print("Target is kinematically unreachable")
    exit(1)

# Plan trajectory
trajectory = planner.plan_reaching_task(
    target_position=target,
    zmp_margin=0.02,
    max_planning_time=5.0
)

if trajectory is None or not trajectory['success']:
    print("Planning failed")
    exit(1)

# Validate trajectory
is_valid, issues = planner.validate_trajectory(trajectory)
if not is_valid:
    print("Trajectory validation failed:")
    for issue in issues:
        print(f"  - {issue}")
    exit(1)

# Analyze stability at each waypoint
print("\nStability Analysis:")
for i, config in enumerate(trajectory['waypoints']):
    # Dummy contact points for demonstration
    contacts = np.array([
        [0.1, 0.05, 0],
        [0.1, -0.05, 0],
        [-0.1, -0.05, 0],
        [-0.1, 0.05, 0]
    ])
    
    is_stable, metrics = stability.check_stability(config, contacts)
    if not is_stable:
        print(f"  Waypoint {i}: UNSTABLE")
    else:
        print(f"  Waypoint {i}: Stable (margin: {metrics['zmp_margin']:.3f}m)")

# Visualize result
planner.visualize_trajectory(trajectory, save_path="output/result.png")

print(f"\n✓ Successfully planned {len(trajectory['waypoints'])} waypoints")
print(f"✓ Duration: {trajectory['duration']:.2f} seconds")
```

---

**Previous**: [Quick Start Guide](quickstart.md) | **Next**: [Architecture Guide](architecture.md)
