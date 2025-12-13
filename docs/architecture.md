# System Architecture

This document describes the system architecture and design principles of the Humanoid Motion Planning system.

## Table of Contents

- [Overview](#overview)
- [Architecture Principles](#architecture-principles)
- [System Components](#system-components)
- [Data Flow](#data-flow)
- [Module Interactions](#module-interactions)
- [Design Patterns](#design-patterns)
- [Extensibility](#extensibility)
- [Performance Considerations](#performance-considerations)

---

## Overview

The Humanoid Motion Planning system is designed as a modular, extensible framework for planning safe reaching motions while maintaining balance. The architecture follows a layered approach with clear separation of concerns.

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User Interface                       │
│              (CLI / Python API / ROS)                   │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                 Motion Planner Core                     │
│         (Trajectory Planning & Optimization)            │
└──┬──────────────┬─────────────┬──────────────┬─────────┘
   │              │             │              │
   ▼              ▼             ▼              ▼
┌──────┐    ┌──────────┐  ┌──────────┐  ┌──────────┐
│ ZMP  │    │Kinematics│  │Stability │  │Trajectory│
│Const.│    │          │  │Analysis  │  │Optimizer │
└──────┘    └──────────┘  └──────────┘  └──────────┘
   │              │             │              │
   └──────────────┴─────────────┴──────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                Support Services                         │
│     (Configuration, Logging, Visualization)             │
└─────────────────────────────────────────────────────────┘
```

---

## Architecture Principles

### 1. Modularity
Each component has a well-defined responsibility and can be used independently or replaced with alternative implementations.

### 2. Separation of Concerns
- **Planning Logic**: Separate from kinematics and dynamics
- **Constraints**: Modular constraint system
- **Visualization**: Decoupled from core planning

### 3. Configuration-Driven
System behavior is controlled through configuration files, not hard-coded parameters.

### 4. Testability
Each module can be unit tested independently with mock dependencies.

### 5. Extensibility
New constraints, objectives, or kinematics solvers can be added without modifying core code.

---

## System Components

### Core Layer

#### 1. Motion Planner (`motion_planner.py`)

**Responsibility**: Orchestrates trajectory planning process

**Key Functions**:
- Coordinate all planning components
- Manage planning state and configuration
- Interface with user requests
- Return validated trajectories

**Dependencies**:
- Kinematics
- Stability Analysis
- Trajectory Optimizer
- ZMP Constraint

**Design Pattern**: Facade pattern - provides simplified interface to complex subsystem

```python
class MotionPlanner:
    def __init__(self, config_path):
        self.config = load_config(config_path)
        self.kinematics = Kinematics()
        self.stability = StabilityAnalysis()
        self.optimizer = TrajectoryOptimizer()
        
    def plan_reaching_task(self, target_position):
        # Orchestrate planning process
        pass
```

#### 2. ZMP Constraint (`zmp_constraint.py`)

**Responsibility**: Enforce Zero Moment Point constraints

**Key Functions**:
- Compute ZMP from forces and positions
- Check if ZMP is within support polygon
- Provide constraint gradients for optimization

**Design Pattern**: Strategy pattern - implements constraint interface

```python
class ZMPConstraint(ConstraintBase):
    def evaluate(self, state):
        # Check constraint satisfaction
        pass
        
    def gradient(self, state):
        # Provide constraint gradient
        pass
```

#### 3. Support Polygon (`support_polygon.py`)

**Responsibility**: Calculate support polygon from contact points

**Key Functions**:
- Compute convex hull of foot contacts
- Determine if point is inside polygon
- Calculate distance to polygon boundary

**Algorithm**: Quick Hull or Graham Scan for convex hull

```python
class SupportPolygonCalculator:
    def compute_support_polygon(self, contact_points):
        # Project to 2D and compute convex hull
        projected = self._project_to_ground(contact_points)
        hull = self._convex_hull(projected)
        return hull
```

#### 4. Trajectory Optimizer (`trajectory_optimizer.py`)

**Responsibility**: Optimize trajectories for smoothness and efficiency

**Key Functions**:
- Time-optimal trajectory generation
- Energy-optimal trajectory generation
- Smooth trajectory interpolation
- Constraint satisfaction

**Optimization Methods**:
- Sequential Quadratic Programming (SQP)
- Direct Transcription
- Shooting methods

```python
class TrajectoryOptimizer:
    def optimize(self, waypoints, constraints, objective):
        # Set up optimization problem
        # Solve with chosen method
        # Return optimized trajectory
        pass
```

### Support Layer

#### 5. Kinematics (`kinematics.py`)

**Responsibility**: Forward and inverse kinematics

**Key Functions**:
- Forward kinematics (FK)
- Inverse kinematics (IK)
- Jacobian computation
- Center of Mass calculation

**Solvers**:
- Analytical IK (when available)
- Numerical IK (Newton-Raphson, Levenberg-Marquardt)
- Drake-based solvers (optional)

```python
class Kinematics:
    def forward_kinematics(self, joint_angles):
        # Compute end-effector pose
        pass
        
    def inverse_kinematics(self, target_pose):
        # Solve for joint angles
        pass
        
    def compute_jacobian(self, joint_angles):
        # Compute velocity Jacobian
        pass
```

#### 6. Stability Analysis (`stability_analysis.py`)

**Responsibility**: Verify robot stability

**Key Functions**:
- Static stability checking
- Dynamic stability metrics
- Stability margin calculation
- Center of Mass (CoM) tracking

**Criteria**:
- ZMP within support polygon
- CoM height constraints
- Angular momentum limits

```python
class StabilityAnalysis:
    def check_stability(self, configuration, contacts):
        # Compute ZMP
        # Check against support polygon
        # Return stability metrics
        pass
```

#### 7. Perception (`perception.py`)

**Responsibility**: Robot state and environment sensing

**Key Functions**:
- Robot state estimation
- Joint encoder reading
- Force/torque sensor processing
- Obstacle detection (future)

```python
class Perception:
    def get_robot_state(self):
        # Read sensors
        # Filter and process
        # Return state estimate
        pass
```

### Service Layer

#### 8. Configuration Management

**Responsibility**: Load and validate configuration

**Files**:
- `planner_params.yaml` - Planning parameters
- `robot_config.yaml` - Robot model parameters
- `perception_config.yaml` - Sensor parameters

```python
def load_config(config_path):
    # Load YAML
    # Validate schema
    # Apply defaults
    # Return config dict
    pass
```

#### 9. Visualization

**Responsibility**: Visualize trajectories and results

**Tools**:
- Matplotlib for 2D plots
- Optional: PyQt/PyQtGraph for 3D visualization
- Optional: RViz for robot visualization

#### 10. Logging

**Responsibility**: Track system behavior and debug

**Levels**:
- DEBUG: Detailed diagnostic information
- INFO: General informational messages
- WARNING: Warning messages
- ERROR: Error messages

---

## Data Flow

### Planning Request Flow

```
1. User Request
   └─> MotionPlanner.plan_reaching_task(target)
   
2. Preprocessing
   └─> Validate target
   └─> Check reachability (IK)
   └─> Initialize trajectory
   
3. Planning Loop
   ├─> Generate candidate trajectory
   ├─> Check ZMP constraints
   ├─> Check stability
   ├─> Optimize trajectory
   └─> Validate solution
   
4. Post-processing
   └─> Smooth trajectory
   └─> Add timestamps
   └─> Compute ZMP trajectory
   
5. Return Result
   └─> Trajectory dict or None
```

### Data Structures

#### Robot Configuration
```python
config = {
    'joint_angles': Dict[str, float],
    'joint_velocities': Dict[str, float],
    'pose': np.ndarray,  # [x, y, z, qx, qy, qz, qw]
}
```

#### Trajectory
```python
trajectory = {
    'waypoints': List[np.ndarray],
    'timestamps': List[float],
    'zmp_positions': List[np.ndarray],
    'joint_angles': Dict[str, List[float]],
    'duration': float,
    'success': bool
}
```

#### Contact Information
```python
contacts = {
    'positions': np.ndarray,  # (n, 3)
    'forces': np.ndarray,     # (n, 3)
    'normals': np.ndarray,    # (n, 3)
}
```

---

## Module Interactions

### Interaction Diagram

```
┌─────────────┐
│   User      │
└──────┬──────┘
       │ plan_reaching_task(target)
       ▼
┌─────────────────────┐
│  MotionPlanner      │
└──┬──────────────────┘
   │
   ├─> Kinematics.inverse_kinematics(target)
   │   └─> returns: joint_angles or None
   │
   ├─> TrajectoryOptimizer.generate_initial_trajectory(start, goal)
   │   └─> returns: initial_waypoints
   │
   ├─> For each waypoint:
   │   ├─> SupportPolygon.compute_support_polygon(contacts)
   │   │   └─> returns: polygon_vertices
   │   │
   │   ├─> ZMPConstraint.compute_zmp(forces, positions)
   │   │   └─> returns: zmp_position
   │   │
   │   └─> ZMPConstraint.is_stable(zmp, polygon)
   │       └─> returns: True/False
   │
   ├─> TrajectoryOptimizer.optimize(waypoints, constraints)
   │   └─> returns: optimized_trajectory
   │
   └─> StabilityAnalysis.validate(trajectory)
       └─> returns: (is_valid, metrics)
```

### Communication Patterns

#### 1. Synchronous Calls
Most module interactions are synchronous function calls for simplicity and determinism.

#### 2. Configuration Injection
Modules receive configuration at initialization time.

#### 3. State Independence
Modules are generally stateless (except for cached data like Jacobians).

---

## Design Patterns

### 1. Facade Pattern
**Where**: `MotionPlanner` class  
**Why**: Simplifies complex subsystem interaction

### 2. Strategy Pattern
**Where**: Constraint classes  
**Why**: Allows different constraint types to be used interchangeably

### 3. Factory Pattern
**Where**: Configuration loading  
**Why**: Creates objects based on configuration

### 4. Observer Pattern (Future)
**Where**: Real-time execution monitoring  
**Why**: Allows multiple components to react to state changes

### 5. Singleton Pattern
**Where**: Logger  
**Why**: Ensures single logging instance

---

## Extensibility

### Adding New Constraints

```python
from humanoid_planner.constraints import ConstraintBase

class MyCustomConstraint(ConstraintBase):
    def evaluate(self, state):
        # Return constraint violation (0 = satisfied)
        pass
        
    def gradient(self, state):
        # Return constraint gradient for optimization
        pass

# Use in planner
planner = MotionPlanner()
custom_constraint = MyCustomConstraint()
planner.add_constraint(custom_constraint)
```

### Adding New Optimization Objectives

```python
from humanoid_planner.objectives import ObjectiveBase

class MyObjective(ObjectiveBase):
    def evaluate(self, trajectory):
        # Return objective value (lower is better)
        pass
        
    def gradient(self, trajectory):
        # Return gradient for optimization
        pass
```

### Adding New Kinematics Solvers

```python
from humanoid_planner.kinematics import KinematicsBase

class MyKinematicsSolver(KinematicsBase):
    def forward_kinematics(self, joint_angles):
        # Your FK implementation
        pass
        
    def inverse_kinematics(self, target_pose):
        # Your IK implementation
        pass
```

---

## Performance Considerations

### Computational Bottlenecks

1. **Inverse Kinematics**: O(n × iterations)
   - Cached for repeated calls with same target
   - Warm-started from previous solution

2. **Trajectory Optimization**: O(n² × iterations)
   - Most computationally expensive
   - Can be parallelized for multiple targets

3. **Convex Hull Computation**: O(n log n)
   - Fast for typical number of contacts (4-8 points)
   - Cached when contacts don't change

4. **ZMP Calculation**: O(n)
   - Linear in number of contacts
   - Very fast in practice

### Optimization Strategies

#### 1. Caching
```python
class Kinematics:
    def __init__(self):
        self._jacobian_cache = {}
        
    def compute_jacobian(self, joint_angles):
        key = tuple(joint_angles)
        if key not in self._jacobian_cache:
            self._jacobian_cache[key] = self._compute_jacobian_impl(joint_angles)
        return self._jacobian_cache[key]
```

#### 2. Lazy Evaluation
Only compute expensive values when needed.

#### 3. Vectorization
Use NumPy operations instead of Python loops.

```python
# Bad: Python loop
for i in range(len(positions)):
    zmp += forces[i] * positions[i]

# Good: Vectorized
zmp = np.sum(forces * positions, axis=0)
```

#### 4. Sparse Matrices
Use sparse matrices for large Jacobians.

### Memory Management

- **Trajectory Storage**: O(n × d) where n = waypoints, d = DOF
- **Jacobian Cache**: O(k × n × m) where k = cache size
- **Support Polygon**: O(1) - small fixed size

### Scalability

The system scales well with:
- ✓ Number of planning requests (stateless design)
- ✓ Robot DOF (up to ~50 joints tested)
- ✗ Real-time constraints (planning takes 2-5 seconds)

For real-time applications, consider:
- Pre-computing motion primitives
- Using learning-based methods for initialization
- Implementing model predictive control (MPC)

---

## Testing Architecture

### Test Organization

```
tests/
├── test_motion_planner.py      # Integration tests
├── test_zmp_constraint.py      # Unit tests
├── test_kinematics.py          # Unit tests
├── test_stability_analysis.py  # Unit tests
└── test_integration.py         # End-to-end tests
```

### Test Strategies

1. **Unit Tests**: Test individual modules in isolation
2. **Integration Tests**: Test module interactions
3. **End-to-End Tests**: Test complete planning pipeline
4. **Regression Tests**: Ensure bug fixes stay fixed

### Mock Objects

```python
class MockKinematics:
    def inverse_kinematics(self, target):
        # Return predetermined solution
        return {'joint1': 0.5, 'joint2': 1.0}
```

---

## Future Architecture Enhancements

### 1. Async Planning
Support asynchronous planning requests with callbacks.

### 2. ROS2 Integration
Full ROS2 node architecture with topics and services.

### 3. Learning-Based Components
Integrate learned models for faster planning initialization.

### 4. Multi-Contact Planning
Extend to hands + feet contacts for more complex tasks.

### 5. Online Replanning
Support dynamic replanning for moving targets.

### 6. Distributed Computing
Parallelize optimization across multiple cores/machines.

---

## Summary

The Humanoid Motion Planning system is designed with:

- ✅ **Modularity**: Clear component boundaries
- ✅ **Extensibility**: Easy to add new features
- ✅ **Testability**: Comprehensive test coverage
- ✅ **Performance**: Optimized for typical use cases
- ✅ **Maintainability**: Clean, documented code

This architecture supports both research experimentation and practical applications while maintaining code quality and performance.

---

**Previous**: [API Reference](api_reference.md) | **Back to**: [README](../README.md)
