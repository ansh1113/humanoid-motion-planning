# Quick Start Guide

Get up and running with the Humanoid Motion Planning system in under 10 minutes!

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Your First Motion Plan](#your-first-motion-plan)
- [Understanding the Output](#understanding-the-output)
- [Configuration](#configuration)
- [Common Use Cases](#common-use-cases)
- [Next Steps](#next-steps)

## Prerequisites

Before you begin, ensure you have:

- Python 3.8 or higher installed
- pip package manager
- Basic familiarity with Python
- 10 minutes of your time

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/ansh1113/humanoid-motion-planning.git
cd humanoid-motion-planning
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
pip install -e .
```

### 3. Verify Installation

```bash
python -c "import humanoid_planner; print('✓ Ready to go!')"
```

## Your First Motion Plan

Let's plan a simple reaching task where the robot reaches for a target position.

### Using the Command Line

Run the provided example script:

```bash
python scripts/run_reaching_task.py --target 0.5 0.3 1.2
```

This command tells the robot to reach toward position (x=0.5m, y=0.3m, z=1.2m).

**Expected output:**
```
============================================================
Humanoid Reaching Task with ZMP Constraints
============================================================

Loading configuration from config/planner_params.yaml...
Initializing motion planner...
Planning trajectory to target [0.5, 0.3, 1.2]...

✓ Planning successful!
  - Number of waypoints: 50
  - Trajectory duration: 3.2 seconds
  - ZMP constraints satisfied: Yes
```

### Adding Visualization

To see the planned trajectory:

```bash
python scripts/run_reaching_task.py --target 0.5 0.3 1.2 --visualize
```

This will display:
- The robot's trajectory
- ZMP trajectory within the support polygon
- Joint angle profiles over time

### Using the Python API

Create a file called `my_first_plan.py`:

```python
#!/usr/bin/env python3
"""My first motion planning example."""

import numpy as np
from humanoid_planner import MotionPlanner

# Initialize the motion planner
planner = MotionPlanner(config_path="config/planner_params.yaml")

# Define target position (x, y, z in meters)
target = np.array([0.5, 0.3, 1.2])

# Plan the trajectory
print(f"Planning trajectory to target: {target}")
trajectory = planner.plan_reaching_task(
    target_position=target,
    zmp_margin=0.02,  # 2cm safety margin
    max_planning_time=5.0
)

# Check if planning succeeded
if trajectory is not None:
    print(f"✓ Success! Generated {len(trajectory)} waypoints")
    
    # Visualize the result
    planner.visualize_trajectory(trajectory)
else:
    print("✗ Planning failed - target may be unreachable")
```

Run it:

```bash
python my_first_plan.py
```

## Understanding the Output

### Trajectory Structure

A planned trajectory contains:

```python
trajectory = {
    'waypoints': [...],      # List of robot configurations
    'timestamps': [...],     # Time at each waypoint
    'zmp_positions': [...],  # ZMP position at each point
    'joint_angles': {...},   # Joint angles for each waypoint
    'duration': 3.2,         # Total trajectory duration (seconds)
}
```

### Success Indicators

✓ **Planning succeeded** when:
- ZMP stays within support polygon
- Target is reached within tolerance
- No joint limits violated
- No self-collisions detected

✗ **Planning may fail** when:
- Target is out of reach
- ZMP constraints cannot be satisfied
- Planning time limit exceeded
- Invalid initial configuration

## Configuration

### Basic Configuration

The main configuration file is `config/planner_params.yaml`:

```yaml
planner:
  zmp_margin: 0.02          # Safety margin (meters)
  max_velocity: 1.0         # Max joint velocity (rad/s)
  max_acceleration: 2.0     # Max joint acceleration (rad/s²)
  planning_time: 5.0        # Max planning time (seconds)
  
stability:
  com_height_threshold: 0.05
  support_polygon_margin: 0.01
```

### Modifying Parameters

You can override parameters programmatically:

```python
from humanoid_planner import MotionPlanner

planner = MotionPlanner(
    config_path="config/planner_params.yaml",
    zmp_margin=0.03,  # Override: use 3cm margin instead
    max_planning_time=10.0  # Override: allow 10 seconds
)
```

### Common Parameter Adjustments

| Parameter | Increase to... | Decrease to... |
|-----------|---------------|----------------|
| `zmp_margin` | Be more conservative | Allow closer to edge |
| `max_velocity` | Move faster | Move slower, more stable |
| `planning_time` | Try harder to find solution | Get faster results |
| `com_height_threshold` | Allow more vertical motion | Maintain flatter CoM |

## Common Use Cases

### Example 1: Reaching Task with Custom Target

```python
from humanoid_planner import MotionPlanner
import numpy as np

planner = MotionPlanner()

# Reach to the right
right_target = np.array([0.4, -0.3, 1.1])
traj1 = planner.plan_reaching_task(target_position=right_target)

# Reach forward
forward_target = np.array([0.6, 0.0, 1.0])
traj2 = planner.plan_reaching_task(target_position=forward_target)
```

### Example 2: Batch Processing Multiple Targets

```python
from humanoid_planner import MotionPlanner
import numpy as np

planner = MotionPlanner()

# Define multiple target positions
targets = [
    np.array([0.5, 0.3, 1.2]),
    np.array([0.4, -0.2, 1.1]),
    np.array([0.6, 0.0, 1.0]),
]

# Plan trajectories for all targets
trajectories = []
for i, target in enumerate(targets):
    print(f"Planning for target {i+1}/{len(targets)}: {target}")
    traj = planner.plan_reaching_task(target_position=target)
    if traj is not None:
        trajectories.append(traj)
        print(f"  ✓ Success")
    else:
        print(f"  ✗ Failed")

print(f"\nSuccessfully planned {len(trajectories)}/{len(targets)} trajectories")
```

### Example 3: Conservative Planning with Stricter Constraints

```python
from humanoid_planner import MotionPlanner
import numpy as np

# Initialize with conservative settings
planner = MotionPlanner(
    config_path="config/planner_params.yaml",
    zmp_margin=0.05,         # Larger safety margin
    max_velocity=0.5,        # Slower movements
    max_acceleration=1.0     # Gentler accelerations
)

target = np.array([0.5, 0.3, 1.2])
trajectory = planner.plan_reaching_task(target_position=target)

if trajectory:
    print("Conservative trajectory planned successfully!")
```

### Example 4: Visualizing ZMP Trajectory

```python
from humanoid_planner import MotionPlanner
import matplotlib.pyplot as plt
import numpy as np

planner = MotionPlanner()
target = np.array([0.5, 0.3, 1.2])
trajectory = planner.plan_reaching_task(target_position=target)

if trajectory:
    # Extract ZMP positions
    zmp_x = [pos[0] for pos in trajectory['zmp_positions']]
    zmp_y = [pos[1] for pos in trajectory['zmp_positions']]
    
    # Plot ZMP trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(zmp_x, zmp_y, 'b-', linewidth=2, label='ZMP Trajectory')
    plt.scatter(zmp_x[0], zmp_y[0], c='green', s=100, label='Start', zorder=5)
    plt.scatter(zmp_x[-1], zmp_y[-1], c='red', s=100, label='End', zorder=5)
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('ZMP Trajectory')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.show()
```

## Troubleshooting Quick Fixes

### Issue: Planning fails with "No solution found"

**Quick Fix:**
```python
# Increase planning time
planner = MotionPlanner(max_planning_time=10.0)

# Or reduce constraints
planner = MotionPlanner(zmp_margin=0.01)
```

### Issue: "Target unreachable"

**Quick Fix:**
```python
# Check if target is within workspace
# Try a closer target
target = np.array([0.3, 0.2, 1.0])  # Closer to robot
```

### Issue: Slow planning

**Quick Fix:**
```python
# Reduce planning time limit for faster results
planner = MotionPlanner(max_planning_time=2.0)
```

## Next Steps

Now that you've completed your first motion plan, explore more:

### Learn More

- **[API Reference](api_reference.md)** - Detailed API documentation
- **[Architecture Guide](architecture.md)** - Understanding system design
- **[Examples Directory](../examples/)** - More complex examples

### Try Advanced Features

1. **Trajectory Optimization**
   ```bash
   python examples/complete_demo.py
   ```

2. **Custom Constraints**
   - Explore `src/humanoid_planner/zmp_constraint.py`
   - Implement your own constraint classes

3. **Integration with Drake**
   ```bash
   pip install drake
   # Unlock advanced simulation features
   ```

### Explore Configuration

- `config/planner_params.yaml` - Motion planning parameters
- `config/robot_config.yaml` - Robot model configuration
- `config/perception_config.yaml` - Perception settings

### Join the Community

- Report issues: [GitHub Issues](https://github.com/ansh1113/humanoid-motion-planning/issues)
- View source: [GitHub Repository](https://github.com/ansh1113/humanoid-motion-planning)
- Contact: anshbhansali5@gmail.com

## Summary

You've learned how to:

- ✓ Install the humanoid motion planner
- ✓ Plan your first reaching trajectory
- ✓ Use both CLI and Python API
- ✓ Understand and visualize results
- ✓ Adjust configuration parameters
- ✓ Handle common issues

**Time invested**: ~10 minutes  
**Skills gained**: Motion planning fundamentals  
**Next level**: Advanced features and customization

---

**Previous**: [Installation Guide](installation.md) | **Next**: [API Reference](api_reference.md)
