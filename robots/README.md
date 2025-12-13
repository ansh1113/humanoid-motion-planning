# Robot Models and URDF Documentation

This directory contains documentation and examples for robot models used with the Humanoid Motion Planning system.

## Table of Contents

- [Overview](#overview)
- [URDF Format](#urdf-format)
- [Supported Robot Models](#supported-robot-models)
- [Creating Custom Robot Models](#creating-custom-robot-models)
- [Robot Configuration](#robot-configuration)
- [Example URDF Structure](#example-urdf-structure)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

---

## Overview

The Humanoid Motion Planning system uses robot models defined in the Unified Robot Description Format (URDF) for kinematics and dynamics calculations. While the system doesn't require a specific robot model to be present, having a proper URDF allows for more accurate planning and simulation.

### What is URDF?

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It describes:
- Robot links (rigid bodies)
- Joints (connections between links)
- Visual and collision geometry
- Inertial properties
- Sensors and actuators

---

## URDF Format

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0.0 0.2 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- More links and joints... -->
  
</robot>
```

### Required Elements

1. **Links**: Physical components of the robot
   - `visual`: How the link appears (for visualization)
   - `collision`: Simplified geometry for collision detection
   - `inertial`: Mass and inertia properties

2. **Joints**: Connections between links
   - `parent` and `child`: Links being connected
   - `origin`: Position and orientation of joint
   - `axis`: Rotation axis (for revolute/prismatic joints)
   - `limit`: Joint limits (position, velocity, effort)

3. **Joint Types**:
   - `revolute`: Rotating joint with limits
   - `continuous`: Rotating joint without limits
   - `prismatic`: Sliding joint
   - `fixed`: Rigid connection
   - `floating`: 6-DOF free joint (for base)

---

## Supported Robot Models

The system is designed to work with humanoid robots having:

### Minimum Requirements

- **Upper body**: At least one arm with 3+ DOF
- **Base/Torso**: Fixed or floating base
- **Contact points**: Defined contact geometry for feet

### Tested Configurations

1. **Basic Humanoid** (7-DOF arm)
   - 3-DOF shoulder (pitch, roll, yaw)
   - 1-DOF elbow
   - 3-DOF wrist (pitch, roll, yaw)

2. **Full Humanoid** (32-DOF)
   - 7-DOF per arm (×2)
   - 6-DOF torso (floating base)
   - 6-DOF per leg (×2)

3. **Simple Manipulator** (6-DOF)
   - Fixed base
   - 6-DOF arm for reaching tasks

---

## Creating Custom Robot Models

### Step 1: Define the Kinematic Chain

Identify the kinematic chain from base to end-effector:

```
base_link -> torso -> shoulder -> upper_arm -> forearm -> wrist -> hand
```

### Step 2: Create Basic URDF

Start with a simple URDF defining only the kinematic structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  
  <link name="base_link"/>
  
  <link name="torso">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.5"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
  
  <link name="shoulder">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="shoulder"/>
    <origin xyz="0 0.2 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- Add more links and joints... -->
  
</robot>
```

### Step 3: Add Physical Properties

Add mass and inertia for dynamics:

```xml
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <inertia ixx="0.015" ixy="0.0" ixz="0.0"
             iyy="0.015" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

### Step 4: Define Contact Points

For stability analysis, define foot contact geometry:

```xml
<link name="foot">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
</link>
```

### Step 5: Validate URDF

Use `check_urdf` tool to validate:

```bash
check_urdf my_robot.urdf
```

Or in Python:

```python
import xml.etree.ElementTree as ET

def validate_urdf(urdf_path):
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        assert root.tag == 'robot', "Root element must be 'robot'"
        print("✓ URDF is valid XML")
        return True
    except Exception as e:
        print(f"✗ URDF validation failed: {e}")
        return False

validate_urdf("my_robot.urdf")
```

---

## Robot Configuration

### Configuration File

In addition to URDF, create a configuration file at `config/robot_config.yaml`:

```yaml
robot:
  name: "my_humanoid"
  urdf_path: "robots/my_robot.urdf"
  
  # End-effector link
  end_effector_link: "hand"
  
  # Base link
  base_link: "base_link"
  
  # Contact links (for foot placement)
  contact_links:
    - "left_foot"
    - "right_foot"
  
  # Joint groups
  arm_joints:
    - "shoulder_pitch"
    - "shoulder_roll"
    - "shoulder_yaw"
    - "elbow"
    - "wrist_pitch"
    - "wrist_roll"
    - "wrist_yaw"
  
  # Joint limits (can override URDF)
  joint_limits:
    shoulder_pitch: [-1.57, 1.57]
    shoulder_roll: [-1.57, 1.57]
    elbow: [0.0, 2.5]
  
  # Physical properties
  total_mass: 50.0  # kg
  height: 1.7       # meters
  
  # Workspace limits
  workspace:
    x: [0.0, 0.8]    # meters
    y: [-0.6, 0.6]
    z: [0.5, 1.8]
```

### Loading Configuration

```python
from humanoid_planner import MotionPlanner

# Load with custom robot config
planner = MotionPlanner(
    config_path="config/planner_params.yaml",
    robot_config_path="config/robot_config.yaml"
)
```

---

## Example URDF Structure

### Minimal Working Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Base -->
  <link name="base_link"/>
  
  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.2"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
  
  <!-- Right Arm Chain -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0 -0.2 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <link name="right_upper_arm">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.015" ixy="0" ixz="0"
               iyy="0.015" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </visual>
  </link>
  
  <joint name="right_shoulder_roll" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- More links for complete arm... -->
  
  <!-- Feet (for ground contact) -->
  <link name="right_foot">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Define left side similarly... -->
  
</robot>
```

---

## Best Practices

### 1. Naming Conventions

- Use descriptive link names: `upper_arm`, `forearm`, `hand`
- Use consistent joint names: `{side}_{joint}_{axis}`
  - Example: `right_shoulder_pitch`, `left_elbow`

### 2. Coordinate Frames

- Follow ROS conventions:
  - X: forward
  - Y: left
  - Z: up
- Use right-hand rule for rotations

### 3. Mass Properties

- Accurate masses improve planning quality
- If unknown, use reasonable estimates:
  - Torso: 20-30 kg
  - Upper arm: 2-3 kg
  - Forearm: 1-2 kg
  - Hand: 0.5-1 kg

### 4. Joint Limits

- Always define realistic joint limits
- Add safety margins to physical limits
- Example:
  ```xml
  <!-- Physical limit: ±90°, use ±85° in URDF -->
  <limit lower="-1.48" upper="1.48" ... />
  ```

### 5. Collision Geometry

- Use simplified geometry for collision detection
- Convex hulls work best
- Avoid complex meshes in collision geometry

### 6. Visual vs Collision

- Visual: Can be detailed meshes
- Collision: Should be simple primitives
- Collision geometry can be larger than visual for safety margin

---

## Troubleshooting

### Issue: "Cannot find link in URDF"

**Solution**: Verify link names match exactly:
```bash
grep -n "link name=" my_robot.urdf
```

### Issue: "Invalid joint limits"

**Solution**: Check that lower < upper:
```xml
<limit lower="-1.57" upper="1.57" ... />
```

### Issue: "Kinematics failed to solve"

**Possible causes**:
1. Target outside workspace
2. Joint limits too restrictive
3. Invalid DH parameters

**Solution**: Validate forward kinematics first:
```python
from humanoid_planner import Kinematics

kin = Kinematics()
# Test with known joint angles
pos = kin.forward_kinematics({'shoulder_pitch': 0.5, ...})
print(f"End-effector at: {pos}")
```

### Issue: "URDF parse error"

**Solution**: Validate XML structure:
```bash
xmllint --noout my_robot.urdf
```

Or use online URDF validators.

---

## Additional Resources

### Tools

- **URDF Tutorial**: http://wiki.ros.org/urdf/Tutorials
- **URDF Validator**: http://wiki.ros.org/urdf#Verification
- **SolidWorks to URDF**: http://wiki.ros.org/sw_urdf_exporter
- **Blender to URDF**: Various plugins available

### Examples

Example URDF files for common robots:
- [PR2 Robot](https://github.com/PR2/pr2_common)
- [Atlas Robot](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/atlas)
- [Simple Humanoid](https://github.com/ros/robot_state_publisher/tree/humble/test)

### References

1. Official URDF Specification: http://wiki.ros.org/urdf/XML
2. Drake URDF Parser: https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html
3. Robot Description Best Practices: http://wiki.ros.org/urdf/Best_Practices

---

## Getting Help

If you need help with robot models:

1. Check existing examples in this directory
2. Consult the [URDF tutorials](http://wiki.ros.org/urdf/Tutorials)
3. Open an issue: https://github.com/ansh1113/humanoid-motion-planning/issues
4. Contact: anshbhansali5@gmail.com

---

**Note**: Currently, the motion planning system works without a specific URDF file by using a simplified kinematic model. Providing a URDF enables more accurate planning and simulation with Drake.

---

**Back to**: [Main README](../README.md) | **See also**: [Architecture Guide](../docs/architecture.md)
