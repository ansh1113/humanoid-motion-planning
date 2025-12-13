# Installation Guide

This guide provides detailed instructions for installing the Humanoid Motion Planning system.

## Table of Contents

- [System Requirements](#system-requirements)
- [Quick Installation](#quick-installation)
- [Detailed Installation Steps](#detailed-installation-steps)
- [Optional Dependencies](#optional-dependencies)
- [Development Setup](#development-setup)
- [Docker Installation](#docker-installation)
- [Troubleshooting](#troubleshooting)
- [Verifying Installation](#verifying-installation)

## System Requirements

### Minimum Requirements

- **Operating System**: Linux (Ubuntu 20.04+), macOS (10.15+), or Windows 10/11 with WSL2
- **Python**: 3.8 or higher
- **RAM**: 4 GB minimum, 8 GB recommended
- **Disk Space**: 2 GB for installation and dependencies

### Recommended Requirements

- **Python**: 3.10 or higher
- **RAM**: 16 GB for large-scale simulations
- **GPU**: Optional, but recommended for visualization

## Quick Installation

The fastest way to get started:

```bash
# Clone the repository
git clone https://github.com/ansh1113/humanoid-motion-planning.git
cd humanoid-motion-planning

# Install dependencies
pip install -r requirements.txt

# Install the package in development mode
pip install -e .

# Verify installation
python -c "import humanoid_planner; print('Installation successful!')"
```

## Detailed Installation Steps

### Step 1: Clone the Repository

```bash
git clone https://github.com/ansh1113/humanoid-motion-planning.git
cd humanoid-motion-planning
```

### Step 2: Create a Virtual Environment (Recommended)

Using `venv`:

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

Or using `conda`:

```bash
conda create -n humanoid-planner python=3.10
conda activate humanoid-planner
```

### Step 3: Install Dependencies

Install the required Python packages:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

This will install:
- NumPy (≥1.21.0) - Numerical computations
- SciPy (≥1.7.0) - Scientific computing and optimization
- Matplotlib (≥3.4.0) - Visualization
- PyYAML (≥5.4.0) - Configuration file parsing

### Step 4: Install the Package

Install the `humanoid_planner` package in development mode:

```bash
pip install -e .
```

This allows you to modify the source code and see changes immediately without reinstalling.

## Optional Dependencies

### Drake (Recommended for Advanced Features)

Drake provides advanced optimization and simulation capabilities:

```bash
pip install drake
```

**Note**: Drake installation can be large (~500 MB) and may take some time.

### Visualization Dependencies

For enhanced 3D visualization:

```bash
pip install pyqt5 pyqtgraph
```

## Development Setup

If you plan to contribute or develop features:

### Step 1: Install Development Dependencies

```bash
pip install -r requirements-dev.txt
```

This includes:
- pytest - Testing framework
- pytest-cov - Code coverage
- black - Code formatter
- flake8 - Linter
- mypy - Type checker
- pre-commit - Git hooks

### Step 2: Set Up Pre-commit Hooks

```bash
pre-commit install
```

This ensures code quality checks run automatically before each commit.

### Step 3: Run Tests

Verify your development setup:

```bash
pytest tests/ -v
```

## Docker Installation

Docker provides an isolated, reproducible environment.

### Step 1: Install Docker

Follow the [official Docker installation guide](https://docs.docker.com/get-docker/) for your platform.

### Step 2: Build the Docker Image

```bash
docker-compose build
```

### Step 3: Run the Container

```bash
# Run a command in the container
docker-compose run humanoid-planner python scripts/run_reaching_task.py --target 0.5 0.3 1.2

# Start an interactive shell
docker-compose run humanoid-planner bash
```

### Step 4: Enable Visualization (Linux)

For GUI applications in Docker:

```bash
xhost +local:docker
docker-compose run humanoid-planner python scripts/run_reaching_task.py --visualize
```

## Troubleshooting

### Common Issues

#### Issue: "ModuleNotFoundError: No module named 'humanoid_planner'"

**Solution**: Make sure you've installed the package:
```bash
pip install -e .
```

#### Issue: "ImportError: No module named 'numpy'"

**Solution**: Install dependencies:
```bash
pip install -r requirements.txt
```

#### Issue: Drake installation fails

**Solution**: Drake has specific platform requirements. Check the [Drake installation guide](https://drake.mit.edu/python_bindings.html) or skip Drake for basic functionality:
```bash
# The package works without Drake for basic features
pip install -e .
```

#### Issue: Permission denied when installing

**Solution**: Use the `--user` flag or a virtual environment:
```bash
pip install --user -r requirements.txt
pip install --user -e .
```

### Platform-Specific Issues

#### Windows

- Use Windows Subsystem for Linux (WSL2) for best compatibility
- Some visualization features may require X server (e.g., VcXsrv, Xming)

#### macOS

- Ensure Xcode Command Line Tools are installed:
  ```bash
  xcode-select --install
  ```

#### Linux

- Some systems may require additional packages:
  ```bash
  sudo apt-get update
  sudo apt-get install python3-dev python3-pip build-essential
  ```

## Verifying Installation

### Quick Test

Run a simple test to verify everything is working:

```bash
python -c "
from humanoid_planner import MotionPlanner
import numpy as np
print('✓ Import successful')

# Test basic functionality
try:
    planner = MotionPlanner()
    print('✓ MotionPlanner initialized')
    print('Installation verified successfully!')
except Exception as e:
    print(f'× Error: {e}')
"
```

### Run Example Script

```bash
python scripts/run_reaching_task.py --target 0.5 0.3 1.2
```

Expected output:
```
============================================================
Humanoid Reaching Task with ZMP Constraints
============================================================

Loading configuration from config/planner_params.yaml...
Planning trajectory to target [0.5, 0.3, 1.2]...
Planning successful!
...
```

### Run Test Suite

```bash
pytest tests/ -v
```

All tests should pass. Some warnings are normal.

## Next Steps

After successful installation:

1. **Quick Start**: Follow the [Quick Start Guide](quickstart.md) for a tutorial
2. **API Reference**: Explore the [API Documentation](api_reference.md)
3. **Examples**: Check out the `examples/` directory for more use cases
4. **Configuration**: Customize settings in `config/planner_params.yaml`

## Getting Help

If you encounter issues:

1. Check the [Troubleshooting](#troubleshooting) section above
2. Search [existing issues](https://github.com/ansh1113/humanoid-motion-planning/issues)
3. Open a [new issue](https://github.com/ansh1113/humanoid-motion-planning/issues/new) with:
   - Your Python version (`python --version`)
   - Your OS and version
   - Complete error message
   - Steps to reproduce

## Updating

To update to the latest version:

```bash
cd humanoid-motion-planning
git pull origin main
pip install --upgrade -r requirements.txt
```

---

**Previous**: [README](../README.md) | **Next**: [Quick Start Guide](quickstart.md)
