# Humanoid Whole-Body Motion Planning

<p align="center">
  <img src="media/showcase_demo.gif" width="600"/>
</p>

**Advanced motion planning system for Unitree G1 humanoid robot in MuJoCo simulation.**

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-green.svg)](https://mujoco.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## Key Features

| Feature | Description | Result |
|---------|-------------|--------|
| **Walk + Reach** | Whole-body coordination | 2m walk + 4/4 reach |
| **ZMP Preview Control** | LIPM CoM trajectory (Kajita) | 69cm trajectory |
| **Footstep Planning** | A* search with obstacles | 16 steps |
| **MPC Balance** | Predictive control | 49% less energy |
| **RL Locomotion** | Pre-trained policy | 2.01m @ 0.4m/s |
| **Push Recovery** | Perturbation resistance | 4/4 survived |

## Demo Results

```
<p align="center">
  <img src="results/locomotion.gif" width="45%"/>
  <img src="results/phase2-4.gif" width="45%"/>
</p>

*Left: RL Locomotion (2.01m walk) | Right: Manipulation, Push Recovery, Wave*

╔══════════════════════════════════════════════════════════╗
║              HUMANOID SHOWCASE DEMO                      ║
║        Walk → Reach → Push Recovery → Wave              ║
╚══════════════════════════════════════════════════════════╝

[PHASE 1] Walking 2 meters...
  ✓ Walked 2.01m

[PHASE 2-4] Manipulation, Push Recovery, Wave...
  ✓ Reaching: 4/4 targets
  ✓ Push recovery: 2/2 survived
  ✓ Victory wave: Done!
```

## Quick Start

```bash
# Clone the repository
git clone https://github.com/ansh1113/humanoid-motion-planning.git
cd humanoid-motion-planning

# Create virtual environment
python -m venv venv
source venv/bin/activate

# Install dependencies
pip install mujoco numpy scipy matplotlib torch

# Run the showcase demo
python src/showcase_demo.py
```

## Project Structure

```
humanoid_motion_planning/
├── src/
│   ├── showcase_demo.py           # Video-friendly continuous demo
│   ├── full_visualization.py      # Step-by-step feature demo
│   ├── walk_and_reach.py          # Whole-body coordination
│   ├── zmp_preview_control.py     # LIPM preview control
│   ├── footstep_planner.py        # A* footstep planning
│   ├── mpc_balance.py             # MPC controller
│   └── locomotion/
│       └── g1_walker.py           # RL-based walking
├── results/                        # Output visualizations
├── media/                          # Demo GIFs and videos
├── mujoco_menagerie/unitree_g1/   # Robot model
└── unitree_rl_gym/                # Pre-trained RL policy
```

## Technical Details

### ZMP Preview Control
Classic Kajita LIPM method for CoM trajectory generation:
```
LIPM: x'' = ω²(x - ZMP), ω = √(g/z_c) ≈ 3.6 rad/s
```

### Footstep Planning
A* search with discrete actions:
- Forward: 8-25cm, Lateral: ±12cm, Rotation: ±17°
- Real-time collision checking with obstacles

### MPC Balance
```
State: [x, ẋ], Control: acceleration
Horizon: 25 steps (0.5s), Cost: J = Σ(Q·x² + R·u²)
```

### Jacobian IK
Damped least-squares with waist compensation:
```python
Δq = J^T(JJ^T + λI)^{-1} · error, λ = 0.005
```

## Performance Metrics

| Metric | Value |
|--------|-------|
| Walking Distance | 2.01m |
| Walking Speed | ~0.4 m/s |
| Manipulation Success | 75-100% |
| Push Recovery | 4/4 directions |
| MPC Energy Savings | 49% vs PD |
| ZMP Trajectory | 69cm |
| Footstep Planning | 16 steps w/ obstacles |

## Visualizations

<p align="center">
  <img src="results/demo_footstep.png" width="30%"/>
  <img src="results/demo_zmp.png" width="30%"/>
  <img src="results/demo_mpc.png" width="30%"/>
</p>

*Left: A* footstep planning, Center: ZMP preview trajectories, Right: MPC balance comparison*

## Running the Demos

### Full Visualization (Interactive)
```bash
python src/full_visualization.py
```
6-phase demo with user prompts:
1. Footstep Planning (A*)
2. ZMP Preview Control
3. RL Locomotion
4. Manipulation (Jacobian IK)
5. MPC Balance
6. ZMP Stability Test

### Showcase Demo (Video-friendly)
```bash
python src/showcase_demo.py
```
Continuous demonstration:
- Walking 2m → Reaching → Push Recovery → Wave

### Individual Features
```bash
python src/walk_and_reach.py        # Walk + Reach
python src/zmp_preview_control.py   # ZMP Preview
python src/footstep_planner.py      # A* Planning
python src/mpc_balance.py           # MPC Balance
```

## Dependencies

- Python 3.8+
- MuJoCo 3.0+
- NumPy, SciPy, Matplotlib
- PyTorch (for RL policy)

## References

1. Kajita et al., "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point"
2. Unitree G1 Documentation
3. MuJoCo Physics Engine

## Portfolio Highlights

This project demonstrates:
- ✅ **Whole-body motion planning** (walk + reach)
- ✅ **Classical control** (ZMP, LIPM, preview control)
- ✅ **Modern optimization** (MPC, A* search)
- ✅ **Practical robotics** (IK, trajectory optimization)
- ✅ **Simulation** (MuJoCo integration)


## Future Work

- [ ] Train custom RL locomotion policy in Isaac Sim
- [ ] Vision-based manipulation
- [ ] Dynamic walking with ZMP tracking
- [ ] Real hardware deployment

## License

MIT License - see LICENSE file for details

## Author

**Ansh Bhansali**
- MS in Autonomy & Robotics, UIUC (2026)
- Email: anshbhansali5@gmail.com
- GitHub: [@ansh1113](https://github.com/ansh1113)

---

*Developed as part of robotics portfolio for humanoid robotics positions*
