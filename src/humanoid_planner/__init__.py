"""
Humanoid Motion Planning Package

Core modules (always available):
- ZMPConstraint: Zero Moment Point constraint handling
- SupportPolygonCalculator: Support polygon computation
- TrajectoryOptimizer: Trajectory optimization
- ForwardKinematics, InverseKinematics: Robot kinematics

Optional modules (require Drake):
- MotionPlanner: Full motion planning with Drake simulation
"""

__version__ = "0.1.0"
__author__ = "Ansh Bhansali"

from .zmp_constraint import ZMPConstraint
from .support_polygon import SupportPolygonCalculator
from .trajectory_optimizer import TrajectoryOptimizer
from .kinematics import ForwardKinematics, InverseKinematics

__all__ = [
    "ZMPConstraint",
    "SupportPolygonCalculator",
    "TrajectoryOptimizer",
    "ForwardKinematics",
    "InverseKinematics",
]

# Try to import MotionPlanner (requires Drake)
try:
    from .motion_planner import MotionPlanner
    __all__.append("MotionPlanner")
except ImportError:
    MotionPlanner = None
    pass
