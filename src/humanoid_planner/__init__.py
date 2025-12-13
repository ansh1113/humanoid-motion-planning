"""
Humanoid Motion Planning Package

Core modules (always available):
- ZMPConstraint: Zero Moment Point constraint handling
- SupportPolygonCalculator: Support polygon computation
- TrajectoryOptimizer: Trajectory optimization
- ForwardKinematics, InverseKinematics: Robot kinematics
- StabilityAnalyzer: Stability analysis and CoM tracking
- Perception: Object detection and workspace analysis

Optional modules (require Drake):
- MotionPlanner: Full motion planning with Drake simulation
- DrakeTrajectoryOptimizer: Advanced trajectory optimization with Drake
"""

__version__ = "0.1.0"
__author__ = "Ansh Bhansali"

from .zmp_constraint import ZMPConstraint
from .support_polygon import SupportPolygonCalculator
from .trajectory_optimizer import TrajectoryOptimizer
from .kinematics import ForwardKinematics, InverseKinematics
from .stability_analysis import StabilityAnalyzer, CenterOfMassTracker
from .perception import (
    ObjectDetector, EnvironmentState, WorkspaceAnalyzer, 
    GoalPredictor, DetectedObject, ObjectType
)

__all__ = [
    "ZMPConstraint",
    "SupportPolygonCalculator",
    "TrajectoryOptimizer",
    "ForwardKinematics",
    "InverseKinematics",
    "StabilityAnalyzer",
    "CenterOfMassTracker",
    "ObjectDetector",
    "EnvironmentState",
    "WorkspaceAnalyzer",
    "GoalPredictor",
    "DetectedObject",
    "ObjectType",
]

# Try to import MotionPlanner (requires Drake)
try:
    from .motion_planner import MotionPlanner, DrakeTrajectoryOptimizer
    __all__.extend(["MotionPlanner", "DrakeTrajectoryOptimizer"])
except ImportError:
    MotionPlanner = None
    DrakeTrajectoryOptimizer = None
    pass
