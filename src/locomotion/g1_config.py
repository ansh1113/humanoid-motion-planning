"""
g1_config.py
Configuration parameters for the Unitree G1 walking controller.
"""
import numpy as np

# Simulation settings
SIM_DT = 0.002
VIEWER_DT = 0.016

JOINT_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint'
]

# --- CONTROL GAINS ---
KP = 500.0   
KD = 40.0    

# --- WALKING PARAMETERS ---
WARMUP_TIME = 1.0       
GAIT_FREQ = 1.2         # Slightly faster to catch the fall
STEP_HEIGHT = 0.05      
STEP_LENGTH = 0.12      

# --- STANCE ---
# Static Lean: -0.4 (Always leaning forward)
HIP_PITCH_OFFSET = -0.4   

# Dynamic Lean: Bow forward by 0.1 rad during the step
HIP_PITCH_AMP = 0.1

KNEE_TARGET = 0.65         
HIP_ROLL_OFFSET = 0.1     
ANKLE_OFFSET = -0.4      # Tuned for the -0.4 hip pitch