import math
import time
import sys
import numpy as np

import rob_motion_commands as m

# before creating a robot we need to (1) pass the profile
# (how fast we want the robot to run), (2) pass kinematic parameters,
# (3) pass the name of the robot. Supported robots:
#   "scara"       -- SCARA robot
#   "quattro"     -- Quattro robot
#   "quattroK"    -- Quattro robot (Keba convention)
#   "quattro_4"   -- Quattro robot with rotational DOF
#   "6axis_wrist" -- 6-axis robot, wrist axes through one point (WIP)
#   "6axis_ur"    -- 6-axis Universal Robot type (WIP)
# Note: these Python scripts are for testing only. In production, robot nodes
# should be launched from YAML config files.

DG2RAD = math.pi / 180.0

# Cartesian profile (max_vel, max_acc, max_jerk, angular_max_vel, angular_max_acc, angular_max_jerk)
pf = m.Profile(2, 50, 1500, 2, 50, 1500)
# Joint-space profile (jnt_max_vel, jnt_max_acc, jnt_max_jerk)
jpf = m.JntProfile(20, 50, 200)

# Kinematic parameters (see quattro.hpp for definitions)
para = np.array([[0.3, -math.pi / 2.0, 0.4, 0, 1.02, 0.1, 0, 0.18 * math.sqrt(2)]]).T

# Base offset w.r.t. world frame: (tx, ty, tz, qw, qx, qy, qz)
defaultBaseOff = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

# Create robot object (starts internal motion threads)
rob = m.Robot("quattro", para, defaultBaseOff, defaultBaseOff, pf)

input("please enter a number\n")

# Because we are using rviz simulation without real feedback we set joint
# angles manually; otherwise the robot thinks it never received feedback and
# will refuse to move.
current_jnts = np.array([[0.0, 0.0, 0.0, 0.0]]).T
rob.SetFeedback(current_jnts)

# Get robot initial Cartesian location and extract branch/turn flags
ok, cur_loc = rob.ForwardKin(current_jnts)
cfg   = cur_loc.G
turns = cur_loc.T

# Set joint profile for all 4 axes
for i in range(4):
    rob.SetJntProfile(i, jpf)

# Start motion thread (simulates servo-on)
rob.StartMotion()

# Speed/acc/jerk scale set to 80 % of maximum values
perc = m.Percent(80, 80, 80)
rob.SetSpeed(perc)

# Define target locations (x, y, z, A, B, C, branch, turn)
loc1 = m.LocData( 0.4,  0.4, -0.75, 0, 0, 0, cfg, turns)
loc2 = m.LocData(-0.4,  0.4, -0.75, 0, 0, 0, cfg, turns)
loc5 = m.LocData(-0.4, -0.4, -0.75, 0, 0, 0, cfg, turns)
loc6 = m.LocData( 0.4, -0.4, -0.75, 0, 0, 0, cfg, turns)

# FrameData: base 1 (default base), tool 1 (default tool), world-frame IPO
fd = m.FrameData(1, 1, m.IpoMode.WORLD)

# Main motion loop
try:
    for i in range(10):
        rob.MoveLine(loc1, fd, 10)
        rob.MoveLine(loc2, fd, 10)
        rob.MovePTP(loc5, fd, 10)
        rob.MovePTP(loc6, fd, 10)
    while not rob.MotionDone():
        time.sleep(1)
except RuntimeError as e:
    rob.Shutdown()
    if "SIGNAL" in str(e):
        code = int(str(e).rsplit(" ", 1)[1])
        sys.exit(128 + code)
    raise
