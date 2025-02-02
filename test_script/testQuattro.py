import math
import rospy
import time
import numpy as np
import sys
# import our robot library
from rob_commands import *

# before crate a robot, we need to (1) pass the profile 
# (how faster we want robot to run), (2) pass kinemamtic para.
# (3) pass the name of the robot, here we support the following
# robot: 
# "scara"  -- scara robto
# "quattro" -- quattro robot
# "quattroK" -- quattro robot keba convention
#"quattro_4" -- quattro robot with rotational dof
# "6axis_wrist" -- 6axis robot with last three axis passing through
# same point (not done yet)
# "6axis_ur" -- 6axis robot of Uiversal Robot type (not done yet)
# note: these python codes are for testing only, in the future, robot
# nodes should be launched from yaml configure files, and user should
# not take care of profile and kinematic parameters


DG2RAD = 3.1415926 / 180.0

# cartesian profile (max_vel, max_acc, max_jerk, angular_max_vel, angular_max_acc, angular_max_jerk )
pf = Profile(2, 50, 1500, 2, 50, 1500)
# joint space profile (jnt_max_vel, jnt_max_acc, jnt_max_jerk)
jpf = JntProfile(20, 50, 200)

# kinematic parameters (check code for the definition of these values)
para = np.array([[0.3, -math.pi/2.0, 0.4, 0, 1.02, 0.1, 0, 0.18 * math.sqrt(2)]]).T

# where, w.r.t. world frame, install your robot
defaultBaseOff = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
# create robot object (basically to start motion thread)
rob = Robot("quattro", para, defaultBaseOff, defaultBaseOff, pf)

value=input("please enter a number\n")
# because we are using rviz simulation, whether there is no feedback
# we have to manually set feedback joint angles, otherwise, robot
# thinks it never receives feedback, and will think it is not safe to move
current_jnts = np.array([[0.0,0.0,0.0,0.0]]).T
rob.SetFeedback(current_jnts)

# get robot initial cartesian loc
cur_loc = LocData()
rob.ForwardKin(current_jnts, cur_loc)

# obtain the configure and turns of initial loc
cfg = cur_loc.G
turns = cur_loc.T

# set joint profile for robot
for i in range(4):
    rob.SetJntProfile(i, jpf)    


# start motion thread (simulate servo-on button)
rob.StartMotion()

# speed/acc/jerk scale set as 80% w.r.t. their maximal values
perc = Percent(80, 80, 80)
rob.SetSpeed(perc)

# create a Location (x,y,z, A, B, C, cfg, turns) 
loc1 = LocData(0.4, 0.4,-0.75, 0,0,0, cfg, turns)
loc2 = LocData(-0.4, 0.4,-0.75, 0,0,0, cfg, turns)
#loc3 = LocData(-0.4, -0.4, -0.75, 0,0,0, cfg, turns)
#loc4 = LocData(0.4, -0.4, -0.75,0,0,0, cfg, turns)
loc5 = LocData(-0.4, -0.4, -0.75, 0, 0, 0, cfg, turns)
loc6 = LocData(0.4, -0.4, -0.75, 0, 0, 0, cfg, turns)

# create a frameData object: baseNo 1 (default base, toolNo 1 (default tool)
# the last parameter is enum IpoMode, only used in MoveLineRel and MovePTPRel commands
fd = FrameData(1,1, WORLD)


# main program for running robot
try:
    for i in range(10):
        rob.MoveLine(loc1, fd, 10)
        rob.MoveLine(loc2, fd, 10)
        #rob.MoveLine(loc3, fd, 10)
        #rob.MoveLine(loc4, fd, 0)
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

