# arm_kinematics_trajectory
Kinematics, trajectectory planning and control library, and robot programming (script) language of robotic arms

This repo. contains a 7-segment S-curve profile interpolation algorithm for pos/speed/acceleration/jerk planning.
It also contains continuous smooth trajectory interpolation that creates smooth transitions between multiple
trajectory segments.

![Screenshot from 2025-02-03 07-34-12](https://github.com/user-attachments/assets/b2073038-814c-4f3c-a8f6-55845c33fbd1)


Second, this repo. provides forward/inverse kinematics for scara/quattro/quattroK (keba convention) / quattro_4 (quattro
with 4th rotational DoF).

Third, this repo. provides a way to create robot programming language (scripting) based upon python c++ binding. (Note
this will require installing pybind11 first). All robot motion commands will be installed in rob_commands  python extension.
For example:

    from rob_commands import *

    pf = Profile(2, 50, 1500, 2, 50, 1500)   #cartesian profile (max_vel, max_acc, max_jerk, angular_max_vel, angular_max_acc, angular_max_jerk )
    
    jpf = JntProfile(20, 50, 200)   # joint space profile (jnt_max_vel, jnt_max_acc, jnt_max_jerk)

    para = np.array([[0.3, -math.pi/2.0, 0.4, 0, 1.02, 0.1, 0, 0.18 * math.sqrt(2)]]).T   # kinematic parameters (check code for the definition of these values)

    defaultBaseOff = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T   # where, w.r.t. world frame, install your robot
    
    rob = Robot("quattro", para, defaultBaseOff, defaultBaseOff, pf)   # create Robot object
    
    loc1 = LocData(0.4, 0.4,-0.75, 0,0,0, cfg, turns)     # define a location data
    
    fd = FrameData(1,1, WORLD)     # define a frame data, base no, tool no, interpolation frame
    
    rob.MoveLine(loc1, fd, 10)    # robot performs a straight line motion toward loc1
    
Users can refer to test_script/testQuattro.py for how to use robot commands in python language


Also, this repo. provides many primitive robot motion commands (LINE, PTP, ARC, PTPR, LINER, etc.).
Users can do lots of customization for their application needs.

Finally, this repo. leaves hook for robot program line mapping for supporting  one step motion forward or
one step motion backward.

