#ifndef KINEMATICS_LIB_COMMON_CONSTANTS_
#define KINEMATICS_LIB_COMMON_CONSTANTS_
#include <string>

namespace kinematics_lib {
static const double EPSILON_CUBIC_ROOTS = 1e-20;
static const double K_EPSILON = 1e-6;  // 0.0001;
// for scurve maximal acc feasibility checking
static const double SCURVE_MAX_ACC_EPSILON = 1e-12;
static const double SCURVE_DURATION_ERR = 1e-3;
static const double SCURVE_MIN_MAX_ACC = 1e-2;
// for determining if higher order term coefficients is close to 0
static const double POLY_EPSILON = 1e-7;
// for finding circles passing though 3 pts
static const double EPSILON_CIRC_3P = 0.00001;            // 0.01mm
static const double EPSILON_MIN_RADIUS = 0.001;           // 1mm
static const double EPSILON_TWOPOINTS_DIAMETER = 0.0001;  // 0.1 mm
// 0.01mm, below which we think translational distance is too small
static const double MIN_TRANS_DIST = 1e-5;  // 0.01mm
// 0.001 radian , below which we think the rotation is too small
static const double MIN_ROT_RADIAN = 1e-4;     // if < 0.001 randian
static const double MIN_TRAJ_DURATION = 1e-3;  // <1ms
// used in binary search to find the time t given the traveled dist
static const double DIST_APPROXI_EPSILON = 1e-5;  // 0.01mm
// minimal approximation dist, under which we think there is no need to
// do approximation, 1e-5 meter is 0.01mm
static const double MIN_APPROX_DIST = 1e-5;  // 0.01 mm
// minimal SCURVE_DIST
static const double MIN_SCURVE_DIST = 1e-5;  // 0.01mm, or 0.00001 degree
// quattro FK condition check
static const double QUATTRO_4_LINKLENGTH_DIFF =
    0.001;  // 0.1mm, due to numerical error, we need this
// traj buffer max size
static const int MAX_TRAJ_BUFF_SIZE = 5;
// commond buffer wait time limit (s)
static const double CMD_BUFF_WAIT_TIME_LIMIT = 0.2;
// Trajectory buffer wait time limit (s)
static const double TRAJ_BUFF_WAIT_TIME_LIMIT = 0.2;
// Init command Buffer size
static const int INIT_CMD_BUFF_SIZE = 2;
// this is the initial angle in Rviz simulation
// it should be changed when the rviz robot description file is changed
/*
 * <joint name='lleg1_base_1' type='continuous'>
    <origin xyz=" 0 0 0" rpy="-0.5350 0 0"/>
     <parent link="con13"/>
     <child link="low_base_1"/>
    <axis xyz="1 0 0"/>
  </joint>
 */
static const double QuattroRvizInitAngle = 0.5350;
// maximal number of base and tool frames
static const unsigned int MAX_BASE_FRAMES = 8;
static const unsigned int MAX_TOOL_FRAMES = 8;

// minimal joint movement in PTP motion, below which we think the joint motion
// is too small
static const double MIN_JNT_DIST = 1e-2;  // 0.01 degree
// maximal matching error (assumed model and actual measurements) calibration
static const double MAX_CALIB_MATCHING_ERR = 1e-12;  // 1e-12; //1e-11; //1e-8;
// constant to determine there exists backlash
static const double MAX_CALIB_BACKLASH_ERR = 1e-8;  // 0.1mm * 0.1mm
static const double MIN_QUAT_SING_VALUE =
    1e-8;  // singular case when quaternion 2 euler has to be individually cal.
static const double MAX_CALIB_MATCHING_ERR_COPLANAR =
    1e-21;  // 1e-16; // 1e-12; //1e-11; //1e-8;
static const double MAX_ORIENT_CALIB_MATCHING_ERR = 1e-7;
static const double MAX_CALIB_STOP_ERR = 1e-9;            // 1e-8;
static const double MAX_CALIB_STOP_ERR_COPLANAR = 1e-20;  // 1e-15;
// maximal iteration used in calibration iterative optimization
static const int MAX_CALIB_ITER = 2000;
static const int MAX_CALIB_INTER_ITER = 2000;
static const int MAX_CALIB_OUTER_ITER = 2000;  // 1000;
// calibration matrix singular
static const double CALIB_SINGULAR_CONST = 1e-16;
// comp matrix singular const
static const double COMP_SINGULAR_CONST = 1e-3;
// calibration line search steps
static const int CALIB_LINE_SEARCH_STEPS = 200;  // 200
// CALIB_OVERALL_SIZE/CALIB_LINE_SEARCH_STEPS gives the actual step size
// p = p0 + CALIB_OVERALL_SIZE/CALIB_LINE_SEARCH_STEPS * delta p
static const double CALIB_OVERALL_SIZE = 1;
// RNN algorithm step size
static const double CALIB_RNN_STEPSIZE = 0.00001 * 0.25;  // 0.025;
// if new_error/previous_error > 0.99, then we can stop
static const double CALIB_CONVERGE_RATIO = 0.99;
// maximal calib step size
static const int MAX_CALIB_STEPS = 2048;
// step size of d, a
static const double DH_LINEAR_EPSILON = 1e-4;  // 0.1 mm
// step size of alpha, theta
static const double DH_ANGULAR_EPSILON = 1e-3;  // 0.001 radian
// max calib single measure error (if a measurement is too off, will not be
// counted)
static const double MAX_CALIB_SINGLE_ERROR = 3e-3;

// verification of CALIB RESULT through error reduction ratio
static const double CALIB_VERIFY_REDUCTION_RATIO =
    0.002;  // 1/5 of original error
// RNN decay variation coef
static const double RNN_DECAY_COEF = 0.1;

static double SIXDOF_HEADDIST = .05;  // it is important to give a right number
static double SIXDOF_WRISTSINGULAR =
    1e-5;  // it is important to give a right number

// maximal D/A diff, and max alpha/theta/beta diff
static const double MAX_A_DIFF_REG = 5e-4;  // 5e-3;  // maximal 0.5 mm
static const double MAX_D_DIFF_REG = 5e-4;  // 5e-3;  // maximal 0.5mm
static const double MAX_ALPHA_DIFF_REG =
    1e-3;  // 0.05236;  // maximal 0.05 degrees
static const double MAX_BETA_DIFF_REG =
    1e-3;  // 0.05236;  // maximal 0.05 degree
static const double MAX_THETA_DIFF_REG =
    1e-3;                               // 0.05236;   // maximal 0.05 degree
static const double MAX_DH_PERC = 0.2;  // maximal 2/1000 percent off

// ridge regression constants for the norm of parameters in weighted
// optimization
static const double ridge_regress_coef = 0.02;
static const double adamp_learning_rate = 0.01;
static const double EPSILON_ADAM_ALG = 1e-20;
static const double SAM_RHO =
    1.0e-3;  // 1 mm, norm of variation of robot model, and joint angle offset
             // are weighted into it

// error code
static const std::string ERR_DESCRIPTION[] = {
    "Function input pointer variable is null",
    "Robot parameters are not initialized",
    "Input parameter has wrong dimension",
    "Tip points of two large arms overlaps, no FK solution",
    "Tip points of three large arms in the same line, no FK solution",
    "two FK circles has no intersections, no FK solution",
    "Quattro forward different kinematics is singular",
    "Command buffer contains wrong command type",
    "Command buffer to trajectory: creating translation command fails",
    "Command buffer to trajectory: creating nonblending traj seg fails",
    "Command buffer to trajectory: generating traj seg got sync error",
    "Robot command ID is out of scope",
    "Robot cartesian pose has no branch info",
    "Robot default IK has to use canonical parameters",
    "Rotation to Euler angles got singularity",
    "Robot Jacobian inverse got singularity",
    "Robot calibration measure data got wrong dimension",
    "Robot calibration gets singular Jacobian",
    "Robot calibration regression gets wrong dimension",
    "Robot calibration regression over maximal iteration",
    "Trajectory compensation without calibration first",
    "Trajectory compensation first step (homotopty alg) fails",
    "Trajectory compensation algorithm diverges",
    "Laser based calibration  with too less samples",
    "Calibration is not initialized",
    "Calibration using RNN algorithm got overfitting",
    "Other calibration error",
    "Robot IK error due to unreachable position",
};
static const int ERR_INPUT_POINTER_NULL = 1;
static const int ERR_ROB_PARAM_NOT_INITIALIZED = 2;
static const int ERR_INPUT_PARA_WRONG_DIM = 3;
static const int ERR_QUATTRO_FK_OVERLAP = 4;
static const int ERR_QUATTRO_FK_3PT_LINE = 5;
static const int ERR_QUATTRO_FK_CIRC_NO_INTERS = 6;
static const int ERR_QUATTRO_DIFF_FK_SINGULAR = 7;
static const int ERR_TRAJ_COMMAND_WRONG_TYPE = 28;
static const int ERR_TRAJ_COMMAND_GENERATE_TRANSLATION_FAILS = 29;
static const int ERR_TRAJ_COMMAND_GENERATE_ENTIRE_TRAJ_SEG = 30;
static const int ERR_TRAJ_GENERATE_SYNC_ERROR = 31;
static const int ERR_TRAJ_COMMAND_OUT_OF_SCOPE = 32;
static const int ERR_TRAJ_SET_BOUND = 33;

static const int ERR_ROB_NO_BRANCH_INFO = 8;
static const int ERR_ROB_DEFAULT_IK_NOT_CANO = 9;
static const int ERR_ROB_IK_OUTOF_REACH = 10;
static const int ERR_ROT2EULER_SINGULAR = 11;
static const int ERR_ROB_JACOBIAN_IK_SINGULAR = 12;
static const int ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM = 13;
static const int ERR_CALIB_JAC_SINGULAR = 14;
static const int ERR_CALIB_REG_WRONG_DIM = 15;
static const int ERR_CALIB_REG_MAX_ITER = 16;
static const int ERR_COMP_WITHOUT_CALIB = 17;
static const int ERR_COMP_HOMOTOPY_FAIL = 18;
static const int ERR_COMP_ALG_DIVERGE = 19;
static const int ERR_CALIB_LASER_LESS_SAMPLES = 20;

// calibration not init error
static const int ERR_CALIB_NOT_INIT = 21;
static const int ERR_CALIB_RNN_OVERFIT = 22;
static const int ERR_CALIB_OTHER_ERR = 23;

// six axis robot IK unreachable error
static const int ERR_ID_IKPOS_UNREACHABLE = 24;
// six axis IK infinite wrist solution error
static const int ERR_ID_IKPOS_INFINITE_WRIST_SOLUTION = 25;
// six axis IK overhead infinite solution error
static const int ERR_ID_IKPOS_INFINITE_SOLUTION = 26;

inline double sqr(const double arg) { return arg * arg; }
}  // namespace kinematics_lib
#endif  // KINEMATICS_LIB_COMMON_CONSTANTS_
