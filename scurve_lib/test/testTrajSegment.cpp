#include "scurve_lib/piecewise_function.hpp"
#include "scurve_lib/segment_planning.hpp"
#include <cmath>
#include <cassert>
#include <sstream>

using namespace kinematics_lib;
using namespace ROBNUXLogging;

void check_fit_traj_segment(double p_start, double p_end, double v_start,
        double v_end, double p_max, double v_max,
        double a_max, double j_max) {
    std::ostringstream strs;
    std::vector<PiecewiseFunction> traj = SegmentPlanning::fit_traj_segment(
            p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max);
    PiecewiseFunction position = traj[0];
    PiecewiseFunction velocity = traj[1];
    
    double t_start, t_end;
    bool ret = position.lowest_t_bound(t_start);
    assert(ret == true);
    
    ret = position.largest_t_bound(t_end);
    assert(ret == true);
    
    double p_start_computed = position.evaluate(t_start);
    double v_start_computed = velocity.evaluate(t_start);
    double p_end_computed = position.evaluate(t_end);
    double v_end_computed = velocity.evaluate(t_end);
    
    strs << "p_start_computed=" << p_start_computed
         << "\n p_end_computed=" << p_end_computed
         << "\n v_start_computed=" << v_start_computed
         << "\n v_end_computed=" << v_end_computed << std::endl;
    strs << "\n p_start=" << p_start
         << "\n p_end=" << p_end
         << "\n v_start=" << v_start
         << "\n v_end=" << v_end << std::endl;
    LOG_INFO(strs);
    
    assert(fabs(p_start - p_start_computed) < 1e-5);
    assert(fabs(p_end - p_end_computed) < 1e-5); 
    assert(fabs(v_start - v_start_computed) < 1e-5);
    assert(fabs(v_end - v_end_computed) < 1e-5);
}

int main() {
    std::ostringstream strs;
    // limits
    double p_max = 30.0;
    double v_max = 3.0;
    double a_max = 4.0;
    double j_max = 10.0;
    
    char c = getchar();
    
    // ############### CASE A: v_start = v_end = 0    [normal case, start and end velocities are zeros]
    // #case A1:  no limit is reached
    check_fit_traj_segment(0.0, 1.0, 0.0, 0.0,     p_max, v_max, a_max, j_max);
    // case A2:  acc_limit is reached 
    check_fit_traj_segment(0.0, 3.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    // case A3:  vel_limit and acc_limit are reached
    check_fit_traj_segment(0.0, 5.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    
    // ############# CASE B: v_start = v_end !=0   [ start and end velocities are equal but not null]
    // case B1:  no limit is reached
    check_fit_traj_segment(0.0, 2.0, 1.0, 1.0, p_max, v_max, a_max, j_max);
    // case B2:  vel_limit is reached
    check_fit_traj_segment(0.0, 3.0, 2.5, 2.5, p_max, v_max, a_max, j_max);
    // case B3:  acc_limit is reached
    check_fit_traj_segment(0.0, 3.0, 0.5, 0.5, p_max, v_max, a_max, j_max);
    // case B4:  vel_limit and acc_limit are reached
    check_fit_traj_segment(0.0, 10.0, 0.5, 0.5, p_max, v_max, a_max, j_max);
    
    // ############## CASE C:  v_start < v_end [acceleration]
    //case C1:  no limit is reached
    check_fit_traj_segment(0.0, 2.0, 0.5, 1.5, p_max, v_max, a_max, j_max);
    //case C2:  vel_limit is reached
    check_fit_traj_segment(0.0, 5.0, 2.0, 2.5, p_max, v_max, a_max, j_max);
    //case C3:  acc_limit is reached
    check_fit_traj_segment(0.0, 3.0, 0.5, 2.5, p_max, v_max, a_max, j_max);
    //case C4:  vel_limit and acc_limit are reached
    check_fit_traj_segment(0.0, 5.0, 0.5, 2.5, p_max, v_max, a_max, j_max);

    // ############## CASE D:  v_start > v_end [deceleration]
    // case D1:  no limit is reached
    check_fit_traj_segment(0.0, 2.0, 1.5, 0.5, p_max, v_max, a_max, j_max);
    // case D2:  vel_limit is reached
    check_fit_traj_segment(0.0, 5.0, 2.5, 2.0, p_max, v_max, a_max, j_max);
    // case D3:  acc_limit is reached
    check_fit_traj_segment(0.0, 3.0, 2.5, 0.5, p_max, v_max, a_max, j_max);
    // case D4:  vel_limit and acc_limit are reached
    check_fit_traj_segment(0.0, 5.0, 2.5, 0.5, p_max, v_max, a_max, j_max);
}   

/*

############### CASE A-: v_start = v_end = 0    [normal case, negative motion]
#case A1-:  no limit is reached     
def test_not_max_vel_nor_max_acc_zero_equal_velocity_negative_motion():
    check_fit_traj_segment(0.0, -1.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)      
    
#case A2-:  acc_limit is reached     
def test_max_acc_but_not_max_vel_zero_equal_velocity_negative_motion():
    check_fit_traj_segment(0.0, -3.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)      
    
#case A3-:  vel_limit and acc_limit are reached    
def test_max_vel_and_max_acc_zero_equal_velocity_negative_motion():
    check_fit_traj_segment(0.0, -5.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)        
    

############## CASE B-: v_start = v_end !=0   [ start and end velocities are equal but not null]
#case B1-:  no limit is reached
def test_not_max_vel_nor_max_acc_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -2.0,      -1.0, -1.0,     p_max, v_max, a_max, j_max)
       
#case B2-:  vel_limit is reached 
def test_max_vel_but_not_max_acc_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -3.0,     -2.5, -2.5,     p_max, v_max, a_max, j_max)   
   
#case B3-:  acc_limit is reached
def test_max_acc_but_not_max_vel_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -3.0,      -0.5, -0.5,     p_max, v_max, a_max, j_max)           
   
#case B4-:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -10.0,     -0.5, -0.5,     p_max, v_max, a_max, j_max)    
   

############## CASE C-:  v_start < v_end [acceleration]
#case C1-:  no limit is reached
def test_not_max_vel_nor_max_acc_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -2.0,       -0.5, -1.5,    p_max, v_max, a_max, j_max)    
    
##case C2-:  vel_limit is reached 
def test_max_vel_but_not_max_acc_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,      -2.0, -2.5,     p_max, v_max, a_max, j_max)    
    
##case C3-:  acc_limit is reached
def test_max_acc_but_not_max_vel_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -3.0,      -0.5, -2.5,     p_max, v_max, a_max, j_max) 
    
##case C4-:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,       -0.5, -2.5,    p_max, v_max, a_max, j_max)    
   

############## CASE D-:  v_start > v_end [deceleration]
#case D1:  no limit is reached
def test_not_max_vel_nor_max_acc_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -2.0,       -1.5, -0.5,    p_max, v_max, a_max, j_max)     
    
##case D2:  vel_limit is reached 
def test_max_vel_but_not_max_acc_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,       -2.5, -2.0,    p_max, v_max, a_max, j_max)        
    
##case D3:  acc_limit is reached
def test_max_acc_but_not_max_vel_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -3.0,       -2.5, -0.5,    p_max, v_max, a_max, j_max)
    
##case D4:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,       -2.5, -0.5,    p_max, v_max, a_max, j_max)        
   



##############################################################################################
############### complex motion: v_start*v_end <0, +ve and -ve parts ##########################
##############################################################################################


############ CASE X: positive dominant motion: pos_diff > 0
### starting from +ve to -ve
def test_X1_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, 10.0,      1.5, -1.0,     p_max, v_max, a_max, j_max)   
### starting from -ve to +ve
def test_X2_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, 5.0,      -1.5,  1.0,     p_max, v_max, a_max, j_max)   


#############CASE Y: negative dominant motion: pos_diff < 0
### starting from +ve to -ve
def test_Y1_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, -10.0,      1.5, -1.0,     p_max, v_max, a_max, j_max)   
### starting from -ve to +ve
def test_Y2_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, -5.0,      -1.5,  1.0,     p_max, v_max, a_max, j_max)   

*/