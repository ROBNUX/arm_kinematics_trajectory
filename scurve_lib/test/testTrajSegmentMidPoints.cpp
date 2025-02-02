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
    PiecewiseFunction acceleration = traj[2];
    PiecewiseFunction jerk = traj[3];
    
    double t_start, t_end;
    bool ret = position.lowest_t_bound(t_start);
    assert(ret == true);
    
    ret = position.largest_t_bound(t_end);
    assert(ret == true);
    
    //### 1) test if the calculated start/end pos/vel are equal to the given ones
    double p_start_computed = position.evaluate(t_start);
    double v_start_computed = velocity.evaluate(t_start);
    double p_end_computed = position.evaluate(t_end);
    double v_end_computed = velocity.evaluate(t_end);
    strs.str("");
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
    
    
    // #### 2) test if the calculated  pos/vel/acc/jrk [at each time_instatnt 
    // has change in segment_phase] are within the given limits p_max, v_max, a_max, j_max  
    std::vector<double> bounds = position.GetBoundary();
    double p_computed, v_computed, a_computed, j_computed;
    for (size_t i=0; i< bounds.size(); i++) {
        double phase_time = bounds[i];
        p_computed = position.evaluate(phase_time);
        v_computed = velocity.evaluate(phase_time);
        a_computed = acceleration.evaluate(phase_time);
        j_computed = jerk.evaluate(phase_time);
        strs.str("");
        strs << "at time= " << phase_time <<  ", p_computed=" << p_computed
         << ", v_computed=" << v_computed
         << ", a_computed=" << a_computed
         << ", j_computed=" << j_computed << std::endl;
        LOG_INFO(strs);
        assert(fabs(p_computed) <= p_max+1e-6); // #added 1e-6 because sometime it gives fails even if the difference is less than (1e-6) 
        assert(fabs(v_computed) <= v_max+1e-6);        
        assert(fabs(a_computed) <= a_max+1e-6);
        assert(fabs(j_computed) <= j_max+1e-6);
    }
}

int main() {
    std::ostringstream strs;
    // limits
    double p_max = 10.0;
    double v_max = 3.0;
    double a_max = 4.0;
    double j_max = 10.0;
    
    char c = getchar();
    check_fit_traj_segment(0.0, 1.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 3.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 5.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 2.0, 1.0, 1.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 3.0, 2.5, 2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 3.0, 0.5, 0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 10.0, 0.5, 0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 2.0, 0.5, 1.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 5.0, 2.0, 2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 3.0, 0.5, 2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 5.0, 0.5, 2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 2.0, 1.5, 0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 5.0, 2.5, 2.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 3.0, 2.5, 0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 5.0, 2.5, 0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -1.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -3.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -5.0, 0.0, 0.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -2.0, -1.0, -1.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -3.0, -2.5, -2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -3.0, -0.5, -0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -10.0, -0.5, -0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -2.0, -0.5, -1.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -5.0, -2.0, -2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -3.0, -0.5, -2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -5.0, -0.5, -2.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -2.0, -1.5, -0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -5.0, -2.5, -2.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -3.0, -2.5, -0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -5.0, -2.5, -0.5, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 8.0, 1.5, -1.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, 5.0, -1.5, +1.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -8.0, 1.5, -1.0, p_max, v_max, a_max, j_max);
    check_fit_traj_segment(0.0, -5.0, -1.5, +1.0, p_max, v_max, a_max, j_max);
}
