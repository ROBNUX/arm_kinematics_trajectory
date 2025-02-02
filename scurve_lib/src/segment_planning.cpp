#include "scurve_lib/segment_planning.hpp"

namespace kinematics_lib {
std::vector<double>
SegmentPlanning::calculate_min_pos_reached_acc_to_reach_max_vel(
    const double v, const double vm, const double am, const double jm) {
  double p0 = 0.0;
  double a0 = 0.0;
  double v0 = v;
  double ar = sqrt(jm * (vm - v0));
  double t = ar / jm;
  double ta = 0.0;
  double tv = 0.0;

  double a1 = jm * t + a0;
  double a2 = a1;
  double a3 = -jm * t + a2;
  double a4 = a3;
  double a5 = -jm * t + a4;
  double a6 = a5;

  double v1 = jm * t * t / 2.0 + a0 * t + v0;
  double v2 = a1 * ta + v1;
  double v3 = -jm * t * t / 2.0 + a2 * t + v2;
  double v4 = a3 * tv + v3;
  double v5 = -jm * t * t / 2.0 + a4 * t + v4;
  double v6 = a5 * ta + v5;

  double p1 = jm * t * t * t / 6.0 + a0 * t * t / 2.0 + v0 * t + p0;
  double p2 = a1 * ta * ta / 2.0 + v1 * ta + p1;
  double p3 = -jm * t * t * t / 6.0 + a2 * t * t / 2.0 + v2 * t + p2;
  double p4 = a3 * t * t / 2.0 + v3 * tv + p3;
  double p5 = -jm * t * t * t / 6.0 + a4 * t * t / 2.0 + v4 * t + p4;
  double p6 = a5 * ta * ta / 2.0 + v5 * ta + p5;
  double p7 = jm * t * t * t / 6.0 + a6 * t * t / 2.0 + v6 * t + p6;
  std::vector<double> out_data;
  out_data.push_back(p7);
  out_data.push_back(ar);
  return out_data;
}

double SegmentPlanning::calculate_reached_acc_in_case_no_const_acc_phase(
    const double Dp, const double v, const double vm, const double am,
    const double jm) {
  std::ostringstream strs;
  double p0 = 0.0;
  double v0 = v;
  double a = 2.0;
  double b = 0.0;
  double c = 4 * v0 * jm;
  double d = (p0 - Dp) * jm * jm;
  std::vector<double> roots;
  if (!Compute_Roots_Utility::CubicEqRealRoots(a, b, c, d, roots)) {
    strs << __FUNCTION__ << ":" << __LINE__
         << ", call CubicEqRealRoots fails, a =" << a << ", b=" << b
         << ", c=" << c << ", d=" << d << std::endl;
    LOG_ERROR(strs);
  }
  size_t n_rts = roots.size();
  if (n_rts == 1) {
    return roots[0];
  } else if (n_rts == 2) {
    return Compute_Roots_Utility::min_positive_root2(roots[0], roots[1]);
  } else if (n_rts == 3) {
    return Compute_Roots_Utility::min_positive_root3(roots[0], roots[1],
                                                     roots[2]);
  }
  return -std::numeric_limits<double>::max();  // return most negative limits
}

std::vector<double>
SegmentPlanning::calculate_min_pos_const_acc_time_to_reach_max_acc_and_max_vel(
    const double v, const double vm, const double am, const double jm) {
  double p0 = 0.0;
  double a0 = 0.0;
  double v0 = v;
  double t = am / jm;
  double tv = 0.0;
  double ta = (vm - v0 - (am * am / jm)) / am;

  double a1 = jm * t + a0;
  double a2 = a1;
  double a3 = -jm * t + a2;
  double a4 = a3;
  double a5 = -jm * t + a4;
  double a6 = a5;

  double v1 = jm * t * t / 2.0 + a0 * t + v0;
  double v2 = a1 * ta + v1;
  double v3 = -jm * t * t / 2.0 + a2 * t + v2;
  double v4 = a3 * tv + v3;
  double v5 = -jm * t * t / 2.0 + a4 * t + v4;
  double v6 = a5 * ta + v5;

  double p1 = jm * t * t * t / 6.0 + a0 * t * t / 2.0 + v0 * t + p0;
  double p2 = a1 * ta * ta / 2.0 + v1 * ta + p1;
  double p3 = -jm * t * t * t / 6.0 + a2 * t * t / 2.0 + v2 * t + p2;
  double p4 = a3 * t * t / 2.0 + v3 * tv + p3;
  double p5 = -jm * t * t * t / 6.0 + a4 * t * t / 2.0 + v4 * t + p4;
  double p6 = a5 * ta * ta / 2.0 + v5 * ta + p5;
  double p7 = jm * t * t * t / 6.0 + a6 * t * t / 2.0 + v6 * t + p6;
  std::vector<double> out_data;
  out_data.push_back(p7 - p0);
  out_data.push_back(ta);
  return out_data;
}

double SegmentPlanning::calculate_min_pos_to_reach_max_acc(const double v,
                                                           const double vm,
                                                           const double am,
                                                           const double jm) {
  double p0 = 0.0;
  double a0 = 0.0;
  double v0 = v;
  double t = am / jm;
  double tv = 0.0;
  double ta = 0.0;

  double a1 = jm * t + a0;
  double a2 = a1;
  double a3 = -jm * t + a2;
  double a4 = a3;
  double a5 = -jm * t + a4;
  double a6 = a5;

  double v1 = jm * t * t / 2.0 + a0 * t + v0;
  double v2 = a1 * ta + v1;
  double v3 = -jm * t * t / 2.0 + a2 * t + v2;
  double v4 = a3 * tv + v3;
  double v5 = -jm * t * t / 2.0 + a4 * t + v4;
  double v6 = a5 * ta + v5;

  double p1 = jm * t * t * t / 6.0 + a0 * t * t / 2.0 + v0 * t + p0;
  double p2 = a1 * ta * ta / 2.0 + v1 * ta + p1;
  double p3 = -jm * t * t * t / 6.0 + a2 * t * t / 2.0 + v2 * t + p2;
  double p4 = a3 * t * t / 2.0 + v3 * tv + p3;
  double p5 = -jm * t * t * t / 6.0 + a4 * t * t / 2.0 + v4 * t + p4;
  double p6 = a5 * ta * ta / 2.0 + v5 * ta + p5;
  double p7 = jm * t * t * t / 6.0 + a6 * t * t / 2.0 + v6 * t + p6;
  return p7 - p0;
}

double SegmentPlanning::calculate_const_acc_time(const double Dp,
                                                 const double v,
                                                 const double vm,
                                                 const double am,
                                                 const double jm) {
  std::ostringstream strs;
  // here we wont reach max vel but still we reach max acc
  double p0 = 0.0;
  double v0 = v;
  // P7 equation is: (2*am^3 + 3*am^2*jm*ta + am*jm^2*ta^2 + 4*v0*am*jm +
  // 2*v0*jm^2*ta + p0*jm^2)/jm^2
  double a = 0.0;
  double jm2 = jm * jm;
  double am2 = am * am;
  double am3 = am2 * am;
  double b = am * jm2;
  double c = 3 * am2 * jm + 2 * v0 * jm2;
  double d = 2 * am3 + 4 * v0 * am * jm + p0 * jm2 - jm2 * Dp;

  std::vector<double> roots;

  if (!Compute_Roots_Utility::CubicEqRealRoots(a, b, c, d, roots)) {
    strs << __FUNCTION__ << ":" << __LINE__
         << ", call CubicEqRealRoots fails, a =" << a << ", b=" << b
         << ", c=" << c << ", d=" << d << std::endl;
    LOG_ERROR(strs);
  }
  size_t n_rts = roots.size();
  if (n_rts == 1) {
    return roots[0];
  } else if (n_rts == 2) {
    return Compute_Roots_Utility::min_positive_root2(roots[0], roots[1]);
  } else if (n_rts == 3) {
    return Compute_Roots_Utility::min_positive_root3(roots[0], roots[1],
                                                     roots[2]);
  }
  return -std::numeric_limits<double>::max();  // return most negative limits
}

std::vector<double> SegmentPlanning::
    calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(
        const double v0, const double vf, const double vm, const double am,
        const double jm_orig) {
  double p0 = 0.0;
  double a0 = 0.0;
  double diff_v = fabs(vf - v0);
  double ar = sqrt(jm_orig * diff_v);
  double tj = ar / jm_orig;
  double ta = 0.0;
  double t = tj;
  std::vector<double> out_values(4, 0);
  double jm;
  if (diff_v < std::numeric_limits<double>::epsilon()) {
    return out_values;
  } else {
    jm = vf > v0 ? jm_orig : -jm_orig;  // math.copysign(jm,  (vf-v0) )
  }

  double a1 = jm * t + a0;
  double a2 = a1;
  double v1 = jm * t * t / 2.0 + a0 * t + v0;
  double v2 = a1 * ta + v1;
  double p1 = jm * t * t * t / 6.0 + a0 * t * t / 2.0 + v0 * t + p0;
  double p2 = a1 * ta * ta / 2.0 + v1 * ta + p1;
  double p3 = -jm * t * t * t / 6.0 + a2 * t * t / 2.0 + v2 * t + p2;

  double acc_to_vf, min_pos_to_vf;
  if (fabs(a2) <= fabs(am)) {
    acc_to_vf = a2;
    min_pos_to_vf = p3;
  } else {
    tj = fabs(am / jm);
    ta = (fabs(vf - v0) - fabs((am * am / jm))) / am;
    t = tj;
    a1 = jm * t + a0;
    a2 = a1;
    v1 = jm * t * t / 2.0 + a0 * t + v0;
    v2 = a1 * ta + v1;
    p1 = jm * t * t * t / 6.0 + a0 * t * t / 2.0 + v0 * t + p0;
    p2 = a1 * ta * ta / 2.0 + v1 * ta + p1;
    p3 = -jm * t * t * t / 6.0 + a2 * t * t / 2.0 + v2 * t + p2;
    acc_to_vf = a2;
    min_pos_to_vf = p3;
  }
  out_values[0] = min_pos_to_vf;
  out_values[1] = acc_to_vf;
  out_values[2] = tj;
  out_values[3] = ta;
  return out_values;
}

std::vector<double> SegmentPlanning::equal_vel_case_planning(
    const double pos_diff, const double v, const double abs_max_vel,
    const double abs_max_acc, const double abs_max_jrk) {
  std::ostringstream strs;
  double reached_vel, reached_acc, t_max_jrk, t_max_acc, t_max_vel;

  // check if a const_acc phase is required or not by calcuating the
  // reached_acc_to_max_vel to reach vf
  double min_pos_to_max_vel, min_pos_max_acc, reached_acc_to_max_vel;
  std::vector<double> outValues =
      calculate_min_pos_reached_acc_to_reach_max_vel(v, abs_max_vel,
                                                     abs_max_acc, abs_max_jrk);

  min_pos_to_max_vel = outValues[0];
  reached_acc_to_max_vel = outValues[1];

  // check if reached_acc_to_max_vel < abs_max_acc:
  // if yes: then no const_acc phase, check if a const_vel phase is required or
  // not (to satisfy pos_diff)
  if (reached_acc_to_max_vel <= abs_max_acc) {
    strs.str("");
    strs << __FUNCTION__ << ": " << __LINE__
         << "case a: maxAcc won't be reached !  /\\/ " << std::endl;
    LOG_DEBUG(strs);
    reached_vel = abs_max_vel;
    reached_acc = reached_acc_to_max_vel;
    t_max_jrk = reached_acc_to_max_vel / abs_max_jrk;
    t_max_acc = 0.0;
    t_max_vel = 0.0;

    if (pos_diff > min_pos_to_max_vel) {
      strs.str("");
      strs << __FUNCTION__ << ": " << __LINE__
           << "\n >>> case a1: require const_vel_phase=zero_acc_phase [ "
              "/\\-----\\/ ]"
           << std::endl;
      LOG_DEBUG(strs);
      t_max_vel = (pos_diff - min_pos_to_max_vel) / abs_max_vel;
    } else {
      strs.str("");
      strs << __FUNCTION__ << ": " << __LINE__
           << "\n >>> case a2: calculate Acc corresponds to pos_diff [ /\\/ ]"
           << std::endl;
      LOG_DEBUG(strs);
      double acc = calculate_reached_acc_in_case_no_const_acc_phase(
          pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk);
      t_max_jrk = acc / abs_max_jrk;
    }
    // if  no: check if a const_acc phase is required or not (to satisfy
    // pos_diff)
  } else if (reached_acc_to_max_vel > abs_max_acc) {
    strs.str("");
    strs << __FUNCTION__ << ": " << __LINE__
         << "\n case b: maxAcc will be reached !  /'''\\.../" << std::endl;
    LOG_DEBUG(strs);
    outValues = calculate_min_pos_const_acc_time_to_reach_max_acc_and_max_vel(
        v, abs_max_vel, abs_max_acc, abs_max_jrk);
    min_pos_to_max_vel = outValues[0];
    t_max_acc = outValues[1];
    reached_vel = abs_max_vel;
    reached_acc = abs_max_acc;
    t_max_jrk = abs_max_acc / abs_max_jrk;
    t_max_vel = 0.0;

    if (pos_diff >= min_pos_to_max_vel) {
      strs.str("");
      strs << __FUNCTION__ << ": " << __LINE__
           << "\n >>> case b1: require const_vel_phase=zero_acc_phase [ "
              "/```\\------\\.../ ]"
           << std::endl;
      LOG_DEBUG(strs);
      t_max_vel = (pos_diff - min_pos_to_max_vel) / abs_max_vel;
    } else {
      double min_pos_to_max_acc = calculate_min_pos_to_reach_max_acc(
          v, abs_max_vel, abs_max_acc, abs_max_jrk);
      strs.str("");
      strs << __FUNCTION__ << ": " << __LINE__
           << " \n case b2: min_pos_to_max_acc=" << min_pos_to_max_acc
           << ", dP=" << pos_diff << std::endl;
      LOG_DEBUG(strs);
      if (pos_diff >= min_pos_to_max_acc) {
        strs.str("");
        strs << __FUNCTION__ << ": " << __LINE__
             << "\n >>> case b2a: calculate acc_time-reached_vel corresponds "
                "to pos_diff [ /````\\..../ ]"
             << std::endl;
        LOG_DEBUG(strs);
        t_max_acc = calculate_const_acc_time(pos_diff, v, abs_max_vel,
                                             abs_max_acc, abs_max_jrk);
      } else {
        strs.str("");
        strs
            << __FUNCTION__ << ": " << __LINE__
            << "\n >>> case b2b: calculate acc corresponds to pos_diff [ /\\/ ]"
            << std::endl;
        double acc = calculate_reached_acc_in_case_no_const_acc_phase(
            pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk);
        t_max_jrk = acc / abs_max_jrk;
        t_max_acc = 0.0;
        t_max_vel = 0.0;
      }
    }
  }
  std::vector<double> outValues1;
  outValues1.push_back(t_max_jrk);
  outValues1.push_back(t_max_acc);
  outValues1.push_back(t_max_vel);
  outValues1.push_back(reached_vel);
  outValues1.push_back(reached_acc);
  return outValues1;
}

std::vector<double> SegmentPlanning::traj_segment_planning(
    const double p_start, const double p_end, const double abs_v_start,
    const double abs_v_end, const double abs_max_vel, const double abs_max_acc,
    const double abs_max_jrk) {
  std::ostringstream strs;
  std::vector<double> outValues1(5, 0);
  // calculate min_pos required to reach vf from v0
  std::vector<double> outValues =
      calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(
          abs_v_start, abs_v_end, abs_max_vel, abs_max_acc, abs_max_jrk);
  double abs_min_pos_to_vf = outValues[0];
  double acc_to_vf = outValues[1];
  double t_jrk_to_vf = outValues[2];
  double t_acc_to_vf = outValues[3];

  double t_jrk, t_acc, t_vel;
  if (abs_min_pos_to_vf >
      fabs(p_end - p_start)) {  // # if abs_min_pos_to_vf> abs(p_end-p_start),
                                // then these values are not feasible
    strs.str("");
    strs << __FUNCTION__ << ": " << __LINE__
         << ">>> min required position difference to reach v_end from v_start="
         << abs_min_pos_to_vf
         << " > abs(p_end-p_start)= " << fabs(p_end - p_start) << std::endl;
    LOG_ERROR(strs);
    return outValues1;
  } else {  // then the motion will be:
    double abs_v;
    // from v0 to reach vf with dis=minPos_to_vf, then then plan the rest of the
    // required distance (pos_diff - minPos_to_vf) using vf (as vf>v0)
    if (abs_v_end > abs_v_start) {
      abs_v = abs_v_end;
    } else {  // plan the rest of the required distance (pos_diff -
              // minPos_to_vf) using v0 (as v0>vf), from v0 to reach vf with
              // dis= minPos_to_vf
      abs_v = abs_v_start;
    }

    if (fabs(p_end - p_start) > abs_min_pos_to_vf) {
      double sgn_abs_min_pos_to_vf =
          p_end > p_start ? abs_min_pos_to_vf : -abs_min_pos_to_vf;
      double pos_diff = p_end - p_start - sgn_abs_min_pos_to_vf;
      // plan the rest of the motion using the equal start/end vel case
      std::vector<double> tmpValues = equal_vel_case_planning(
          fabs(pos_diff), abs_v, abs_max_vel, abs_max_acc, abs_max_jrk);
      t_jrk = tmpValues[0];
      t_acc = tmpValues[1];
      t_vel = tmpValues[2];
    } else {
      t_jrk = 0.0;
      t_acc = 0.0;
      t_vel = 0.0;
    }
  }
  // return time for both: from v0_to_vf case and for equal_vel_case
  strs.str("");
  strs << __FUNCTION__ << ": " << __LINE__
       << " \n >>> output of traj_segment_planning: "
       << ", t_jrk_to_vf=" << t_jrk_to_vf << ", t_acc_to_vf=" << t_acc_to_vf
       << ", t_jrk=" << t_jrk << ", t_acc=" << t_acc << ", t_vel=" << t_vel
       << std::endl;
  LOG_DEBUG(strs);
  outValues1[0] = t_jrk_to_vf;
  outValues1[1] = t_acc_to_vf;
  outValues1[2] = t_jrk;
  outValues1[3] = t_acc;
  outValues1[4] = t_vel;
  return outValues1;
}

std::vector<double> SegmentPlanning::assign_jerk_sign_According_to_motion_type(
    const double p_start, const double p_end, const double v_start,
    const double v_end, const double p_max, const double v_max,
    const double a_max, const double j_max_orig) {
  std::ostringstream strs;
  double abs_v_start = fabs(v_start);
  double abs_v_end = fabs(v_end);
  double j_max_to_vf, j_max;

  if (fabs(v_start - v_end) < std::numeric_limits<double>::epsilon()) {
    j_max_to_vf = 0.0;
    j_max = p_end > p_start ? j_max_orig : -j_max_orig;
  } else {                      // v_end != v_start:
    if (v_start * v_end < 0) {  // won't be need it in complex motion case
      strs.str("");
      strs << "this a complex motion, stop point will be calculated "
           << "to join the +ve/-ve motion part " << std::endl;
      LOG_DEBUG(strs);
    } else if (abs_v_start < abs_v_end) {  // acc motion
      if (v_start >= 0 && v_end >= 0) {    // positive motion
        j_max_to_vf = j_max_orig;          // math.copysign(j_max, v_end)
        j_max = j_max_orig;
      } else {  // (v_start <= 0 and v_end <= 0): # negative motion
        j_max_to_vf = -j_max_orig;  // math.copysign(j_max, v_end)
        j_max = -j_max_orig;
      }
    } else {                             // v_start > v_end : #dec motion
      if (v_start >= 0 && v_end >= 0) {  //# positive motion
        j_max_to_vf = -j_max_orig;       // math.copysign(j_max, v_end)
        j_max = j_max_orig;              // math.copysign(j_max, v_end)
      } else {                           // # negative motion
        j_max_to_vf = j_max_orig;        // math.copysign(j_max, v_end)
        j_max = -j_max_orig;             // math.copysign(j_max, v_end)
      }
    }
  }

  std::vector<double> outValues;
  outValues.push_back(j_max_to_vf);
  outValues.push_back(j_max);
  return outValues;
}

std::vector<JerkTimePair> SegmentPlanning::calculate_jerk_sign_and_duration(
    const double p_start_orig, const double p_end_orig,
    const double v_start_orig, const double v_end_orig, const double p_max_orig,
    const double v_max_orig, const double a_max_orig, const double j_max_orig) {
  std::ostringstream strs;
  std::vector<JerkTimePair> outputPairs, emptyPairs;
  double p_max = fabs(p_max_orig);
  double a_max = fabs(a_max_orig);
  double j_max = fabs(j_max_orig);
  double v_max = fabs(v_max_orig);

  /*
   # Step_1:  check limits for given start/end velocities/positions
  # if absolute values v_start/v_end/p_end is greater than v_max/p_max, we
  replace the values with max one # another option is to raise error and exit #
  for p_start: it depends on direction of v_start, as we can not put p_start as
  p_max if v_start is in +ve direction
  */
  double v_start = v_start_orig, v_end = v_end_orig, p_start = p_start_orig,
         p_end = p_end_orig;
  if (fabs(v_start_orig) > v_max) {
    v_start = v_start_orig > 0 ? v_max : -v_max;
  }

  if (fabs(v_end_orig) > v_max) {
    v_end = v_end_orig > 0 ? v_max : -v_max;
  }

  if (fabs(p_end_orig) > p_max) {
    p_end = p_end_orig > 0 ? p_max : -p_max;
  }
  // bool nosolution = false;
  if (fabs(p_start_orig) > p_max) {
    p_start = p_start_orig > 0 ? p_max : -p_max;
    if (p_start * v_start > 0.0 ||
        (fabs(v_start) < std::numeric_limits<double>::epsilon() &&
         p_start * v_end > 0.0)) {  // direction of motion
      strs.str("");
      strs << "\nWarning: \n>>> these values are not feasible,"
           << "p_start = p_max, and motion in the direction of v_start will "
              "violate p_max!"
           << std::endl;
      LOG_ERROR(strs);
      // raise ValueError("non feasible case: violate p_max" )
      // nosolution = true;
      return emptyPairs;
    }
  }
  // # reject unfeasible/iillogical cases
  if (v_start > 0 && v_end > 0 &&
      p_end - p_start < 0) {  //# +ve motion vs -ve pos_diff
    strs.str("");
    strs << "non feasible case: vel_motion opposite to pos_motion" << std::endl;
    LOG_ERROR(strs);
    // nosolution = true;
    return emptyPairs;
  } else if (v_start < 0 && v_end < 0 &&
             p_end - p_start > 0) {  //: # -ve motion  vs +ve pos_diff
    strs.str("");
    strs << "non feasible case: vel_motion opposite to pos_motion" << std::endl;
    LOG_ERROR(strs);
    // nosolution = true;
    return emptyPairs;
  }

  // absolute value of the velocities
  double abs_v_start = fabs(v_start);
  double abs_v_end = fabs(v_end);

  //# Step_2:  check motion type: complex or simple motion
  // # 1) complex motion:  positive and negative velocities, v_start*v_end<0
  // ####
  double minPos_to_zero, acc_to_zero, t_jrk_to_zero, t_acc_to_zero;
  double minPos_to_vf, acc_to_vf, t_jrk_to_vf, t_acc_to_vf;
  double t_jrk_not_used, t_acc_not_used, t_jrk_dominant, t_acc_dominant,
      t_vel_dominant, t_jrk, t_acc, t_vel;
  std::vector<double> timeValues;
  if (v_start * v_end <
      0.0) {  //#complex motion:  positive and negative velocity, check min
              //distance to change diraction of the motion
    timeValues =
        calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(
            v_start, 0.0, v_max, a_max, j_max);
    minPos_to_zero = timeValues[0];
    acc_to_zero = timeValues[1];
    t_jrk_to_zero = timeValues[2];
    t_acc_to_zero = timeValues[3];
    timeValues =
        calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(
            0.0, v_end, v_max, a_max, j_max);
    minPos_to_vf = timeValues[0];
    acc_to_vf = timeValues[1];
    t_jrk_to_vf = timeValues[2];
    t_acc_to_vf = timeValues[3];
    double pos_diff = p_end - p_start;
    double pos_dominant = pos_diff - minPos_to_zero - minPos_to_vf;

    //# A) complex positive motion case
    if (pos_dominant > 0.0) {  // # positive dominant case, main part of the
                               // motion is in the +ve direction
      if (v_start < 0.0 && v_end > 0.0) {  //# from negative to positive
        if (fabs(p_start + minPos_to_zero) > p_max ||
            fabs(p_start + minPos_to_zero + minPos_to_vf) > p_max ||
            fabs(p_start + minPos_to_zero + minPos_to_vf + pos_dominant) >
                p_max) {
          strs.str("");
          strs << "non feasible case: violate p_max" << std::endl;
          LOG_ERROR(strs);
          return emptyPairs;
        }
        strs.str("");
        strs << "\n\n>>>positive dominant case: negative to positive"
             << ": pstart=" << p_start << ", p_end=" << p_end
             << ", v_start=" << v_start << ", v_end=" << v_end << std::endl;
        LOG_DEBUG(strs);
        timeValues = traj_segment_planning(
            p_start, p_end - minPos_to_zero - minPos_to_vf, abs_v_end,
            abs_v_end, v_max, a_max, j_max);
        t_jrk_not_used = timeValues[0];
        t_acc_not_used = timeValues[1];
        t_jrk_dominant = timeValues[2];
        t_acc_dominant = timeValues[3];
        t_vel_dominant = timeValues[4];

        outputPairs.clear();
        JerkTimePair mypair(j_max, t_jrk_to_zero);
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_vel_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

      } else if (v_start > 0.0 && v_end < 0.0) {  //#from positive to negative
        if (fabs(p_start + pos_dominant) > p_max ||
            fabs(p_start + pos_dominant + minPos_to_zero) > p_max ||
            fabs(p_start + pos_dominant + minPos_to_zero + minPos_to_vf) >
                p_max) {
          strs.str("");
          strs << "non feasible case: violate p_max" << std::endl;
          LOG_ERROR(strs);
          return emptyPairs;
        }
        strs.str("");
        strs << "\n\n>>>positive dominant case: positive to negative"
             << ": pstart=" << p_start << ", p_end=" << p_end
             << ", v_start=" << v_start << ", v_end=" << v_end << std::endl;
        LOG_DEBUG(strs);
        timeValues = traj_segment_planning(
            p_start, p_end - minPos_to_zero - minPos_to_vf, abs_v_start,
            abs_v_start, v_max, a_max, j_max);

        t_jrk_not_used = timeValues[0];
        t_acc_not_used = timeValues[1];
        t_jrk_dominant = timeValues[2];
        t_acc_dominant = timeValues[3];
        t_vel_dominant = timeValues[4];

        outputPairs.clear();
        JerkTimePair mypair(j_max, t_jrk_dominant);
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_vel_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);
      } else {
        strs.str("");
        strs << "should be simple motion instead of complex motion case!"
             << std::endl;
        LOG_ERROR(strs);
        return emptyPairs;
      }
    }

    //# B) complex negative motion case
    if (pos_dominant < 0.0) {  // # negative dominant case, main part of the
                               // motion is in the -ve direction
      if (v_start < 0.0 && v_end > 0.0) {  //# from negative to positive
        if (fabs(p_start + pos_dominant) > p_max ||
            fabs(p_start + pos_dominant + minPos_to_zero) > p_max ||
            fabs(p_start + pos_dominant + minPos_to_zero + minPos_to_vf) >
                p_max) {
          strs.str("");
          strs << __FUNCTION__ << ":" << __LINE__
               << ":non feasible case: violate p_max" << std::endl;
          LOG_ERROR(strs);
          return emptyPairs;
        }
        strs.str("");
        strs << "\n\n>>>negative dominant case: negative to positive"
             << ": pstart=" << p_start << ", p_end=" << p_end
             << ", v_start=" << v_start << ", v_end=" << v_end << std::endl;
        LOG_DEBUG(strs);
        timeValues = traj_segment_planning(
            p_start, p_end - minPos_to_zero - minPos_to_vf, abs_v_start,
            abs_v_start, v_max, a_max, j_max);
        t_jrk_not_used = timeValues[0];
        t_acc_not_used = timeValues[1];
        t_jrk_dominant = timeValues[2];
        t_acc_dominant = timeValues[3];
        t_vel_dominant = timeValues[4];

        outputPairs.clear();
        JerkTimePair mypair(-j_max, t_jrk_dominant);
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_vel_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);
      } else if (v_start > 0.0 && v_end < 0.0) {  //#from positive to negative
        if (fabs(p_start + minPos_to_zero) > p_max ||
            fabs(p_start + minPos_to_zero + minPos_to_vf) > p_max ||
            fabs(p_start + minPos_to_zero + minPos_to_vf + pos_dominant) >
                p_max) {
          strs.str("");
          strs << __FUNCTION__ << ":" << __LINE__
               << ":non feasible case: violate p_max" << std::endl;
          LOG_ERROR(strs);
          return emptyPairs;
        }
        strs.str("");
        strs << "\n\n>>>negative dominant case: positive to negative"
             << ": pstart=" << p_start << ", p_end=" << p_end
             << ", v_start=" << v_start << ", v_end=" << v_end << std::endl;
        LOG_DEBUG(strs);
        timeValues = traj_segment_planning(
            p_start + minPos_to_zero + minPos_to_vf, p_end, abs_v_end,
            abs_v_end, v_max, a_max, j_max);
        t_jrk_not_used = timeValues[0];
        t_acc_not_used = timeValues[1];
        t_jrk_dominant = timeValues[2];
        t_acc_dominant = timeValues[3];
        t_vel_dominant = timeValues[4];

        outputPairs.clear();
        JerkTimePair mypair(-j_max, t_jrk_to_zero);
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_zero;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_to_vf;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0.0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_vel_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = 0;
        mypair.TimeInterval_ = t_acc_dominant;
        outputPairs.push_back(mypair);

        mypair.Jerk_ = -j_max;
        mypair.TimeInterval_ = t_jrk_dominant;
        outputPairs.push_back(mypair);

      } else {
        strs.str("");
        strs << "should be simple motion instead of complex motion case!"
             << std::endl;
        LOG_ERROR(strs);
        return emptyPairs;
      }
    }
    //# check if final_velocity value gives optimal motion to change from
    //+ve/-ve to -ve/+ve # this part can be used later to assign velocity vf in
    //the parameterizarion part
    double minPos_v02vf = minPos_to_zero + minPos_to_vf;
    if (v_start < 0 && v_end > 0) {  //#from -ve to +ve
      if (pos_diff < minPos_v02vf) {
        strs.str("");
        strs << ">>>>>> non optimal case <<<<<<< " << std::endl;
        LOG_DEBUG(strs);
      }
    } else {
      if (pos_diff > minPos_v02vf) {
        strs.str("");
        strs << ">>>>>> non optimal case <<<<<<< " << std::endl;
        LOG_DEBUG(strs);
      }
    }
  } else {
    //# 2)simple motion:  positive or negative velocity, v0 and vf have same
    //sign
    // same action will be performed in both simple +ve or simple -ve motion,
    // this part can be used later # A) simple positive motion
    strs.str("");
    if (v_start >= 0 && v_end >= 0) {  //# case one: both are positive
      strs << ">>>>>> simple postive motion: <<<<<<< " << std::endl;
    } else if (v_start <= 0 && v_end <= 0) {  //# B) simple negative motion
      //# case two: both are negative
      strs << ">>>>>> simple negative motion: <<<<<<< " << std::endl;
    }
    strs << ": pstart=" << p_start << ", p_end=" << p_end
         << ", v_start=" << v_start << ", v_end=" << v_end << std::endl;
    LOG_DEBUG(strs);
    timeValues = traj_segment_planning(p_start, p_end, abs_v_start, abs_v_end,
                                       v_max, a_max, j_max);
    t_jrk_to_vf = timeValues[0];
    t_acc_to_vf = timeValues[1];
    t_jrk = timeValues[2];
    t_acc = timeValues[3];
    t_vel = timeValues[4];
    std::vector<double> tmpValues = assign_jerk_sign_According_to_motion_type(
        p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max);
    double j_max_to_vf = tmpValues[0];
    j_max = tmpValues[1];
    if (abs_v_end > abs_v_start) {
      outputPairs.clear();
      JerkTimePair mypair(j_max_to_vf, t_jrk_to_vf);
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_acc_to_vf;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = -j_max_to_vf;
      mypair.TimeInterval_ = t_jrk_to_vf;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_acc;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = -j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_vel;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = -j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_acc;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);
    } else {
      outputPairs.clear();
      JerkTimePair mypair(j_max, t_jrk);
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_acc;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = -j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_vel;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = -j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_acc;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = j_max;
      mypair.TimeInterval_ = t_jrk;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = j_max_to_vf;
      mypair.TimeInterval_ = t_jrk_to_vf;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = 0;
      mypair.TimeInterval_ = t_acc_to_vf;
      outputPairs.push_back(mypair);

      mypair.Jerk_ = -j_max_to_vf;
      mypair.TimeInterval_ = t_jrk_to_vf;
      outputPairs.push_back(mypair);
    }
  }
  //  # one option to retun segment_jerks_and_durations and send it to JTC and
  //  then use it for interpolation on the JTC side
  return outputPairs;
}

std::vector<PiecewiseFunction> SegmentPlanning::fit_traj_segment(
    const double p_start, const double p_end, const double v_start,
    const double v_end, const double p_max, const double v_max,
    const double a_max, const double j_max) {
  std::ostringstream strs;
  std::vector<PiecewiseFunction> outputPieceWiseFunctions;

  //# Step_1. calculate jerk_sign_and_duration
  std::vector<JerkTimePair> jtPairs = calculate_jerk_sign_and_duration(
      p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max);

  strs.str("");
  strs << "segment_jerks_and_durations=";
  for (size_t i = 0; i < jtPairs.size(); i++) {
    strs << "(" << jtPairs[i].Jerk_ << "," << jtPairs[i].TimeInterval_ << "), ";
  }
  strs << std::endl;
  LOG_DEBUG(strs);
  // # Step_2:  generate pos, vel, acc, jrk using the calculated
  // "segment_jerks_and_durations"
  double p0 = p_start;
  double v0 = v_start;
  double a0 = 0.0;
  std::vector<double> times;
  std::vector<PolyNomials> jerk_functions, acceleration_functions,
      velocity_functions, position_functions;
  times.push_back(0.0);
  //# Integrate jerk starting from the start of the trajectory and going all the
  //way through the end.
  for (size_t i = 0; i < jtPairs.size(); i++) {
    double T = jtPairs[i].TimeInterval_;
    times.push_back(times.back() + T);
    std::vector<double> jk_coef;
    jk_coef.push_back(jtPairs[i].Jerk_);
    PolyNomials jk_f(jk_coef);
    jerk_functions.push_back(jk_f);

    PolyNomials acc_f = jk_f.Integrate(a0);
    acceleration_functions.push_back(acc_f);

    PolyNomials vel_f = acc_f.Integrate(v0);
    velocity_functions.push_back(vel_f);

    PolyNomials pos_f = vel_f.Integrate(p0);
    position_functions.push_back(pos_f);

    a0 = acc_f.evaluate(T);
    v0 = vel_f.evaluate(T);
    p0 = pos_f.evaluate(T);
  }

  PiecewiseFunction position(times, position_functions);
  PiecewiseFunction velocity(times, velocity_functions);
  PiecewiseFunction acceleration(times, acceleration_functions);
  PiecewiseFunction jerk(times, jerk_functions);
  outputPieceWiseFunctions.push_back(position);
  outputPieceWiseFunctions.push_back(velocity);
  outputPieceWiseFunctions.push_back(acceleration);
  outputPieceWiseFunctions.push_back(jerk);
  return outputPieceWiseFunctions;
}

std::vector<double> SegmentPlanning::fit_traj_segment_samples(
    const double p_start, const double p_end, const double v_start,
    const double v_end, const double p_max, const double v_max,
    const double a_max, const double j_max, const int n_points,
    EigenDRef<Eigen::VectorXd> &time_list, EigenDRef<Eigen::VectorXd> &pos_list,
    EigenDRef<Eigen::VectorXd> &vel_list, EigenDRef<Eigen::VectorXd> &acc_list,
    EigenDRef<Eigen::VectorXd> &jerk_list) {
  std::ostringstream strs;
  std::vector<double> bounds;
  std::vector<PiecewiseFunction> pfs = fit_traj_segment(
      p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max);

  if (n_points < 2 || time_list.size() != n_points ||
      pos_list.size() != n_points || vel_list.size() != n_points ||
      acc_list.size() != n_points || jerk_list.size() != n_points) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__
         << ", input data size not matching with n_points=" << n_points
         << std::endl;
    LOG_ERROR(strs);
    return bounds;
  }

  PiecewiseFunction pos = pfs[0];
  PiecewiseFunction vel = pfs[1];
  PiecewiseFunction acc = pfs[2];
  PiecewiseFunction jerk = pfs[3];

  double t_i, t_f;
  if (!jerk.lowest_t_bound(t_i) || !jerk.largest_t_bound(t_f)) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__
         << ", get time lower and upper bound error" << std::endl;
    LOG_ERROR(strs);
    return bounds;
  }
  bounds = jerk.GetBoundary();
  double delta_t = (t_f - t_i) / (n_points - 1);
  // time_list.clear();
  // pos_list.clear(); vel_list.clear(); acc_list.clear(); jerk_list.clear();
  double t = t_i;
  for (size_t i = 0; i < n_points; i++) {
    time_list(i) = t;
    // pos_list.push_back(pos.evaluate(t));
    pos_list(i) = pos.evaluate(t);
    // vel_list.push_back(vel.evaluate(t));
    vel_list(i) = vel.evaluate(t);
    // acc_list.push_back(acc.evaluate(t));
    acc_list(i) = acc.evaluate(t);
    // jerk_list.push_back(jerk.evaluate(t));
    jerk_list(i) = jerk.evaluate(t);
    t += delta_t;
  }
  return bounds;
}

}  // namespace kinematics_lib