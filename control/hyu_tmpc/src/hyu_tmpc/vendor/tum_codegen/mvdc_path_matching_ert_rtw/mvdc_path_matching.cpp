//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_path_matching.cpp
//
// Code generated for Simulink model 'mvdc_path_matching'.
//
// Model version                  : 14.0
// Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
// C/C++ source code generated on : Tue Jun 30 15:12:20 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "mvdc_path_matching.h"
#include "rtwtypes.h"
#include <cstring>
#include <cmath>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "mvdc_path_matching_types.h"
#include "mvdc_path_matching_capi.h"
#include "mvdc_path_matching_private.h"

// Forward declaration for local functions
static boolean_T mvdc_path_matching_vectorAny(const boolean_T x_data[], const
  int32_T *x_size);
static void mvdc_path_matching_minimum(const real_T x[50], real_T *ex, int32_T
  *idx);
static real_T mvdc_path_matching_normalizeAngle(real_T u);
static real_T mvdc_path_matching_interp1_k(const real_T varargin_1[50], const
  real_T varargin_2[50], real_T varargin_3);
static real_T mvdc_path_matching_interp1(const real_T varargin_1[2], const
  real_T varargin_2[2], real_T varargin_3);
static real_T mvdc_path_matching_interp1_k4t(const real_T varargin_1[50], const
  real_T varargin_2[50], real_T varargin_3, real_T varargin_5);
static real_T mvdc_path_matching_interp1_k4(const real_T varargin_1[2], const
  real_T varargin_2[2], real_T varargin_3, real_T varargin_5);

// Function for MATLAB Function: '<S2>/MATLAB Function1'
static boolean_T mvdc_path_matching_vectorAny(const boolean_T x_data[], const
  int32_T *x_size)
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= *x_size - 1)) {
    if (x_data[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void mvdc_path_matching_minimum(const real_T x[50], real_T *ex, int32_T
  *idx)
{
  int32_T b_idx;
  int32_T k;
  if (!rtIsNaN(x[0])) {
    b_idx = 1;
  } else {
    boolean_T exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 51)) {
      if (!rtIsNaN(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[b_idx - 1];
    *idx = b_idx;
    for (k = b_idx + 1; k < 51; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (*ex > x_0) {
        *ex = x_0;
        *idx = k;
      }
    }
  }
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T mvdc_path_matching_normalizeAngle(real_T u)
{
  real_T q;
  real_T y;
  y = u;
  if (u > 3.1415926535897931) {
    if (rtIsInf(u + 3.1415926535897931)) {
      q = (rtNaN);
    } else {
      q = (u + 3.1415926535897931) / 6.2831853071795862;
      if (std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q) {
        q = std::fmod(u + 3.1415926535897931, 6.2831853071795862);
      } else {
        q = 0.0;
      }
    }

    y = q - 3.1415926535897931;
  }

  if (u < -3.1415926535897931) {
    if (rtIsInf(-(u - 3.1415926535897931))) {
      q = (rtNaN);
    } else {
      q = -(u - 3.1415926535897931) / 6.2831853071795862;
      if (std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q) {
        q = std::fmod(-(u - 3.1415926535897931), 6.2831853071795862);
      } else {
        q = 0.0;
      }
    }

    y = -(q - 3.1415926535897931);
  }

  return y;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T mvdc_path_matching_interp1_k(const real_T varargin_1[50], const
  real_T varargin_2[50], real_T varargin_3)
{
  real_T x[50];
  real_T y[50];
  real_T Vq;
  real_T xtmp;
  int32_T i;
  std::memcpy(&y[0], &varargin_2[0], 50U * sizeof(real_T));
  std::memcpy(&x[0], &varargin_1[0], 50U * sizeof(real_T));
  if (varargin_1[1] < varargin_1[0]) {
    for (i = 0; i < 25; i++) {
      xtmp = x[i];
      x[i] = x[49 - i];
      x[49 - i] = xtmp;
      xtmp = y[i];
      y[i] = y[49 - i];
      y[49 - i] = xtmp;
    }
  }

  if (rtIsNaN(varargin_3)) {
    Vq = (rtNaN);
  } else if (varargin_3 > x[49]) {
    Vq = (varargin_3 - x[49]) / (x[49] - x[48]) * (y[49] - y[48]) + y[49];
  } else if (varargin_3 < x[0]) {
    Vq = (varargin_3 - x[0]) / (x[1] - x[0]) * (y[1] - y[0]) + y[0];
  } else {
    int32_T high_i;
    int32_T low_ip1;
    i = 1;
    low_ip1 = 2;
    high_i = 50;
    while (high_i > low_ip1) {
      int32_T mid_i;
      mid_i = (i + high_i) >> 1;
      if (varargin_3 >= x[mid_i - 1]) {
        i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    xtmp = x[i - 1];
    xtmp = (varargin_3 - xtmp) / (x[i] - xtmp);
    if (xtmp == 0.0) {
      Vq = y[i - 1];
    } else if (xtmp == 1.0) {
      Vq = y[i];
    } else if (y[i - 1] == y[i]) {
      Vq = y[i - 1];
    } else {
      Vq = (1.0 - xtmp) * y[i - 1] + xtmp * y[i];
    }
  }

  return Vq;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T mvdc_path_matching_interp1(const real_T varargin_1[2], const
  real_T varargin_2[2], real_T varargin_3)
{
  real_T Vq;
  real_T r;
  real_T x_idx_1;
  real_T y_idx_1;
  Vq = varargin_2[0];
  r = varargin_1[0];
  y_idx_1 = varargin_2[1];
  x_idx_1 = varargin_1[1];
  if (varargin_1[1] < varargin_1[0]) {
    r = varargin_1[1];
    x_idx_1 = varargin_1[0];
    Vq = varargin_2[1];
    y_idx_1 = varargin_2[0];
  }

  if (rtIsNaN(varargin_3)) {
    Vq = (rtNaN);
  } else if (varargin_3 > x_idx_1) {
    Vq = (varargin_3 - x_idx_1) / (x_idx_1 - r) * (y_idx_1 - Vq) + y_idx_1;
  } else if (varargin_3 < r) {
    Vq += (varargin_3 - r) / (x_idx_1 - r) * (y_idx_1 - Vq);
  } else {
    r = (varargin_3 - r) / (x_idx_1 - r);
    if (!(r == 0.0)) {
      if (r == 1.0) {
        Vq = y_idx_1;
      } else if (!(Vq == y_idx_1)) {
        Vq = (1.0 - r) * Vq + r * y_idx_1;
      }
    }
  }

  return Vq;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T mvdc_path_matching_interp1_k4t(const real_T varargin_1[50], const
  real_T varargin_2[50], real_T varargin_3, real_T varargin_5)
{
  real_T x[50];
  real_T y[50];
  real_T Vq;
  real_T xtmp;
  int32_T i;
  Vq = varargin_5;
  std::memcpy(&y[0], &varargin_2[0], 50U * sizeof(real_T));
  std::memcpy(&x[0], &varargin_1[0], 50U * sizeof(real_T));
  if (varargin_1[1] < varargin_1[0]) {
    for (i = 0; i < 25; i++) {
      xtmp = x[i];
      x[i] = x[49 - i];
      x[49 - i] = xtmp;
      xtmp = y[i];
      y[i] = y[49 - i];
      y[49 - i] = xtmp;
    }
  }

  if (rtIsNaN(varargin_3)) {
    Vq = (rtNaN);
  } else if ((!(varargin_3 > x[49])) && (!(varargin_3 < x[0]))) {
    int32_T high_i;
    int32_T low_ip1;
    i = 1;
    low_ip1 = 2;
    high_i = 50;
    while (high_i > low_ip1) {
      int32_T mid_i;
      mid_i = (i + high_i) >> 1;
      if (varargin_3 >= x[mid_i - 1]) {
        i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    xtmp = x[i - 1];
    xtmp = (varargin_3 - xtmp) / (x[i] - xtmp);
    if (xtmp == 0.0) {
      Vq = y[i - 1];
    } else if (xtmp == 1.0) {
      Vq = y[i];
    } else if (y[i - 1] == y[i]) {
      Vq = y[i - 1];
    } else {
      Vq = (1.0 - xtmp) * y[i - 1] + xtmp * y[i];
    }
  }

  return Vq;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T mvdc_path_matching_interp1_k4(const real_T varargin_1[2], const
  real_T varargin_2[2], real_T varargin_3, real_T varargin_5)
{
  real_T Vq;
  real_T r;
  real_T x_idx_1;
  real_T y_idx_0;
  real_T y_idx_1;
  Vq = varargin_5;
  y_idx_0 = varargin_2[0];
  r = varargin_1[0];
  y_idx_1 = varargin_2[1];
  x_idx_1 = varargin_1[1];
  if (varargin_1[1] < varargin_1[0]) {
    r = varargin_1[1];
    x_idx_1 = varargin_1[0];
    y_idx_0 = varargin_2[1];
    y_idx_1 = varargin_2[0];
  }

  if (rtIsNaN(varargin_3)) {
    Vq = (rtNaN);
  } else if ((!(varargin_3 > x_idx_1)) && (!(varargin_3 < r))) {
    r = (varargin_3 - r) / (x_idx_1 - r);
    if (r == 0.0) {
      Vq = y_idx_0;
    } else if (r == 1.0) {
      Vq = y_idx_1;
    } else if (y_idx_0 == y_idx_1) {
      Vq = y_idx_0;
    } else {
      Vq = (1.0 - r) * y_idx_0 + r * y_idx_1;
    }
  }

  return Vq;
}

// Model step function
void mvdc_path_matching_step(RT_MODEL_mvdc_path_matching_T *const
  mvdc_path_matching_M)
{
  B_mvdc_path_matching_T *mvdc_path_matching_B = mvdc_path_matching_M->blockIO;
  DW_mvdc_path_matching_T *mvdc_path_matching_DW = mvdc_path_matching_M->dwork;
  ExtU_mvdc_path_matching_T *mvdc_path_matching_U =
    static_cast<ExtU_mvdc_path_matching_T *>(mvdc_path_matching_M->inputs);
  ExtY_mvdc_path_matching_T *mvdc_path_matching_Y =
    static_cast<ExtY_mvdc_path_matching_T *>(mvdc_path_matching_M->outputs);
  real_T ax_mps2_recalc[50];
  real_T diff_x_m[50];
  real_T dist_squared_trialpoints[50];
  real_T tmp_0[2];
  real_T tmp_1[2];
  real_T b_tmp2;
  real_T c_tmp2;
  real_T tmp2;
  real_T u1;
  real_T work;
  int32_T exitg1;
  int32_T high_i;
  int32_T i;
  int32_T i_0;
  int32_T low_ip1;
  int32_T mid_i;
  boolean_T y_data[50];
  boolean_T c;
  boolean_T exitg2;
  boolean_T rtb_LogicalOperator5;
  boolean_T rtb_traj_ok;
  boolean_T rtb_traj_ok_j;
  boolean_T tmp;
  boolean_T tmp_2;
  boolean_T tmp_3;

  // MATLAB Function: '<S2>/MATLAB Function2' incorporates:
  //   Inport: '<Root>/TrajectoryPlanning'

  if (mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.TrajCnt ==
      0U) {
    rtb_traj_ok = false;
  } else {
    work =
      mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.s_loc_m[0];
    i = 49;
    for (i_0 = 0; i_0 < 49; i_0++) {
      tmp2 = work;
      work =
        mvdc_path_matching_U->
        TrajectoryPlanning_m.PerformanceTrajectory.s_loc_m[i_0 + 1];
      y_data[i_0] = (work - tmp2 <= 0.0);
    }

    if (mvdc_path_matching_vectorAny(y_data, &i)) {
      rtb_traj_ok = false;
    } else {
      tmp2 = 0.5 * mvdc_path_matching_P.drag_coefficient *
        mvdc_path_matching_P.roh_air;
      i = 50;
      for (i_0 = 0; i_0 < 50; i_0++) {
        work =
          mvdc_path_matching_U->
          TrajectoryPlanning_m.PerformanceTrajectory.v_mps[i_0];
        work *= work;
        y_data[i_0] = !(std::abs((work * tmp2 /
          mvdc_path_matching_P.vehiclemass_kg +
          mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.ax_mps2
          [i_0]) /
          mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.ax_lim_mps2
          [i_0]) + std::abs(work *
                            mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.kappa_radpm
                            [i_0] /
                            mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.ay_lim_mps2
                            [i_0]) <= 1.025);
      }

      if (mvdc_path_matching_vectorAny(y_data, &i)) {
        rtb_traj_ok = false;
      } else {
        std::memset(&ax_mps2_recalc[0], 0, 50U * sizeof(real_T));
        work =
          mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.s_loc_m
          [0];
        tmp2 =
          mvdc_path_matching_U->
          TrajectoryPlanning_m.PerformanceTrajectory.v_mps[0];
        for (i = 0; i < 49; i++) {
          b_tmp2 = work;
          work =
            mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.s_loc_m
            [i + 1];
          c_tmp2 = tmp2;
          tmp2 =
            mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.v_mps
            [i + 1];
          c_tmp2 = tmp2 - c_tmp2;
          ax_mps2_recalc[i] = (2.0 * c_tmp2 *
                               mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.v_mps
                               [i] + c_tmp2 * c_tmp2) / ((work - b_tmp2) * 2.0);
        }

        ax_mps2_recalc[49] = ax_mps2_recalc[48];
        for (i_0 = 0; i_0 < 50; i_0++) {
          ax_mps2_recalc[i_0] -=
            mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.ax_mps2
            [i_0];
        }

        i = 0;
        do {
          exitg1 = 0;
          if (i < 49) {
            if (mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.v_mps
                [i + 1] > 0.0) {
              for (i_0 = 0; i_0 < 50; i_0++) {
                diff_x_m[i_0] = std::abs(ax_mps2_recalc[i_0]);
              }

              rtb_LogicalOperator5 = true;
              i_0 = 0;
              exitg2 = false;
              while ((!exitg2) && (i_0 < 50)) {
                if (!(diff_x_m[i_0] > 2.0)) {
                  rtb_LogicalOperator5 = false;
                  exitg2 = true;
                } else {
                  i_0++;
                }
              }

              if (rtb_LogicalOperator5) {
                rtb_traj_ok = false;
                exitg1 = 1;
              } else {
                i++;
              }
            } else {
              for (i_0 = 0; i_0 < 50; i_0++) {
                y_data[i_0] =
                  (mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory.ax_mps2
                   [i_0] > 0.0);
              }

              rtb_LogicalOperator5 = true;
              low_ip1 = 0;
              exitg2 = false;
              while ((!exitg2) && (low_ip1 < 50)) {
                if (!y_data[low_ip1]) {
                  rtb_LogicalOperator5 = false;
                  exitg2 = true;
                } else {
                  low_ip1++;
                }
              }

              if (rtb_LogicalOperator5) {
                rtb_traj_ok = false;
                exitg1 = 1;
              } else {
                i++;
              }
            }
          } else {
            rtb_traj_ok = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  // End of MATLAB Function: '<S2>/MATLAB Function2'

  // MATLAB Function: '<S2>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/TrajectoryPlanning'

  if (mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.TrajCnt ==
      0U) {
    rtb_traj_ok_j = false;
  } else {
    work =
      mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.s_loc_m[0];
    i = 49;
    for (i_0 = 0; i_0 < 49; i_0++) {
      tmp2 = work;
      work =
        mvdc_path_matching_U->
        TrajectoryPlanning_m.EmergencyTrajectory.s_loc_m[i_0 + 1];
      y_data[i_0] = (work - tmp2 <= 0.0);
    }

    if (mvdc_path_matching_vectorAny(y_data, &i)) {
      rtb_traj_ok_j = false;
    } else {
      tmp2 = 0.5 * mvdc_path_matching_P.drag_coefficient *
        mvdc_path_matching_P.roh_air;
      i = 50;
      for (i_0 = 0; i_0 < 50; i_0++) {
        work =
          mvdc_path_matching_U->
          TrajectoryPlanning_m.EmergencyTrajectory.v_mps[i_0];
        work *= work;
        y_data[i_0] = !(std::abs((work * tmp2 /
          mvdc_path_matching_P.vehiclemass_kg +
          mvdc_path_matching_U->
          TrajectoryPlanning_m.EmergencyTrajectory.ax_mps2[i_0]) /
          mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.ax_lim_mps2
          [i_0]) + std::abs(work *
                            mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.kappa_radpm
                            [i_0] /
                            mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.ay_lim_mps2
                            [i_0]) <= 1.025);
      }

      if (mvdc_path_matching_vectorAny(y_data, &i)) {
        rtb_traj_ok_j = false;
      } else if
          (mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.v_mps
           [49] > mvdc_path_matching_P.P_VDC_FinalSpeedEmergency_mps) {
        rtb_traj_ok_j = false;
      } else {
        std::memset(&ax_mps2_recalc[0], 0, 50U * sizeof(real_T));
        work =
          mvdc_path_matching_U->
          TrajectoryPlanning_m.EmergencyTrajectory.s_loc_m[0];
        tmp2 =
          mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.v_mps[0];
        for (i = 0; i < 49; i++) {
          b_tmp2 = work;
          work =
            mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.s_loc_m
            [i + 1];
          c_tmp2 = tmp2;
          tmp2 =
            mvdc_path_matching_U->
            TrajectoryPlanning_m.EmergencyTrajectory.v_mps[i + 1];
          c_tmp2 = tmp2 - c_tmp2;
          ax_mps2_recalc[i] = (2.0 * c_tmp2 *
                               mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.v_mps
                               [i] + c_tmp2 * c_tmp2) / ((work - b_tmp2) * 2.0);
        }

        ax_mps2_recalc[49] = ax_mps2_recalc[48];
        for (i_0 = 0; i_0 < 50; i_0++) {
          ax_mps2_recalc[i_0] -=
            mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.ax_mps2
            [i_0];
        }

        i = 0;
        do {
          exitg1 = 0;
          if (i < 49) {
            if (mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.v_mps
                [i + 1] > 0.0) {
              for (i_0 = 0; i_0 < 50; i_0++) {
                dist_squared_trialpoints[i_0] = std::abs(ax_mps2_recalc[i_0]);
              }

              rtb_LogicalOperator5 = true;
              i_0 = 0;
              exitg2 = false;
              while ((!exitg2) && (i_0 < 50)) {
                if (!(dist_squared_trialpoints[i_0] > 2.0)) {
                  rtb_LogicalOperator5 = false;
                  exitg2 = true;
                } else {
                  i_0++;
                }
              }

              if (rtb_LogicalOperator5) {
                rtb_traj_ok_j = false;
                exitg1 = 1;
              } else {
                i++;
              }
            } else {
              for (i_0 = 0; i_0 < 50; i_0++) {
                y_data[i_0] =
                  (mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory.ax_mps2
                   [i_0] > 0.0);
              }

              rtb_LogicalOperator5 = true;
              low_ip1 = 0;
              exitg2 = false;
              while ((!exitg2) && (low_ip1 < 50)) {
                if (!y_data[low_ip1]) {
                  rtb_LogicalOperator5 = false;
                  exitg2 = true;
                } else {
                  low_ip1++;
                }
              }

              if (rtb_LogicalOperator5) {
                rtb_traj_ok_j = false;
                exitg1 = 1;
              } else {
                i++;
              }
            }
          } else {
            rtb_traj_ok_j = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }

  // End of MATLAB Function: '<S2>/MATLAB Function1'

  // Switch: '<S2>/Switch2' incorporates:
  //   Constant: '<S2>/Constant'
  //   Constant: '<S2>/Constant1'
  //   Switch: '<S2>/Switch'

  if (!mvdc_path_matching_P.P_VDC_EnableSafetyChecks) {
    rtb_traj_ok = mvdc_path_matching_P.Constant1_Value;
    rtb_traj_ok_j = mvdc_path_matching_P.Constant1_Value;
  }

  // End of Switch: '<S2>/Switch2'

  // Switch: '<S2>/Switch3' incorporates:
  //   Delay: '<S2>/Delay2'
  //   RelationalOperator: '<S5>/FixPt Relational Operator'
  //   UnitDelay: '<S5>/Delay Input1'
  //
  //  Block description for '<S5>/Delay Input1':
  //
  //   Store in Global RAM

  if (mvdc_path_matching_U->EnableEmergency !=
      mvdc_path_matching_DW->DelayInput1_DSTATE) {
    mvdc_path_matching_DW->Delay2_DSTATE =
      mvdc_path_matching_U->TrajectoryPlanning_m.EmergencyTrajectory;
  }

  // End of Switch: '<S2>/Switch3'

  // Switch: '<S2>/Switch1' incorporates:
  //   Delay: '<S2>/Delay2'

  if (mvdc_path_matching_U->EnableEmergency) {
    mvdc_path_matching_Y->ActualTargetTrajectory =
      mvdc_path_matching_DW->Delay2_DSTATE;
  } else {
    mvdc_path_matching_Y->ActualTargetTrajectory =
      mvdc_path_matching_U->TrajectoryPlanning_m.PerformanceTrajectory;
  }

  // End of Switch: '<S2>/Switch1'

  // Logic: '<S2>/Logical Operator5'
  rtb_LogicalOperator5 = (rtb_traj_ok_j && rtb_traj_ok);

  // Switch: '<Root>/Switch3' incorporates:
  //   Delay: '<Root>/Delay2'
  //   Logic: '<Root>/Logical Operator'

  mvdc_path_matching_DW->Delay2_DSTATE_j = ((rtb_LogicalOperator5 &&
    mvdc_path_matching_U->EnablePathMatching) ||
    mvdc_path_matching_DW->Delay2_DSTATE_j);

  // Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
  //   EnablePort: '<S1>/Enable'

  if (mvdc_path_matching_DW->Delay2_DSTATE_j) {
    // MATLAB Function: '<S1>/MATLAB Function' incorporates:
    //   Inport: '<Root>/VehicleDynamicState'
    //   Outport: '<Root>/ActualTrajectoryPoint'
    //   Outport: '<Root>/PathPos'
    //   SignalConversion generated from: '<S1>/d_m'

    for (i_0 = 0; i_0 < 50; i_0++) {
      work = mvdc_path_matching_U->VehicleDynamicState_g.Pos.x_m -
        mvdc_path_matching_Y->ActualTargetTrajectory.x_m[i_0];
      diff_x_m[i_0] = work;
      tmp2 = mvdc_path_matching_U->VehicleDynamicState_g.Pos.y_m -
        mvdc_path_matching_Y->ActualTargetTrajectory.y_m[i_0];
      ax_mps2_recalc[i_0] = tmp2;
      dist_squared_trialpoints[i_0] = work * work + tmp2 * tmp2;
    }

    mvdc_path_matching_minimum(dist_squared_trialpoints, &work, &i_0);
    if (!rtIsNaN(dist_squared_trialpoints[0])) {
      i = 1;
    } else {
      i = 0;
      low_ip1 = 2;
      exitg2 = false;
      while ((!exitg2) && (low_ip1 < 51)) {
        if (!rtIsNaN(dist_squared_trialpoints[low_ip1 - 1])) {
          i = low_ip1;
          exitg2 = true;
        } else {
          low_ip1++;
        }
      }
    }

    if (i == 0) {
      dist_squared_trialpoints[i_0 - 1] = dist_squared_trialpoints[0];
    } else {
      work = dist_squared_trialpoints[i - 1];
      for (low_ip1 = i + 1; low_ip1 < 51; low_ip1++) {
        tmp2 = dist_squared_trialpoints[low_ip1 - 1];
        if (work < tmp2) {
          work = tmp2;
        }
      }

      dist_squared_trialpoints[i_0 - 1] = work;
    }

    mvdc_path_matching_minimum(dist_squared_trialpoints, &work, &i);
    if (i > i_0) {
      b_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.x_m[i_0 - 1];
      work = mvdc_path_matching_Y->ActualTargetTrajectory.x_m[i - 1] - b_tmp2;
      c_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.y_m[i_0 - 1];
      tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.y_m[i - 1] - c_tmp2;
      b_tmp2 = mvdc_path_matching_U->VehicleDynamicState_g.Pos.x_m - b_tmp2;
      c_tmp2 = mvdc_path_matching_U->VehicleDynamicState_g.Pos.y_m - c_tmp2;
    } else {
      b_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.x_m[i - 1];
      work = mvdc_path_matching_Y->ActualTargetTrajectory.x_m[i_0 - 1] - b_tmp2;
      c_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.y_m[i - 1];
      tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.y_m[i_0 - 1] - c_tmp2;
      b_tmp2 = mvdc_path_matching_U->VehicleDynamicState_g.Pos.x_m - b_tmp2;
      c_tmp2 = mvdc_path_matching_U->VehicleDynamicState_g.Pos.y_m - c_tmp2;
    }

    u1 = std::sqrt(work * work + tmp2 * tmp2);
    if ((u1 <= 0.05) || rtIsNaN(u1)) {
      u1 = 0.05;
    }

    mvdc_path_matching_B->d_m = -(b_tmp2 * tmp2 - work * c_tmp2) / u1;
    mvdc_path_matching_Y->PathPos_i.d_m = mvdc_path_matching_B->d_m;
    work = -(mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i_0 - 1] +
             1.5707963267948966);
    mvdc_path_matching_Y->PathPos_i.s_m = (diff_x_m[i_0 - 1] * std::cos(work) -
      ax_mps2_recalc[i_0 - 1] * std::sin(work)) +
      mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i_0 - 1];
    if ((mvdc_path_matching_Y->PathPos_i.s_m <=
         mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[49]) || rtIsNaN
        (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[49])) {
      work = mvdc_path_matching_Y->PathPos_i.s_m;
    } else {
      work = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[49];
    }

    if ((work >= mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0]) ||
        rtIsNaN(mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0])) {
      mvdc_path_matching_Y->PathPos_i.s_m = work;
    } else {
      mvdc_path_matching_Y->PathPos_i.s_m =
        mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0];
    }

    work = mvdc_path_matching_Y->PathPos_i.s_m;
    i = 1;
    do {
      exitg1 = 0;
      if (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i] < work) {
        i++;
        if (i + 1 > 50) {
          tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[49];
          exitg1 = 1;
        }
      } else {
        tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i - 1];
        b_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i] - tmp2;
        if (b_tmp2 > 3.1415926535897931) {
          tmp_0[0] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i - 1];
          tmp_0[1] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i];
          tmp_1[0] = tmp2;
          tmp_1[1] = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i] -
            6.2831853071795862;
          tmp2 = mvdc_path_matching_interp1(tmp_0, tmp_1, work);
        } else if (b_tmp2 < -3.1415926535897931) {
          tmp_0[0] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i - 1];
          tmp_0[1] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i];
          tmp_1[0] = tmp2;
          tmp_1[1] = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i] +
            6.2831853071795862;
          tmp2 = mvdc_path_matching_interp1(tmp_0, tmp_1, work);
        } else {
          tmp_0[0] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i - 1];
          tmp_0[1] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i];
          tmp_1[0] = tmp2;
          tmp_1[1] = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i];
          tmp2 = mvdc_path_matching_interp1(tmp_0, tmp_1, work);
        }

        tmp2 = mvdc_path_matching_normalizeAngle(tmp2);
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    mvdc_path_matching_Y->PathPos_i.psi_rad = mvdc_path_matching_normalizeAngle
      (mvdc_path_matching_U->VehicleDynamicState_g.Pos.psi_rad - tmp2);
    mvdc_path_matching_Y->ActualTrajectoryPoint.LapCnt =
      mvdc_path_matching_Y->ActualTargetTrajectory.LapCnt;
    mvdc_path_matching_Y->ActualTrajectoryPoint.TrajCnt =
      mvdc_path_matching_Y->ActualTargetTrajectory.TrajCnt;
    mvdc_path_matching_Y->ActualTrajectoryPoint.s_loc_m =
      mvdc_path_matching_Y->PathPos_i.s_m;
    mvdc_path_matching_Y->ActualTrajectoryPoint.s_glob_m =
      mvdc_path_matching_interp1_k
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.s_glob_m,
       mvdc_path_matching_Y->PathPos_i.s_m);
    mvdc_path_matching_Y->ActualTrajectoryPoint.x_m =
      mvdc_path_matching_interp1_k
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.x_m,
       mvdc_path_matching_Y->PathPos_i.s_m);
    mvdc_path_matching_Y->ActualTrajectoryPoint.y_m =
      mvdc_path_matching_interp1_k
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.y_m,
       mvdc_path_matching_Y->PathPos_i.s_m);
    c_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[49];
    i = 1;
    do {
      exitg1 = 0;
      if (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i] < work) {
        i++;
        if (i + 1 > 50) {
          mvdc_path_matching_Y->ActualTrajectoryPoint.psi_rad =
            mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[49];
          exitg1 = 1;
        }
      } else {
        tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i - 1];
        b_tmp2 = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i] - tmp2;
        if (b_tmp2 > 3.1415926535897931) {
          tmp_0[0] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i - 1];
          tmp_0[1] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i];
          tmp_1[0] = tmp2;
          tmp_1[1] = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i] -
            6.2831853071795862;
          tmp2 = mvdc_path_matching_interp1_k4(tmp_0, tmp_1, work, c_tmp2);
        } else if (b_tmp2 < -3.1415926535897931) {
          tmp_0[0] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i - 1];
          tmp_0[1] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i];
          tmp_1[0] = tmp2;
          tmp_1[1] = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i] +
            6.2831853071795862;
          tmp2 = mvdc_path_matching_interp1_k4(tmp_0, tmp_1, work, c_tmp2);
        } else {
          tmp_0[0] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i - 1];
          tmp_0[1] = mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[i];
          tmp_1[0] = tmp2;
          tmp_1[1] = mvdc_path_matching_Y->ActualTargetTrajectory.psi_rad[i];
          tmp2 = mvdc_path_matching_interp1_k4(tmp_0, tmp_1, work, c_tmp2);
        }

        mvdc_path_matching_Y->ActualTrajectoryPoint.psi_rad =
          mvdc_path_matching_normalizeAngle(tmp2);
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    mvdc_path_matching_Y->ActualTrajectoryPoint.kappa_radpm =
      mvdc_path_matching_interp1_k4t
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.kappa_radpm,
       mvdc_path_matching_Y->PathPos_i.s_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.kappa_radpm[49]);
    std::memcpy(&diff_x_m[0],
                &mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0], 50U *
                sizeof(real_T));
    std::memcpy(&dist_squared_trialpoints[0],
                &mvdc_path_matching_Y->ActualTargetTrajectory.v_mps[0], 50U *
                sizeof(real_T));
    if (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[1] <
        mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0]) {
      for (i = 0; i < 25; i++) {
        tmp2 = diff_x_m[i];
        diff_x_m[i] = diff_x_m[49 - i];
        diff_x_m[49 - i] = tmp2;
      }

      std::memcpy(&dist_squared_trialpoints[0],
                  &mvdc_path_matching_Y->ActualTargetTrajectory.v_mps[0], 50U *
                  sizeof(real_T));
      for (i = 0; i < 25; i++) {
        tmp2 = dist_squared_trialpoints[i];
        dist_squared_trialpoints[i] = dist_squared_trialpoints[49 - i];
        dist_squared_trialpoints[49 - i] = tmp2;
      }
    }

    mvdc_path_matching_Y->ActualTrajectoryPoint.v_mps = 0.0;
    tmp = rtIsNaN(mvdc_path_matching_Y->PathPos_i.s_m);
    if (tmp) {
      mvdc_path_matching_Y->ActualTrajectoryPoint.v_mps = (rtNaN);
    } else if ((!(mvdc_path_matching_Y->PathPos_i.s_m > diff_x_m[49])) &&
               (!(mvdc_path_matching_Y->PathPos_i.s_m < diff_x_m[0]))) {
      i = 1;
      low_ip1 = 2;
      high_i = 50;
      while (high_i > low_ip1) {
        mid_i = (i + high_i) >> 1;
        if (work >= diff_x_m[mid_i - 1]) {
          i = mid_i;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }

      tmp2 = diff_x_m[i - 1];
      tmp2 = (mvdc_path_matching_Y->PathPos_i.s_m - tmp2) / (diff_x_m[i] - tmp2);
      if (tmp2 == 0.0) {
        mvdc_path_matching_Y->ActualTrajectoryPoint.v_mps =
          dist_squared_trialpoints[i - 1];
      } else if (tmp2 == 1.0) {
        mvdc_path_matching_Y->ActualTrajectoryPoint.v_mps =
          dist_squared_trialpoints[i];
      } else if (dist_squared_trialpoints[i - 1] == dist_squared_trialpoints[i])
      {
        mvdc_path_matching_Y->ActualTrajectoryPoint.v_mps =
          dist_squared_trialpoints[i - 1];
      } else {
        mvdc_path_matching_Y->ActualTrajectoryPoint.v_mps = (1.0 - tmp2) *
          dist_squared_trialpoints[i - 1] + tmp2 * dist_squared_trialpoints[i];
      }
    }

    mvdc_path_matching_Y->ActualTrajectoryPoint.banking_rad =
      mvdc_path_matching_interp1_k4t
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.banking_rad,
       mvdc_path_matching_Y->PathPos_i.s_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.banking_rad[49]);
    mvdc_path_matching_Y->ActualTrajectoryPoint.ax_lim_mps2 =
      mvdc_path_matching_interp1_k4t
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.ax_lim_mps2,
       mvdc_path_matching_Y->PathPos_i.s_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.ax_lim_mps2[49]);
    mvdc_path_matching_Y->ActualTrajectoryPoint.ay_lim_mps2 =
      mvdc_path_matching_interp1_k4t
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.ay_lim_mps2,
       mvdc_path_matching_Y->PathPos_i.s_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.ay_lim_mps2[49]);
    mvdc_path_matching_Y->ActualTrajectoryPoint.tube_r_m =
      mvdc_path_matching_interp1_k4t
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.tube_r_m,
       mvdc_path_matching_Y->PathPos_i.s_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.tube_r_m[49]);
    mvdc_path_matching_Y->ActualTrajectoryPoint.tube_l_m =
      mvdc_path_matching_interp1_k4t
      (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.tube_l_m,
       mvdc_path_matching_Y->PathPos_i.s_m,
       mvdc_path_matching_Y->ActualTargetTrajectory.tube_l_m[49]);
    std::memcpy(&diff_x_m[0],
                &mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0], 50U *
                sizeof(real_T));
    std::memcpy(&dist_squared_trialpoints[0],
                &mvdc_path_matching_Y->ActualTargetTrajectory.ax_mps2[0], 50U *
                sizeof(real_T));
    if (mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[1] <
        mvdc_path_matching_Y->ActualTargetTrajectory.s_loc_m[0]) {
      for (i = 0; i < 25; i++) {
        tmp2 = diff_x_m[i];
        diff_x_m[i] = diff_x_m[49 - i];
        diff_x_m[49 - i] = tmp2;
      }

      std::memcpy(&dist_squared_trialpoints[0],
                  &mvdc_path_matching_Y->ActualTargetTrajectory.ax_mps2[0], 50U *
                  sizeof(real_T));
      for (low_ip1 = 0; low_ip1 < 25; low_ip1++) {
        tmp2 = dist_squared_trialpoints[low_ip1];
        dist_squared_trialpoints[low_ip1] = dist_squared_trialpoints[49 -
          low_ip1];
        dist_squared_trialpoints[49 - low_ip1] = tmp2;
      }
    }

    mvdc_path_matching_Y->ActualTrajectoryPoint.ax_mps2 = 0.0;
    if (tmp) {
      mvdc_path_matching_Y->ActualTrajectoryPoint.ax_mps2 = (rtNaN);
    } else if (mvdc_path_matching_Y->PathPos_i.s_m == diff_x_m[49]) {
      mvdc_path_matching_Y->ActualTrajectoryPoint.ax_mps2 =
        dist_squared_trialpoints[49];
    } else if ((!(mvdc_path_matching_Y->PathPos_i.s_m > diff_x_m[49])) &&
               (!(mvdc_path_matching_Y->PathPos_i.s_m < diff_x_m[0]))) {
      i = 1;
      low_ip1 = 2;
      high_i = 50;
      while (high_i > low_ip1) {
        mid_i = (i + high_i) >> 1;
        if (work >= diff_x_m[mid_i - 1]) {
          i = mid_i;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }

      mvdc_path_matching_Y->ActualTrajectoryPoint.ax_mps2 =
        dist_squared_trialpoints[i - 1];
    }

    if (i_0 < 0) {
      i_0 = 0;
    } else if (i_0 > 65535) {
      i_0 = 65535;
    }

    mvdc_path_matching_Y->ActualTrajectoryPoint.PointIdx = static_cast<uint16_T>
      (i_0);

    // End of MATLAB Function: '<S1>/MATLAB Function'

    // Product: '<S1>/Multiply' incorporates:
    //   Outport: '<Root>/PathPos'
    //   Sum: '<S1>/Sum'
    //   Trigonometry: '<S1>/Sin'

    mvdc_path_matching_B->Multiply = std::sin
      (mvdc_path_matching_Y->PathPos_i.psi_rad +
       mvdc_path_matching_U->VehicleDynamicState_g.beta_rad) *
      mvdc_path_matching_U->VehicleDynamicState_g.v_mps;
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Outport: '<Root>/ActualTrajectoryPoint'
  //   UnaryMinus: '<Root>/Unary Minus'

  mvdc_path_matching_Y->debug.ActualTrajPoint =
    mvdc_path_matching_Y->ActualTrajectoryPoint;
  mvdc_path_matching_Y->debug.ActualTrajPoint.tube_r_m =
    -mvdc_path_matching_Y->ActualTrajectoryPoint.tube_r_m;

  // BusCreator generated from: '<Root>/debug' incorporates:
  //   Outport: '<Root>/debug'

  mvdc_path_matching_Y->debug.PerformanceTrajOK_b = rtb_traj_ok;
  mvdc_path_matching_Y->debug.EmergencyTrajOK_b = rtb_traj_ok_j;
  mvdc_path_matching_Y->debug.PathPosData = mvdc_path_matching_Y->PathPos_i;

  // Truth Table: '<Root>/Truth Table' incorporates:
  //   Outport: '<Root>/ActualTrajectoryPoint'

  //  Matching active
  //  Trajectory OK for matching
  //  End of trajectory reached
  rtb_traj_ok = (mvdc_path_matching_Y->ActualTrajectoryPoint.PointIdx == 50);

  //  High deviation to trajectory
  tmp2 = std::abs(mvdc_path_matching_B->d_m);
  if (tmp2 > mvdc_path_matching_P.P_VDC_LateralPathDeviationMax_m) {
    rtb_traj_ok_j = true;
  } else {
    rtb_traj_ok_j = (std::abs(mvdc_path_matching_B->Multiply /
      mvdc_path_matching_P.P_VDC_LateralPathDeviationDerMax_mps +
      mvdc_path_matching_B->d_m /
      mvdc_path_matching_P.P_VDC_LateralPathDeviationMax_m) > 1.0);
  }

  //  Very high deviation to trajectory
  if (tmp2 > 2.0 * mvdc_path_matching_P.P_VDC_LateralPathDeviationMax_m) {
    c = true;
  } else {
    c = (std::abs(mvdc_path_matching_B->Multiply /
                  mvdc_path_matching_P.P_VDC_LateralPathDeviationDerMax_mps +
                  mvdc_path_matching_B->d_m /
                  mvdc_path_matching_P.P_VDC_LateralPathDeviationMax_m) > 2.0);
  }

  tmp = (mvdc_path_matching_U->EnablePathMatching && rtb_LogicalOperator5);
  tmp_2 = (tmp && (!rtb_traj_ok));
  tmp_3 = !c;
  if (tmp_2 && (!rtb_traj_ok_j) && tmp_3) {
    // Outport: '<Root>/Status'
    //  valid
    mvdc_path_matching_Y->Status = PM_VALID;
  } else if (tmp_2 && rtb_traj_ok_j && c) {
    // Outport: '<Root>/Status'
    //  valid but very high deviation
    mvdc_path_matching_Y->Status = PM_VALID_VERYHIGHDEVIATION;
  } else if (mvdc_path_matching_U->EnablePathMatching && rtb_LogicalOperator5 &&
             (!rtb_traj_ok) && rtb_traj_ok_j && tmp_3) {
    // Outport: '<Root>/Status'
    //  valid but high deviation
    mvdc_path_matching_Y->Status = PM_VALID_HIGHDEVIATION;
  } else if (tmp && rtb_traj_ok) {
    // Outport: '<Root>/Status'
    //  end of trajectory reached
    mvdc_path_matching_Y->Status = PM_ENDREACHED;
  } else if (mvdc_path_matching_U->EnablePathMatching && (!rtb_LogicalOperator5))
  {
    // Outport: '<Root>/Status'
    //  not valid
    mvdc_path_matching_Y->Status = PM_NOTVALID;
  } else {
    // Outport: '<Root>/Status'
    //  Default
    //  not active
    mvdc_path_matching_Y->Status = PM_OFF;
  }

  // End of Truth Table: '<Root>/Truth Table'

  // Update for UnitDelay: '<S5>/Delay Input1'
  //
  //  Block description for '<S5>/Delay Input1':
  //
  //   Store in Global RAM

  mvdc_path_matching_DW->DelayInput1_DSTATE =
    mvdc_path_matching_U->EnableEmergency;
}

// Model initialize function
void mvdc_path_matching_initialize(RT_MODEL_mvdc_path_matching_T *const
  mvdc_path_matching_M)
{
  B_mvdc_path_matching_T *mvdc_path_matching_B = mvdc_path_matching_M->blockIO;
  DW_mvdc_path_matching_T *mvdc_path_matching_DW = mvdc_path_matching_M->dwork;
  ExtY_mvdc_path_matching_T *mvdc_path_matching_Y =
    static_cast<ExtY_mvdc_path_matching_T *>(mvdc_path_matching_M->outputs);

  // InitializeConditions for UnitDelay: '<S5>/Delay Input1'
  //
  //  Block description for '<S5>/Delay Input1':
  //
  //   Store in Global RAM

  mvdc_path_matching_DW->DelayInput1_DSTATE =
    mvdc_path_matching_P.DetectChange_vinit;

  // InitializeConditions for Delay: '<S2>/Delay2'
  mvdc_path_matching_DW->Delay2_DSTATE =
    mvdc_path_matching_P.Delay2_InitialCondition;

  // InitializeConditions for Switch: '<Root>/Switch3' incorporates:
  //   Delay: '<Root>/Delay2'

  mvdc_path_matching_DW->Delay2_DSTATE_j =
    mvdc_path_matching_P.Delay2_InitialCondition_c;

  // SystemInitialize for Enabled SubSystem: '<Root>/Subsystem'
  // SystemInitialize for Outport: '<Root>/PathPos' incorporates:
  //   Outport: '<S1>/PathPos'

  mvdc_path_matching_Y->PathPos_i = mvdc_path_matching_P.PathPos_Y0;

  // SystemInitialize for Outport: '<Root>/ActualTrajectoryPoint' incorporates:
  //   Outport: '<S1>/ActualTrajectoryPoint'

  mvdc_path_matching_Y->ActualTrajectoryPoint =
    mvdc_path_matching_P.ActualTrajectoryPoint_Y0;

  // SystemInitialize for SignalConversion generated from: '<S1>/d_m' incorporates:
  //   Outport: '<S1>/d_m'

  mvdc_path_matching_B->d_m = mvdc_path_matching_P.d_m_Y0;

  // SystemInitialize for Product: '<S1>/Multiply' incorporates:
  //   Outport: '<S1>/d_dot_mps'

  mvdc_path_matching_B->Multiply = mvdc_path_matching_P.d_dot_mps_Y0;

  // End of SystemInitialize for SubSystem: '<Root>/Subsystem'
}

// Model terminate function
void mvdc_path_matching_terminate(RT_MODEL_mvdc_path_matching_T
  * mvdc_path_matching_M)
{
  // model code
  rt_FREE(mvdc_path_matching_M->blockIO);
  rt_FREE(mvdc_path_matching_M->inputs);
  rt_FREE(mvdc_path_matching_M->outputs);
  rt_FREE(mvdc_path_matching_M->dwork);
  delete mvdc_path_matching_M;
}

// Model data allocation function
RT_MODEL_mvdc_path_matching_T *mvdc_path_matching(void)
{
  RT_MODEL_mvdc_path_matching_T *mvdc_path_matching_M;
  mvdc_path_matching_M = new RT_MODEL_mvdc_path_matching_T();
  if (mvdc_path_matching_M == (NULL)) {
    return (NULL);
  }

  // block I/O
  {
    B_mvdc_path_matching_T *b = (B_mvdc_path_matching_T *) malloc(sizeof
      (B_mvdc_path_matching_T));
    rt_VALIDATE_MEMORY(mvdc_path_matching_M,b);
    mvdc_path_matching_M->blockIO = (b);
  }

  // states (dwork)
  {
    DW_mvdc_path_matching_T *dwork = static_cast<DW_mvdc_path_matching_T *>
      (malloc(sizeof(DW_mvdc_path_matching_T)));
    rt_VALIDATE_MEMORY(mvdc_path_matching_M,dwork);
    mvdc_path_matching_M->dwork = (dwork);
  }

  // external inputs
  {
    ExtU_mvdc_path_matching_T *mvdc_path_matching_U =
      static_cast<ExtU_mvdc_path_matching_T *>(malloc(sizeof
      (ExtU_mvdc_path_matching_T)));
    rt_VALIDATE_MEMORY(mvdc_path_matching_M,mvdc_path_matching_U);
    mvdc_path_matching_M->inputs = ((static_cast<ExtU_mvdc_path_matching_T *>
      (mvdc_path_matching_U)));
  }

  // external outputs
  {
    ExtY_mvdc_path_matching_T *mvdc_path_matching_Y =
      static_cast<ExtY_mvdc_path_matching_T *>(malloc(sizeof
      (ExtY_mvdc_path_matching_T)));
    rt_VALIDATE_MEMORY(mvdc_path_matching_M,mvdc_path_matching_Y);
    mvdc_path_matching_M->outputs = (mvdc_path_matching_Y);
  }

  // Initialize DataMapInfo substructure containing ModelMap for C API
  mvdc_path_matching_InitializeDataMapInfo(mvdc_path_matching_M);

  {
    B_mvdc_path_matching_T *mvdc_path_matching_B = mvdc_path_matching_M->blockIO;
    DW_mvdc_path_matching_T *mvdc_path_matching_DW = mvdc_path_matching_M->dwork;
    ExtU_mvdc_path_matching_T *mvdc_path_matching_U =
      static_cast<ExtU_mvdc_path_matching_T *>(mvdc_path_matching_M->inputs);
    ExtY_mvdc_path_matching_T *mvdc_path_matching_Y =
      static_cast<ExtY_mvdc_path_matching_T *>(mvdc_path_matching_M->outputs);

    // initialize non-finites
    rt_InitInfAndNaN(sizeof(real_T));

    // block I/O
    (void) std::memset((static_cast<void *>(mvdc_path_matching_B)), 0,
                       sizeof(B_mvdc_path_matching_T));

    // states (dwork)
    (void) std::memset(static_cast<void *>(mvdc_path_matching_DW), 0,
                       sizeof(DW_mvdc_path_matching_T));

    // external inputs
    (void)std::memset(mvdc_path_matching_U, 0, sizeof(ExtU_mvdc_path_matching_T));

    // external outputs
    (void)std::memset(mvdc_path_matching_Y, 0, sizeof(ExtY_mvdc_path_matching_T));
  }

  return mvdc_path_matching_M;
}

const char_T* RT_MODEL_mvdc_path_matching_T::getErrorStatus() const
{
  return (errorStatus);
}

void RT_MODEL_mvdc_path_matching_T::setErrorStatus(const char_T* const volatile
  aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

RT_MODEL_mvdc_path_matching_T::DataMapInfo_T RT_MODEL_mvdc_path_matching_T::
  getDataMapInfo() const
{
  return DataMapInfo;
}

void RT_MODEL_mvdc_path_matching_T::setDataMapInfo(RT_MODEL_mvdc_path_matching_T::
  DataMapInfo_T aDataMapInfo)
{
  DataMapInfo = aDataMapInfo;
}

//
// File trailer for generated code.
//
// [EOF]
//
