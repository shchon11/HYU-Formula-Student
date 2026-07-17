#ifndef OSQP_WRAPPER_H
#define OSQP_WRAPPER_H

#include "time.h"
#include "header/osqp.h"

typedef struct osqp_wrapper{
    // problem configuration
    c_int n; // number of optimization variables
    c_int m; // number of constraints
    c_int P_nnz; // number of nonzero elements in P matrix
    c_int A_nnz; // number of nonzero elements in A matrix

    // OSQP data
    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;

    // output data, public to be accessible from s-function
    c_int flag_bound_upd;
    c_int flag_A_upd;
    c_int flag_P_upd;
    c_int flag_q_upd;
    c_int flag_solve;
    c_int flag_setup;

    // --- fail-closed guard state -------------------------------------------
    // Appended fields are safe: the generated model allocates this struct with
    // c_malloc(sizeof(osqp_wrapper)) against this same header. The stock
    // wrapper ignored every failure signal: osqp_update_bounds() REJECTS the
    // whole update when any l > u (the tube constraint crosses whenever the
    // car sits outside the tube), so the QP was silently solved with the
    // previous cycle's bounds, and osqp_solve()'s status was never read, so on
    // a primal-infeasible problem the diverging ADMM iterate (it grows along
    // an infeasibility certificate whose sign has nothing to do with the
    // drivable direction) was copied out as if it were a solution -- observed
    // as +-2.8 rad steering requests reported with an OK tube status.
    c_float* last_good_x;        // solution of the last accepted solve
    c_float* last_good_y;        // duals of the last accepted solve
    c_int has_last_good;
    c_int consecutive_failures;  // solves since the last accepted one
    c_int last_status_val;       // OSQPWorkspace info->status_val of last solve
} osqp_wrapper;

void init_osqp_wrapper(osqp_wrapper* wrapper,
    c_float *p_0, c_float *p_1, c_float *p_2, c_float *p_3,
  c_float *p_4, c_float *p_5, c_float *p_6, c_float *p_7,
c_float *p_8, c_float *p_9, c_float *p_10, c_float *p_11, c_float *p_13);

void cleanup_osqp_wrapper(osqp_wrapper* wrapper);

void restart_osqp_wrapper(osqp_wrapper* wrapper);

void update_osqp_wrapper(osqp_wrapper* wrapper, c_float* q_upd, c_float* l_upd,
    c_float* u_upd, c_float* A_x_upd);

#endif
