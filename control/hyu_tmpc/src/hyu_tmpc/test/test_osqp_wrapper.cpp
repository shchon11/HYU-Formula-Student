#include <gtest/gtest.h>

#include <cmath>

extern "C" {
#include "osqp_wrapper.h"
}

namespace
{

// Tiny QP: minimize x^2 subject to constraint rows selecting x. n=1, m=2 lets
// the two rows contradict each other WITHOUT any single l>u pair, which is the
// primal-infeasible shape the tube constraints take when the car sits outside
// the tube (the stock wrapper's failure mode under test).
class OsqpWrapperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // init_osqp_wrapper parameter layout (see the s-function call in
    // mvdc_mpc.cpp): p0=q, p1=&m, p2=&n, p3=P_x, p4=P_i, p5=P_p(n+1, P_p[n]=nnz),
    // p6=A_x, p7=A_i, p8=A_p(n+1, A_p[n]=nnz), p9=l, p10=u, p11=(unused),
    // p12=max_iter.
    q_[0] = 0.0;
    m_param_[0] = 2.0;
    n_param_[0] = 1.0;
    P_x_[0] = 2.0;
    P_i_[0] = 0.0;
    P_p_[0] = 0.0;
    P_p_[1] = 1.0;
    A_x_[0] = 1.0;
    A_x_[1] = 1.0;
    A_i_[0] = 0.0;
    A_i_[1] = 1.0;
    A_p_[0] = 0.0;
    A_p_[1] = 2.0;
    l_[0] = 1.0;
    l_[1] = 0.0;
    u_[0] = 2.0;
    u_[1] = 3.0;
    unused_[0] = 0.0;
    max_iter_[0] = 4000.0;

    init_osqp_wrapper(
      &wrapper_, q_, m_param_, n_param_, P_x_, P_i_, P_p_, A_x_, A_i_, A_p_,
      l_, u_, unused_, max_iter_);
    ASSERT_EQ(wrapper_.flag_setup, 0);
  }

  void TearDown() override {cleanup_osqp_wrapper(&wrapper_);}

  void Update(c_float l0, c_float u0, c_float l1, c_float u1)
  {
    c_float l[2] = {l0, l1};
    c_float u[2] = {u0, u1};
    c_float q[1] = {0.0};
    c_float a[2] = {1.0, 1.0};
    update_osqp_wrapper(&wrapper_, q, l, u, a);
  }

  double Solution() const {return wrapper_.work->solution->x[0];}

  osqp_wrapper wrapper_{};
  c_float q_[1], m_param_[1], n_param_[1];
  c_float P_x_[1], P_i_[1], P_p_[2];
  c_float A_x_[2], A_i_[2], A_p_[2];
  c_float l_[2], u_[2], unused_[1], max_iter_[1];
};

TEST_F(OsqpWrapperTest, FeasibleSolveIsAcceptedAndCached)
{
  // min x^2 s.t. 1 <= x <= 2 (both rows agree) -> x = 1.
  Update(1.0, 2.0, 0.0, 3.0);
  EXPECT_EQ(wrapper_.last_status_val, OSQP_SOLVED);
  EXPECT_NEAR(Solution(), 1.0, 1e-3);
  EXPECT_EQ(wrapper_.has_last_good, 1);
  EXPECT_EQ(wrapper_.consecutive_failures, 0);
}

TEST_F(OsqpWrapperTest, CrossedBoundsAreCollapsedInsteadOfSilentlyRejected)
{
  Update(1.0, 2.0, 0.0, 3.0);
  // Crossed pair l=3 > u=1 on row 0: the stock wrapper's osqp_update_bounds
  // rejected the whole update and solved LAST cycle's bounds. Sanitized to the
  // midpoint 2, the QP stays well-posed: x = 2.
  Update(3.0, 1.0, 0.0, 3.0);
  EXPECT_EQ(wrapper_.last_status_val, OSQP_SOLVED);
  EXPECT_NEAR(Solution(), 2.0, 1e-3);
}

TEST_F(OsqpWrapperTest, InfeasibleSolveIsBridgedThenExposedThenRecovers)
{
  Update(1.0, 2.0, 0.0, 3.0);
  const double good = Solution();

  // Rows contradict (x<=1 and x>=2.5) with no single crossed pair: primal
  // infeasible. Within the bridge window the last accepted solution must be
  // served instead of the diverging iterate.
  for (int i = 0; i < 10; ++i) {
    Update(0.0, 1.0, 2.5, 3.0);
    EXPECT_NE(wrapper_.last_status_val, OSQP_SOLVED) << "iteration " << i;
    EXPECT_NEAR(Solution(), good, 1e-6) << "bridge cycle " << i;
  }

  // Past the window the raw iterate is exposed on purpose (downstream guards
  // own sustained failure); it must at least be marked failed.
  Update(0.0, 1.0, 2.5, 3.0);
  EXPECT_GT(wrapper_.consecutive_failures, 10);
  EXPECT_NE(wrapper_.last_status_val, OSQP_SOLVED);

  // Feasibility returns: thanks to the cold restart the very next solve must
  // converge again (the stock wrapper kept iterating from the runaway state).
  Update(1.0, 2.0, 0.0, 3.0);
  EXPECT_EQ(wrapper_.last_status_val, OSQP_SOLVED);
  EXPECT_NEAR(Solution(), 1.0, 1e-3);
  EXPECT_EQ(wrapper_.consecutive_failures, 0);
}

}  // namespace
