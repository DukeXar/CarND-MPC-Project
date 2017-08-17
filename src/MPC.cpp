#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <type_traits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

// Helper wrapper to not do index math everywhere.
template <typename T>
class VarsView {
 public:
  typedef typename std::conditional<
      std::is_const<T>::value,
      typename std::add_const<typename T::value_type>::type,
      typename std::remove_const<typename T::value_type>::type>::type
      value_type;

  // Constructs the view, wrapping container v starting at offset idx
  VarsView(T& v, size_t idx, size_t nSteps)
      : m_v(v), m_idx(idx), m_nSteps(nSteps) {}

  value_type& x() { return m_v[x_start + m_idx]; }
  value_type& y() { return m_v[y_start + m_idx]; }
  value_type& psi() { return m_v[psi_start + m_idx]; }
  value_type& v() { return m_v[v_start + m_idx]; }
  value_type& cte() { return m_v[cte_start + m_idx]; }
  value_type& psie() { return m_v[psie_start + m_idx]; }

  value_type& steer() { return m_v[steer_start + m_idx]; }
  value_type& acc() { return m_v[acc_start + m_idx]; }

 private:
  T& m_v;
  const size_t m_idx;
  const size_t m_nSteps;

  const size_t x_start = 0;
  const size_t y_start = x_start + m_nSteps;
  const size_t psi_start = y_start + m_nSteps;
  const size_t v_start = psi_start + m_nSteps;
  const size_t cte_start = v_start + m_nSteps;
  const size_t psie_start = cte_start + m_nSteps;
  const size_t vars_end = psie_start + m_nSteps;

  const size_t steer_start = vars_end;
  const size_t acc_start = steer_start + (m_nSteps - 1);
  const size_t act_end = acc_start + (m_nSteps - 1);
};

namespace {

template <typename T>
VarsView<T> GetView(T& v, size_t idx, size_t nSteps) {
  return VarsView<T>(v, idx, nSteps);
}

}  // namespace

// A functor which is used by ipopt solver to estimate actuators' forces based
// on input measurements and car motion model.
class FG_eval {
 private:
  template <typename T>
  VarsView<T> GetView(T& v, size_t idx) {
    return ::GetView(v, idx, m_nSteps);
  }

 public:
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // Constructs a functor.
  // @param coeffs are 3rd-order polynomial coefficients of the target
  //               trajectory function.
  // For other parameters see MPC.
  FG_eval(Eigen::VectorXd coeffs, size_t nSteps, double refV, double stepDt,
          const Penalties& penalties, double latency)
      : m_coeffs(coeffs),
        m_nSteps(nSteps),
        m_refV(refV),
        m_stepDt(stepDt),
        m_penalties(penalties),
        m_latency(latency) {}

  // Interface for ipopt.
  // @param fg is a vector of the cost constraints.
  // @param vars is a vector of variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    AD<double> cost = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < m_nSteps; ++t) {
      auto view = GetView(vars, t);
      cost += m_penalties.cte * CppAD::pow(view.cte(), 2);
      cost += m_penalties.psie * CppAD::pow(view.psie(), 2);
      AD<double> dv = view.v() - m_refV;
      cost += m_penalties.v * CppAD::pow(dv, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < m_nSteps - 1; ++t) {
      auto view = GetView(vars, t);
      cost += m_penalties.steer * CppAD::pow(view.steer(), 2);
      cost += m_penalties.acc * CppAD::pow(view.acc(), 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < m_nSteps - 2; ++t) {
      auto view0 = GetView(vars, t);
      auto view1 = GetView(vars, t + 1);
      AD<double> dsteer = view0.steer() - view1.steer();
      cost += m_penalties.steer_gap * CppAD::pow(dsteer, 2);
      AD<double> dacc = view0.acc() - view1.acc();
      cost += m_penalties.acc_gap * CppAD::pow(dacc, 2);
    }

    fg[0] = cost;

    {
      auto viewfg = GetView(fg, 1);
      auto viewv = GetView(vars, 0);
      viewfg.x() = viewv.x();
      viewfg.y() = viewv.y();
      viewfg.psi() = viewv.psi();
      viewfg.v() = viewv.v();
      viewfg.cte() = viewv.cte();
      viewfg.psie() = viewv.psie();
    }

    // Evaluate the model into m_nSteps in future.

    for (int t = 1; t < m_nSteps; ++t) {
      const double dt = m_stepDt;

      auto view0 = GetView(vars, t - 1);
      auto view1 = GetView(vars, t);
      auto viewfg = GetView(fg, t + 1);

      AD<double> f0 = m_coeffs[0] + m_coeffs[1] * view0.x() +
                      m_coeffs[2] * view0.x() * view0.x() +
                      m_coeffs[3] * view0.x() * view0.x() * view0.x();
      AD<double> psides0 =
          CppAD::atan(m_coeffs[1] + 2 * m_coeffs[2] * view0.x() +
                      3 * m_coeffs[3] * view0.x() * view0.x());

      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      viewfg.x() =
          view1.x() - (view0.x() + view0.v() * CppAD::cos(view0.psi()) * dt);

      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      viewfg.y() =
          view1.y() - (view0.y() + view0.v() * CppAD::sin(view0.psi()) * dt);

      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      viewfg.psi() =
          view1.psi() - (view0.psi() + view0.v() * view0.steer() / Lf * dt);

      // v_[t+1] = v[t] + a[t] * dt
      viewfg.v() = view1.v() - (view0.v() + view0.acc() * dt);

      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      viewfg.cte() = view1.cte() - (f0 - view0.y() +
                                    view0.v() * CppAD::sin(view0.psie()) * dt);

      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      viewfg.psie() = view1.psie() - ((view0.psi() - psides0) +
                                      view0.v() * view0.steer() / Lf * dt);
    }
  }

 private:
  const Eigen::VectorXd m_coeffs;
  const size_t m_nSteps;
  const double m_refV;
  const double m_stepDt;
  const Penalties m_penalties;
  const double m_latency;
};

MPC::Result MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  const size_t n_vars = N_STATE * m_nSteps + N_ACTUATORS * (m_nSteps - 1);
  const size_t n_constraints = N_STATE * m_nSteps;

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double psie = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }

  auto varsView = GetView(vars, 0, m_nSteps);
  varsView.x() = x;
  varsView.y() = y;
  varsView.psi() = psi;
  varsView.v() = v;
  varsView.cte() = cte;
  varsView.psie() = psie;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Variables limits, just fill everything and fix actuators later.
  for (int i = 0; i < n_vars; ++i) {
    // NOTE: Don't set to std::numeric_limits<double>::min()/max();
    vars_lowerbound[i] = -1e19;
    vars_upperbound[i] = 1e19;
  }

  // Actuators limits.
  for (int i = 0; i < (m_nSteps - 1); ++i) {
    GetView(vars_lowerbound, i, m_nSteps).steer() = -MAX_STEER_IN_RAD;
    GetView(vars_upperbound, i, m_nSteps).steer() = MAX_STEER_IN_RAD;
    GetView(vars_lowerbound, i, m_nSteps).acc() = -1.0;
    GetView(vars_upperbound, i, m_nSteps).acc() = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Simply fix original values.
  auto clbView = GetView(constraints_lowerbound, 0, m_nSteps);
  auto cubView = GetView(constraints_upperbound, 0, m_nSteps);
  clbView.x() = x;
  cubView.x() = x;
  clbView.y() = y;
  cubView.y() = y;
  clbView.psi() = psi;
  cubView.psi() = psi;
  clbView.v() = v;
  cubView.v() = v;
  clbView.cte() = cte;
  cubView.cte() = cte;
  clbView.psie() = psie;
  cubView.psie() = psie;

  /*
  std::cout << "Vars lowerbound: " << vars_lowerbound << std::endl;
  std::cout << "Vars upperbound: " << vars_upperbound << std::endl;
  std::cout << "Consts lowerbound: " << constraints_lowerbound << std::endl;
  std::cout << "Consts upperbound: " << constraints_upperbound << std::endl;
  std::cout << "Coeffs: " << coeffs << std::endl;
  */

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, m_nSteps, m_refV, m_stepDt, m_penalties, m_latency);

  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    std::cout << "Solution status is not success! Ouch, how it could be? 99% "
                 "you have a bug in your code. status="
              << solution.status << std::endl;
  }

  // const auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  std::vector<double> resX, resY;
  for (size_t i = 0; i < m_nSteps; ++i) {
    resX.push_back(GetView(solution.x, i, m_nSteps).x());
    resY.push_back(GetView(solution.x, i, m_nSteps).y());
  }

  auto actView = GetView(solution.x, 0, m_nSteps);

  // std::cout << "Result: " << actView.steer() << ", " << actView.acc()
  //          << std::endl;

  return Result{resX, resY, actView.steer(), actView.acc()};
}