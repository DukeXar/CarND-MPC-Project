#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <type_traits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Observation: when this is 5 and dt is 0.2, car steers outside of the lane and
// stays there, while driving parallel to the main road.
const double STEP_DT = 0.1;
const size_t N_STEPS = 10;
const double REF_V = 50;

// Set the number of model variables (includes both states and inputs).
// For example: If the state is a 4 element vector, the actuators is a 2
// element vector and there are 10 timesteps. The number of variables is:
// 4 * 10 + 2 * 9

const size_t n_state = 6;
const size_t n_vars = n_state * N_STEPS + 2 * (N_STEPS - 1);
const size_t n_constraints = n_state * N_STEPS;

const size_t x_start = 0;
const size_t y_start = x_start + N_STEPS;
const size_t psi_start = y_start + N_STEPS;
const size_t v_start = psi_start + N_STEPS;
const size_t cte_start = v_start + N_STEPS;
const size_t psie_start = cte_start + N_STEPS;
const size_t vars_end = psie_start + N_STEPS;

const size_t steer_start = vars_end;
const size_t acc_start = steer_start + (N_STEPS - 1);
const size_t act_end = acc_start + (N_STEPS - 1);

typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

namespace {

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double polyevalDeriv(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i - 1) * i;
  }
  return result;
}

std::pair<double, double> GlobalToLocal(double x, double y, double x0,
                                        double y0, double psi) {
  double tx = x - x0;
  double ty = y - y0;
  double lx = tx * cos(psi) - ty * sin(psi);
  double ly = tx * sin(psi) + ty * cos(psi);
  return make_pair(lx, ly);
}
}  // namespace

template <typename T>
class VarsView {
 public:
  typedef typename std::conditional<
      std::is_const<T>::value,
      typename std::add_const<typename T::value_type>::type,
      typename std::remove_const<typename T::value_type>::type>::type
      value_type;

  VarsView(T& v, size_t idx) : m_v(v), m_idx(idx) {}

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
};

class FG_eval {
 public:
  typedef ADvector ADvector;

  explicit FG_eval(Eigen::VectorXd coeffs, size_t nSteps, double refV)
      : m_coeffs(coeffs), m_nSteps(nSteps), m_refV(refV) {}

  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable
    // values (state & actuators)

    AD<double> cost = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < m_nSteps; ++t) {
      VarsView<const ADvector> view(vars, t);
      cost += CppAD::pow(view.cte(), 2);
      cost += CppAD::pow(view.psie(), 2);
      cost += CppAD::pow(view.v() - m_refV, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < m_nSteps - 1; ++t) {
      VarsView<const ADvector> view(vars, t);
      cost += CppAD::pow(view.steer(), 2);
      cost += CppAD::pow(view.acc(), 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < m_nSteps - 2; ++t) {
      VarsView<const ADvector> view0(vars, t);
      VarsView<const ADvector> view1(vars, t + 1);
      cost += CppAD::pow(view0.steer() - view1.steer(), 2);
      cost += CppAD::pow(view0.acc() - view1.acc(), 2);
    }

    fg[0] = cost;

    {
      VarsView<ADvector> viewfg(fg, 1);
      VarsView<const ADvector> viewv(vars, 0);
      viewfg.x() = viewv.x();
      viewfg.y() = viewv.y();
      viewfg.psi() = viewv.psi();
      viewfg.v() = viewv.v();
      viewfg.cte() = viewv.cte();
      viewfg.psie() = viewv.psie();
    }

    for (int t = 1; t < m_nSteps; ++t) {
      VarsView<const ADvector> view0(vars, t - 1);
      VarsView<const ADvector> view1(vars, t);
      VarsView<ADvector> viewfg(fg, t + 1);

      AD<double> f0 = m_coeffs[0] + m_coeffs[1] * view0.x() +
                      m_coeffs[2] * view0.x() * view0.x() +
                      m_coeffs[3] * view0.x() * view0.x() * view0.x();
      AD<double> psides0 =
          CppAD::atan(m_coeffs[1] + 2 * m_coeffs[2] * view0.x() +
                      3 * m_coeffs[3] * view0.x() * view0.x());

      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      viewfg.x() = view1.x() -
                   (view0.x() + view0.v() * CppAD::cos(view0.psi()) * STEP_DT);

      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      viewfg.y() = view1.y() -
                   (view0.y() + view0.v() * CppAD::sin(view0.psi()) * STEP_DT);

      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      viewfg.psi() = view1.psi() -
                     (view0.psi() + view0.v() * view0.steer() / Lf * STEP_DT);

      // v_[t+1] = v[t] + a[t] * dt
      viewfg.v() = view1.v() - (view0.v() + view0.acc() * STEP_DT);

      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      viewfg.cte() =
          view1.cte() -
          (f0 - view0.y() + view0.v() * CppAD::sin(view0.psie()) * STEP_DT);

      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      viewfg.psie() = view1.psie() - ((view0.psi() - psides0) +
                                      view0.v() * view0.steer() / Lf * STEP_DT);
    }
  }

 private:
  const Eigen::VectorXd m_coeffs;
  size_t m_nSteps;
  double m_refV;
};

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // FIXME(dukexar): ???
  const size_t n_steps = N_STEPS;

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

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[psie_start] = psie;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Variables limits
  for (int i = 0; i < vars_end; ++i) {
    vars_lowerbound[i] = -1e19;  // std::numeric_limits<double>::min();
    vars_upperbound[i] = 1e19;   // std::numeric_limits<double>::max();
  }

  // Steering limits
  for (int i = 0; i < (n_steps - 1); ++i) {
    vars_lowerbound[steer_start + i] = -0.436332;  // -25 deg in rad
    vars_upperbound[steer_start + i] = 0.436332;   // +25 deg in rad
  }

  // Throttle/accelleration limits
  for (int i = 0; i < (n_steps - 1); ++i) {
    vars_lowerbound[acc_start + i] = -1.0;
    vars_upperbound[acc_start + i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Simply fix original values
  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[psie_start] = psie;
  constraints_upperbound[psie_start] = psie;

  /*
  std::cout << "Vars lowerbound: " << vars_lowerbound << std::endl;
  std::cout << "Vars upperbound: " << vars_upperbound << std::endl;
  std::cout << "Consts lowerbound: " << constraints_lowerbound << std::endl;
  std::cout << "Consts upperbound: " << constraints_upperbound << std::endl;
  std::cout << "Coeffs: " << coeffs << std::endl;
  */

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, n_steps, REF_V);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  1\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    // FIXME(dukexar): WTF?
    std::cout
        << "Solution status is not success! Ouch, how it could be? status="
        << solution.status << std::endl;
  }

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  const auto& sx = solution.x;
  std::cout << "Result: " << sx[steer_start] << ", " << sx[acc_start]
            << std::endl;
  return {sx[steer_start], sx[acc_start]};
}

void Navigator::Update(const std::vector<double>& ptsx,
                       const std::vector<double>& ptsy, double px, double py,
                       double psi, double speed) {
  Eigen::VectorXd ptsxLocal(ptsx.size());
  Eigen::VectorXd ptsyLocal(ptsy.size());

  for (int i = 0; i < ptsx.size(); ++i) {
    auto xy = GlobalToLocal(ptsx[i], ptsy[i], px, py, psi);
    ptsxLocal[i] = xy.first;
    ptsyLocal[i] = xy.second;
  }

  auto coeffs = polyfit(ptsxLocal, ptsyLocal, 3);
  double cte = polyeval(coeffs, 0);

  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
  double psie = -atan(polyevalDeriv(coeffs, 0));
  Eigen::VectorXd state(n_state);
  state << 0, 0, 0, speed, cte, psie;

  std::cout << "State: " << state << std::endl;

  m_result = m_mpc.Solve(state, coeffs);

  // std::cout << "Result: " << m_result << std::endl;

  // FIXME(dukexar): These should have the values you would get after
  // calculating polyeval on way points going forward.
  m_nextX = std::vector<double>(ptsxLocal.size());
  m_nextY = std::vector<double>(ptsyLocal.size());
  for (size_t i = 0; i < ptsxLocal.size(); ++i) {
    m_nextX[i] = ptsxLocal[i];
    m_nextY[i] = ptsyLocal[i];
  }
}

std::vector<double> Navigator::GetMpcXVals() const { return {}; }

std::vector<double> Navigator::GetMpcYVals() const { return {}; }

std::vector<double> Navigator::GetNextXVals() const { return m_nextX; }

std::vector<double> Navigator::GetNextYVals() const { return m_nextY; }

double Navigator::GetSteerValue() const { return -m_result[0]; }

double Navigator::GetThrottleValue() const { return m_result[1]; }