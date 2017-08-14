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

// The following two are really hardcoded, as they are tightly coupled with
// model functions.
// Number of actuators.
const size_t N_ACTUATORS = 2;
// Number of state variables.
const size_t N_STATE = 6;

// 25 deg in rad
const double MAX_STEER_IN_RAD = 0.436332;

typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

namespace {

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
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
double Polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double PolyevalDeriv(Eigen::VectorXd coeffs, double x) {
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
  return std::make_pair(lx, ly);
}
}  // namespace

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
  typedef ADvector ADvector;

  // Constructs a functor.
  // @param coeffs are 3rd-order polynomial coefficients of the target
  //               trajectory function.
  // @param nSteps is number of steps to calculate into future.
  // @param refV is a reference velocity of a car.
  // @param stepDt is a time delta in seconds between steps.
  FG_eval(Eigen::VectorXd coeffs, size_t nSteps, double refV, double stepDt,
          const Penalties& penalties)
      : m_coeffs(coeffs),
        m_nSteps(nSteps),
        m_refV(refV),
        m_stepDt(stepDt),
        m_penalties(penalties) {}

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
      viewfg.x() = view1.x() -
                   (view0.x() + view0.v() * CppAD::cos(view0.psi()) * m_stepDt);

      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      viewfg.y() = view1.y() -
                   (view0.y() + view0.v() * CppAD::sin(view0.psi()) * m_stepDt);

      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      viewfg.psi() = view1.psi() -
                     (view0.psi() + view0.v() * view0.steer() / Lf * m_stepDt);

      // v_[t+1] = v[t] + a[t] * dt
      viewfg.v() = view1.v() - (view0.v() + view0.acc() * m_stepDt);

      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      viewfg.cte() =
          view1.cte() -
          (f0 - view0.y() + view0.v() * CppAD::sin(view0.psie()) * m_stepDt);

      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      viewfg.psie() =
          view1.psie() -
          ((view0.psi() - psides0) + view0.v() * view0.steer() / Lf * m_stepDt);
    }
  }

 private:
  const Eigen::VectorXd m_coeffs;
  const size_t m_nSteps;
  const double m_refV;
  const double m_stepDt;
  const Penalties m_penalties;
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
  FG_eval fg_eval(coeffs, m_nSteps, m_refV, m_stepDt, m_penalties);

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

Navigator::Navigator(double refV, double stepDt, size_t nSteps,
                     const Penalties& penalties)
    : m_stepDt(stepDt),
      m_nSteps(nSteps),
      m_mpc(refV, stepDt, nSteps, penalties) {}

void Navigator::Update(const std::vector<double>& ptsx,
                       const std::vector<double>& ptsy, double px, double py,
                       double psi, double speed) {
  // Convert from global coorinate system to local (car's) one to simplify
  // further calculations.
  Eigen::VectorXd ptsxLocal(ptsx.size());
  Eigen::VectorXd ptsyLocal(ptsy.size());

  for (int i = 0; i < ptsx.size(); ++i) {
    const auto xy = GlobalToLocal(ptsx[i], ptsy[i], px, py, psi);
    ptsxLocal[i] = xy.first;
    ptsyLocal[i] = xy.second;
  }

  // Fit a polinomial for desired vaypoints.
  const auto coeffs = Polyfit(ptsxLocal, ptsyLocal, 3);
  // As we are in local coordinate system, cross track error is just f(0).
  const double cte = Polyeval(coeffs, 0);
  // psie = psi - angle at 0. psi in local coordinates is just 0
  const double psie = 0 - atan(PolyevalDeriv(coeffs, 0));
  Eigen::VectorXd state(N_STATE);
  state << 0, 0, 0, speed, cte, psie;

  m_result = m_mpc.Solve(state, coeffs);

  // This is how we planned the car should drive.
  m_nextX = std::vector<double>(m_nSteps, 0);
  m_nextY = m_nextX;
  for (size_t i = 0; i < m_nextX.size(); ++i) {
    m_nextX[i] = i * speed * m_stepDt;
    m_nextY[i] = Polyeval(coeffs, m_nextX[i]);
  }
}

std::vector<double> Navigator::GetMpcXVals() const { return m_result.x; }

std::vector<double> Navigator::GetMpcYVals() const { return m_result.y; }

std::vector<double> Navigator::GetNextXVals() const { return m_nextX; }

std::vector<double> Navigator::GetNextYVals() const { return m_nextY; }

double Navigator::GetSteerValue() const { return m_result.steer; }

double Navigator::GetThrottleValue() const { return m_result.acc; }