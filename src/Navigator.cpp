#include "Navigator.h"
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

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
  double lx = tx * cos(-psi) - ty * sin(-psi);
  double ly = tx * sin(-psi) + ty * cos(-psi);
  return std::make_pair(lx, ly);
}
}  // namespace

Navigator::Navigator(double refV, double stepDt, size_t nSteps,
                     const Penalties& penalties, double latency)
    : m_stepDt(stepDt),
      m_nSteps(nSteps),
      m_mpc(refV, stepDt, nSteps, penalties, latency),
      m_latency(latency) {}

void Navigator::Update(const std::vector<double>& ptsx,
                       const std::vector<double>& ptsy, double px, double py,
                       double psi, double speed, double steer,
                       double throttle) {
  // TODO(dukexar): Yes, it is silly to negate back, but we need to apply model
  // here.
  // steer = -steer;
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  const double corrX = px + speed * cos(psi) * m_latency;
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  const double corrY = py + speed * sin(psi) * m_latency;
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  const double corrPsi = psi + speed * steer / Lf * m_latency;
  // Assume that speed does not change much in m_latency time, as otherwise
  // there is no simple way to convert the throttle into acceleration.
  // const double corrSpeed = speed + acc * m_latency;
  const double corrSpeed = speed;

  std::cout << "Predicted corrX=" << corrX << std::endl;
  std::cout << "Predicted corrY=" << corrY << std::endl;
  std::cout << "Predicted corrPsi=" << corrPsi << std::endl;
  std::cout << "Predicted corrSpeed=" << corrSpeed << std::endl;

  // Convert from global coorinate system to local (car's) one to simplify
  // further calculations.
  Eigen::VectorXd ptsxLocal(ptsx.size());
  Eigen::VectorXd ptsyLocal(ptsy.size());

  for (int i = 0; i < ptsx.size(); ++i) {
    const auto xy = GlobalToLocal(ptsx[i], ptsy[i], corrX, corrY, corrPsi);
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
  state << 0, 0, 0, corrSpeed, cte, psie;

  m_result = m_mpc.Solve(state, coeffs);

  // This is how we planned the car should drive.
  m_nextX = std::vector<double>(m_nSteps, 0);
  m_nextY = m_nextX;
  for (size_t i = 0; i < m_nextX.size(); ++i) {
    m_nextX[i] = i * m_stepDt * 10;  // i * speed * m_stepDt;
    m_nextY[i] = Polyeval(coeffs, m_nextX[i]);
  }
}

std::vector<double> Navigator::GetMpcXVals() const { return m_result.x; }

std::vector<double> Navigator::GetMpcYVals() const { return m_result.y; }

std::vector<double> Navigator::GetNextXVals() const { return m_nextX; }

std::vector<double> Navigator::GetNextYVals() const { return m_nextY; }

double Navigator::GetSteerValue() const {
  return m_result.steer;
  // return 0.1;
  // return 0;
}

double Navigator::GetThrottleValue() const {
  return m_result.acc;
  // return 0.5;
}