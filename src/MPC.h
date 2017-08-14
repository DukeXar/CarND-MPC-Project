#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  struct Result {
    // Predicted trajectory, x coordinates in car's local coordinates.
    std::vector<double> x;
    // Predicted trajectory, y coordinates in car's local coordinates.
    std::vector<double> y;
    // Actuator value for steering. In range [-deg2rad(25), deg2rad(25)].
    double steer;
    // Actuator value for acceleration. In range [-1, 1].
    double acc;
  };

  // Constructs MPC evaluator.
  // @param refV is reference velocity.
  // @param stepDt is time delta between steps, in seconds.
  // @param nSteps is number of steps to evaluate.
  MPC(double refV, double stepDt, size_t nSteps)
      : m_refV(refV), m_stepDt(stepDt), m_nSteps(nSteps) {}

  // Solve the model given an initial state and polynomial coefficients.
  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

 private:
  const double m_refV;
  const double m_stepDt;
  const size_t m_nSteps;
};

class Navigator {
 public:
  Navigator(double refV, double stepDt, size_t nSteps);

  void Update(const std::vector<double> &ptsx, const std::vector<double> &ptsy,
              double px, double py, double psi, double speed);

  std::vector<double> GetMpcXVals() const;
  std::vector<double> GetMpcYVals() const;
  std::vector<double> GetNextXVals() const;
  std::vector<double> GetNextYVals() const;

  double GetSteerValue() const;
  double GetThrottleValue() const;

 private:
  double m_stepDt;
  size_t m_nSteps;
  MPC m_mpc;
  MPC::Result m_result;
  std::vector<double> m_nextX;
  std::vector<double> m_nextY;
};

#endif /* MPC_H */
