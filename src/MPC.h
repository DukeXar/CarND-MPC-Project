#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  // const size_t m_n = 5;
  // const double m_refV = 50;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

class Navigator {
 public:
  void Update(const std::vector<double> &ptsx, const std::vector<double> &ptsy,
              double px, double py, double psi, double speed);

  std::vector<double> GetMpcXVals() const;
  std::vector<double> GetMpcYVals() const;
  std::vector<double> GetNextXVals() const;
  std::vector<double> GetNextYVals() const;

  double GetSteerValue() const;
  double GetThrottleValue() const;

 private:
  MPC m_mpc;
  std::vector<double> m_result;
  std::vector<double> m_nextX;
  std::vector<double> m_nextY;
};

#endif /* MPC_H */
