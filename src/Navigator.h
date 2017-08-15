#pragma once

#include "MPC.h"

#include <vector>

// Navigator is a wrapper over MPC that provides a shim between Udacity
// simulator measurements and actuators and MPC model.
class Navigator {
 public:
  // Constructs navigator.
  // @param refV is reference velocity.
  // @param stepDt is time delta between steps, in seconds.
  // @param nSteps is number of steps to evaluate.
  // @param penalties is a value for pealties for the model.
  // @param latency is actuators and measurement latency.
  Navigator(double refV, double stepDt, size_t nSteps,
            const Penalties &penalties, double latency);

  void Update(const std::vector<double> &ptsx, const std::vector<double> &ptsy,
              double px, double py, double psi, double speed, double steer,
              double throttle);

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
  double m_latency;
  MPC::Result m_result;
  std::vector<double> m_nextX;
  std::vector<double> m_nextY;
};