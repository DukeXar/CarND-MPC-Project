#pragma once

#include <vector>
#include "MPC.h"

// Navigator is a wrapper over MPC that provides a shim between Udacity
// simulator measurements and actuators and MPC model.
class Navigator {
 public:
  // Constructs navigator.
  // @param refV is reference velocity.
  // @param stepDt is time delta between steps, in seconds.
  // @param nSteps is number of steps to evaluate.
  // @param penalties is a value for pealties for the model.
  // @param latency is an actuators' latency.
  Navigator(double refV, double stepDt, size_t nSteps,
            const Penalties &penalties, double latency);

  // Updates the model by applying incoming measurements.
  // After update, the corresponding Get* functions would return values for the
  // activations.
  // @param ptsx is a vector of x coordinates of the waypoints.
  // @param ptsy is a vector of y coordinates of the waypoints.
  // @param px is a x coordinate of a car.
  // @param py is an y coordinate of a car.
  // @param psi is an orientation angle of a car in radians (angle between
  //            orientation vector and x axis CCW).
  // @param speed is a current speed of a car.
  // @param steer is a steering angle of a car (in radians, positive CCW).
  // @param throttle is a throttle value (not used in the model).
  void Update(const std::vector<double> &ptsx, const std::vector<double> &ptsy,
              double px, double py, double psi, double speed, double steer,
              double throttle);

  std::vector<double> GetMpcXVals() const;
  std::vector<double> GetMpcYVals() const;
  std::vector<double> GetNextXVals() const;
  std::vector<double> GetNextYVals() const;

  // Returns steering value in the range [-25 deg; +25 deg] CCW.
  double GetSteerValue() const;
  // Returns throttle value in the range [-1; 1].
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