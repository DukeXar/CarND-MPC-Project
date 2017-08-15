#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

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

// Represents set of penalties - weights of errors in the cost function.
struct Penalties {
  Penalties()
      : cte(1), psie(1), v(1), steer(1), acc(1), steer_gap(1), acc_gap(1) {}

  // Weight of the cross track error.
  double cte;
  // Weight of the psi error.
  double psie;
  // Weight of the velocity error.
  double v;
  // Weight of the stering actuator.
  double steer;
  // Weight of the acceleration actuator.
  double acc;
  // Weight of the gap between sequential activations for steering.
  double steer_gap;
  // Weight of ... for acceleration.
  double acc_gap;
};

// Model Predictive Control controller.
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

  // Constructs MPC controller.
  // @param refV is reference velocity.
  // @param stepDt is time delta between steps, in seconds.
  // @param nSteps is number of steps to evaluate.
  // @param penalties is a value for pealties for the model.
  // @param latency is actuators and measurement latency.
  MPC(double refV, double stepDt, size_t nSteps, const Penalties &penalties,
      double latency)
      : m_refV(refV),
        m_stepDt(stepDt),
        m_nSteps(nSteps),
        m_penalties(penalties),
        m_latency(latency) {}

  // Solve the model given an initial state and polynomial coefficients.
  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

 private:
  const double m_refV;
  const double m_stepDt;
  const size_t m_nSteps;
  const Penalties m_penalties;
  const double m_latency;
};

#endif /* MPC_H */
