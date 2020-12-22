#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set the timestep length and duration
const size_t N = 7;
const double DT = 0.1;

// Both the reference cross track and orientation errors are 0.
// Set reference velocity (in mph).
const double REF_V = 70;
const double CTE_COST = 50.0;
const double EPSI_COST = 50.0;
const double SPEED_COST = 1.0;
const double STEER_COST = 20000.0;
const double ACCELERATION_COST = 0.05;
const double STEER_DIFF_COST = 200.0;
const double ACCELERATION_DIFF_COST = 1.0;

// The solver takes all the state variables and actuator variables in a singular vector.
// Thus, it is practical to set ranges, when one variable starts and another ends.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state, 
                            const Eigen::VectorXd &coeffs);
};

#endif  // MPC_H
