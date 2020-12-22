#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;
class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  /**
  * @param fg vector of the cost constraints
  * @param vars a vector of variable values (state & actuators)
  */
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; ++t) {
      // Sum three components to reach the aggregate cost:
      //cross-track error
      fg[0] += CTE_COST * CppAD::pow(vars[cte_start + t], 2);
      // heading error
      fg[0] += EPSI_COST * CppAD::pow(vars[epsi_start + t], 2);
      // velocity error
      fg[0] += SPEED_COST * CppAD::pow(vars[v_start + t] - REF_V, 2);
    }

    /**
     *  Minimize the use of actuators.
     *
     *  Enhancement to constrain erratic control inputs.
     *  If we're making a turn, we'd like the turn to be smooth, not sharp.
     *  The vehicle velocity should not change too radically.
     */
    for (size_t t = 0; t < N - 1; ++t) {
      fg[0] += STEER_COST * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += ACCELERATION_COST * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    // This makes control decisions more consistent, or smoother.
    for (size_t t = 0; t < N - 2; ++t) {
      fg[0] += ACCELERATION_DIFF_COST * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      // Multiplying that part by a value > 1 (e.g. 500, 100) will influence the solver into keeping sequential steering values closer together.
      fg[0] += STEER_DIFF_COST * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; ++t) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuator at time t
      AD<double> delta0;
      AD<double> a0;
      if (1 == t){
        delta0 = vars[delta_start + t - 1];
        a0 = vars[a_start + t - 1];
      }
      // This avoids shakiness in curves
      else {
        delta0 = vars[delta_start + t - 2];
        a0 = vars[a_start + t - 2];
      }
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * DT
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * DT
      /*
       * psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * DT
       * Note if \large \delta is positive we rotate counter-clockwise, or turn left.
       * In the simulator however, a positive value implies a right turn and a negative value implies a left turn.
       */
      // v_[t+1] = v[t] + a[t] * DT
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * DT
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * DT
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * DT);
      fg[1 + v_start + t] = v1 - (v0 + a0 * DT);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * DT);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

/**
  * Initialize PID.
  * @param state Vector, containing the measurements, describing current state of the vehicle (x, y, psi, v, cte, epsi)
  * @param coeffs Coefficients of the polynomial, that had been fit into (x, y) trajectory of the vehicle.
  *               In case of the 1st order polynomial (ax+n), it is just one coefficient a.
*/
std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * Set the number of constraints: size of vector multilied my the amount of timestamps.
   */
  size_t n_constraints = state.size() * N;
  /**
   * Set the number of model variables (includes both states and inputs).
   * The state is a 6 element vector, the actuators is a 2
   * element vector and there are 25 timesteps. The number of variables is:
   * 6*25+2*24, where 6*25 is same as number of constraints
   */
  size_t n_vars = n_constraints + 2 * (N -1);


  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  // Set values of the initial variables by unfolding the state vector.
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Set lower and upper limits for the actuator angle (-25, 25).
  for (size_t i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // Set lower and upper limits for the Acceleration/decceleration.
  for (size_t i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = vars[x_start];
  constraints_lowerbound[y_start] = vars[y_start];
  constraints_lowerbound[psi_start] = vars[psi_start];
  constraints_lowerbound[v_start] = vars[v_start];
  constraints_lowerbound[cte_start] = vars[cte_start];
  constraints_lowerbound[epsi_start] = vars[epsi_start];

  constraints_upperbound[x_start] = constraints_lowerbound[x_start];
  constraints_upperbound[y_start] = constraints_lowerbound[y_start];
  constraints_upperbound[psi_start] = constraints_lowerbound[psi_start];
  constraints_upperbound[v_start] = constraints_lowerbound[v_start];
  constraints_upperbound[cte_start] = constraints_lowerbound[cte_start];
  constraints_upperbound[epsi_start] = constraints_lowerbound[epsi_start];


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
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

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Keep in mind, that the vector 'solution' has cost at solution.x[0], so here our ranges are moved by +1.
  // Return a patchworked array:
  // [0] steer angle change
  // [1] acceleration (throttle)
  // [2 - N*2] one after another, x & y values of the correction trajectory.

  std::vector<double> res;
  res.push_back(solution.x[delta_start]);
  res.push_back(solution.x[a_start]);
  for (size_t i = 0; i < N -1; ++i){
    res.push_back(solution.x[x_start + i +1]);
    res.push_back(solution.x[y_start + i +1]);
  }
  return res;
  }