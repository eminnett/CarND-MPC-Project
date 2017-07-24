#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// N: 25, dt: 0.05: The car oscillates left and right
// N: 20, dt: 0.05: The car oscillates left and right
// N: 15, dt: 0.05: The car oscillates left and right but a little less
// N: 15, dt: 0.1: The car oscillates left and right but a little less
// N: 10, dt: 0.1: The car drives safely.
// The best performance appears to be for N: 10, dt: 0.1. I wanted to explore
// having a prediction horizon T value greater than 1, but found when it
// is too large, the model performed poorly.
const size_t N = 10;
const double dt = 0.1;

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

double referrence_cte  = 0;
double referrence_epsi = 0;
double referrence_v    = 100; // This is the maximum speed for the throttle controller.

// Store the first index for each set of variables for later use when
// accessing the constraints vector fg.
const int x_first_index     = 0;
const int y_first_index     = x_first_index + N;
const int psi_first_index   = y_first_index + N;
const int v_first_index     = psi_first_index + N;
const int cte_first_index   = v_first_index + N;
const int epsi_first_index  = cte_first_index + N;
const int delta_first_index = epsi_first_index + N;
const int a_first_index     = delta_first_index + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;
    // Reference State Cost

    // After several hours of trial and error, I settled on the following
    // cost multipliers with a reference veleocity of 100. I found this
    // configuration resulted in the best compromise between safety,
    // smoothness, and speed.
    const int cte_cost_multiplier       = 5000;
    const int epsi_cost_multiplier      = 1000;
    const int steering_cost_multiplier  = 50000;
    const int throttle_cost_multiplier  = 75;
    const int delta_cost_multiplier     = 100;
    const int a_cost_multiplier         = 10;
    for (int i = 0; i < N; i++) {
      // The part of the cost based on the reference state.
      fg[0] += cte_cost_multiplier * CppAD::pow(vars[cte_first_index + i] - referrence_cte, 2);
      fg[0] += epsi_cost_multiplier * CppAD::pow(vars[epsi_first_index + i] - referrence_epsi, 2);
      // Adding a multiplier in front of this term causes the model to behave erratically.
      fg[0] += CppAD::pow(vars[v_first_index + i] - referrence_v, 2);
      // Minimise actuations.
      if (i < N - 1) {
        fg[0] += steering_cost_multiplier * CppAD::pow(vars[delta_first_index + i], 2);
        fg[0] += throttle_cost_multiplier * CppAD::pow(vars[a_first_index + i], 2);
        // Minimise the 'value gap' between actuations
        if (i < N - 2) {
          fg[0] += delta_cost_multiplier * CppAD::pow(vars[delta_first_index + i + 1] - vars[delta_first_index + i], 2);
          fg[0] += a_cost_multiplier * CppAD::pow(vars[a_first_index + i + 1] - vars[a_first_index + i], 2);
        }
      }
    }

    // Constraints

    // Initial constraints
    // Increment the first indices to account for the cost at index 0.
    fg[1 + x_first_index]    = vars[x_first_index];
    fg[1 + y_first_index]    = vars[y_first_index];
    fg[1 + psi_first_index]  = vars[psi_first_index];
    fg[1 + v_first_index]    = vars[v_first_index];
    fg[1 + cte_first_index]  = vars[cte_first_index];
    fg[1 + epsi_first_index] = vars[epsi_first_index];

    // The remainder of the model constraints.
    for (int i = 0; i < N - 1; i++) {
      // The states for time setp 0.
      AD<double> x_0    = vars[x_first_index + i];
      AD<double> y_0    = vars[y_first_index + i];
      AD<double> psi_0  = vars[psi_first_index + i];
      AD<double> v_0    = vars[v_first_index + i];
      AD<double> cte_0  = vars[cte_first_index + i];
      AD<double> epsi_0 = vars[epsi_first_index + i];

      // The actuators at time step 0.
      AD<double> delta_0 = vars[delta_first_index + i];
      AD<double> a_0     = vars[a_first_index + i];

      // The third degree polynomial for time step 0.
      AD<double> f_0 = coeffs[0] + coeffs[1] * x_0 + coeffs[2] * x_0 * x_0 + coeffs[3] * x_0 * x_0 * x_0 ;
      // The desired orientation at time step 0.
      AD<double> psi_desired_0 = CppAD::atan(3 * coeffs[3] * x_0 * x_0 + 2 * coeffs[2] * x_0 + coeffs[1]);

      // The states for time setp 1.
      AD<double> x_1    = vars[x_first_index + i + 1];
      AD<double> y_1    = vars[y_first_index + i + 1];
      AD<double> psi_1  = vars[psi_first_index + i + 1];
      AD<double> v_1    = vars[v_first_index + i + 1];
      AD<double> cte_1  = vars[cte_first_index + i + 1];
      AD<double> epsi_1 = vars[epsi_first_index + i + 1];

      // The constraints defined by the difference between the states at time
      // step 1 and the results of the global kinematic model applied to the
      // state at time step 0 and the difference between steps dt.
      fg[2 + x_first_index + i]    = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      fg[2 + y_first_index + i]    = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      fg[2 + psi_first_index + i]  = psi_1 - (psi_0 - v_0 * delta_0 / Lf * dt);
      fg[2 + v_first_index + i]    = v_1 - (v_0 + a_0 * dt);
      fg[2 + cte_first_index + i]  = cte_1 - ((f_0 - y_0) + (v_0 * CppAD::sin(epsi_0) * dt));
      fg[2 + epsi_first_index + i] = epsi_1 - ((psi_0 - psi_desired_0) - v_0 * delta_0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // NB: The majority of this method has been copied from the MPC quiz
  // solution that can be found here:
  // https://github.com/eminnett/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

  const int n_vars =  N * 6 + (N - 1) * 2;
  const int n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_first_index; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_first_index; i < a_first_index; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_first_index; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_first_index]    = x;
  constraints_lowerbound[y_first_index]    = y;
  constraints_lowerbound[psi_first_index]  = psi;
  constraints_lowerbound[v_first_index]    = v;
  constraints_lowerbound[cte_first_index]  = cte;
  constraints_lowerbound[epsi_first_index] = epsi;

  constraints_upperbound[x_first_index]    = x;
  constraints_upperbound[y_first_index]    = y;
  constraints_upperbound[psi_first_index]  = psi;
  constraints_upperbound[v_first_index]    = v;
  constraints_upperbound[cte_first_index]  = cte;
  constraints_upperbound[epsi_first_index] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
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

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> mpc_solution;
  mpc_solution.push_back(solution.x[delta_first_index]);
  mpc_solution.push_back(solution.x[a_first_index]);
  for (int i = 0; i < N - 1; i++) {
  	mpc_solution.push_back(solution.x[x_first_index + i + 1]);
  	mpc_solution.push_back(solution.x[y_first_index + i + 1]);
  }

  return mpc_solution;
}
