# Term 2 MPC Control Quizzes

This repository hosts the quiz/lab for *Vehicle Models* and *Model Predictive Control* sections.

---

## Table of Contents

0. General Build Process Overview
1. [Polynomial Fitting](./polyfit) - Fit and evaluate polynomials.
2. [Global Kinematic Model](./global_kinematic_model) - Implement the *Global Kinematic Model*.
3. [MPC](./mpc_to_line) - Implement MPC and minimize cross track and orientation errors to a straight line trajectory.

## 0. General Build Process Overview

1. The quizzes were build in Windows 10 x64 platform running WSL (Windows Subsystem for Linux) Ubuntu 16.04 version.
2. Pre-requisites and dependencies were installed based on following instructions.
3. Each projecct was build by creating a build directory using `mkdir build`. Then the project was compiled using `cmake .. && make`.

#### Installation of Dependencies for MPC quiz

* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Version 3.12.1 was download from [here](https://www.coin-or.org/download/source/Ipopt/) and installed using `sudo install_ipopt.sh`. Additional dependendies were automatically installed.
* [CppAD](https://www.coin-or.org/CppAD/)
  * On WSL, CppAD was installed using `sudo apt-get install cppad`.

In the following sections, code implemetation for each quiz is discussed.

## 1. Polynomial Fitting

In this lab, the pre-defined functions, `polyfit` and `polyval` were used to fit a polynomial to points and then evaluate the polynomial at defined points. The fitting was done using the following code block.

```
Eigen::VectorXd ceoffs = polyfit(xvals, yvals, 3);
```

The fitted coefficients were then used to evaluate the polynomial.

```
for (double x = 0; x <= 20; x += 1.0) {
  std::cout << polyeval(ceoffs, x) << std::endl;
}
```

The expected output was observed.

## 2. Global Kinematic Model

In this lab, the global kinematic model was implemented to predict state vector at a future time instant. The function call used was:

```
auto next_state = globalKinematic(state, actuators, 0.3);
```

The implementation of this function is:

```
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());
  next_state[0] = state[0] + state[3]*cos(state[2])*dt;
  next_state[1] = state[1] + state[3]*cos(state[2])*dt;
  next_state[2] = state[2] + state[3]*actuators[0]*dt/Lf;
  next_state[3] = state[3] + actuators[1]*dt;
  return next_state;
}
```

The expected output was observed.

## 3. MPC Implementation

In this lab, MPC algorithm was implemented to minimize cross track and orientation errors to a straight line trajectory. The implementation is carried out in two major parts:
* Definition of the `FG_eval` class.
* Definition of the `MPC::Solve(Eigen::VectorXd, Eigen::VectorXd)` function.

Before the implementation is discussed, list of hyperparameter used to tune the MPC model is listed below.
* `size_t N` is the number of preductions in future. This is set to 25.
* `double dt` is the time difference between prediction. This is set to 0.05.
* Additional hyperparameters are set to penalize agressive maneuvers. `double lambda_delta` is to penalize large steering input (set to 500) and `double lambda_a` is to penalize large acceleration inputs (set to 100).
* Reference velocity (`double ref_v`) is set to 40 MPH.

A description of each code block and line numbers in the source file (`MPC.cpp`) is presented below.

1. The definition of the cost related the reference state is presented in code between lines 67-79. The cost is the optimization object and the cummulative cost is stored in first element of constraint vector `fg`. In addition to cost from CTE, epsi and velocity - aspects such as magnitude of steering inputs and time changes in steering/acceleration inputs are also used to penalize cost. For the last cost component, tunable Lagrange multipliers are used (`lambda_delta` and `lambda_a`).

2. The definition of the model constraints are presented in the code section between lines 91-133. The constraints on initial state and difference between two time steps is assembled into constraint vector `fg`. It is noted that the key equations from global kinematic model are used in assembling constraints using code section between lines 128-133.

3. Finally, the `MPC::Solve` function is implemented in code section betwen lines 145-254. The key steps in the lab are implementing the lower/upper bound constraints for the state variabels (positions [`px`,`py`], orientation `psi`, velocity `v`, cross-track error `cte`, orientation error `epsi`). Finally, an `FG_eval` class object is initialize with the `coeffs` vector of fitted trajectory. The initial state `x0` is then utilized to solve the state variable for each time step using non-linear optimization in code section listed betwen lines 235-240. The solution is verified using code in line 246. The first predicted state in the horizon is returned.

In the `main` function block, the linear trajectory points are fitted with a line and along with the state, the control inputs (`delta` and `a`) are predicted. After running the code using defined input state and trajectory, expected results were obtained. This implementation is further used in the MPC project.