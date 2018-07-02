#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

constexpr std::size_t N = 20; // number of steps
constexpr double dt = 0.05; // duration of one step, in seconds


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
constexpr double Lf = 2.67;

constexpr double ref_v = 40; // vehicle velocity

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
constexpr size_t x_start = 0;
constexpr size_t y_start = x_start + N;
constexpr size_t psi_start = y_start + N;
constexpr size_t v_start = psi_start + N;
constexpr size_t cte_start = v_start + N;
constexpr size_t epsi_start = cte_start + N;
constexpr size_t delta_start = epsi_start + N;
constexpr size_t a_start = delta_start + N - 1;

constexpr unsigned STEERING_TRANSITION_SMOOTHNESS = 700;
constexpr unsigned ACCELERATION_SMOOTHNESS = 20;



struct MpcOutput {
  public:

    vector<double> X;
    vector<double> Y;
    vector<double> DELTA; // orientation change
    vector<double> A; // acceleration

    /*
     * Return the predicted Steering Angle for given delay, in milliseconds
     */
    double GetSteeringAngle(unsigned delayed_state_index) const{
      // orientation angle scale in simulator is inverted to calculated by controller
      return -DELTA.at(delayed_state_index);
    }

    double GetThrottle(unsigned delayed_state_index) const{
      return A.at(delayed_state_index);
    }
};

class MPC {
 public:
  MPC(unsigned actuators_latency);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  MpcOutput Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);

  unsigned GetPseudoCurrentStateIndex() const;

  private:
    // index of vehicle state vector in computed mpc solution that represents
    // the pseudo current state with respect to actuators latency
    unsigned compensated_state_index_;
};

#endif /* MPC_H */
