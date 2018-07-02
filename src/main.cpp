#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <utility>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2mps(double v) { return v * 0.44704; }


constexpr double MAX_TURN_ANGLE = 25.0; // in degrees
constexpr unsigned ACTUATORS_LATENCY = 100; // in milliseconds

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


const std::pair<Eigen::VectorXd, Eigen::VectorXd> TransformTrajectoryMapToVehicle(
  double x, double y, double psi, const vector<double>& ptsx, const vector<double>& ptsy) {

  // make sure that number of passed x and y values is equal
  assert(ptsx.size() == ptsy.size());

  std::size_t points_num = ptsx.size();

  Eigen::VectorXd points_x(points_num);
  Eigen::VectorXd points_y(points_num);

  for (std::size_t i=0; i < points_num; ++i){
    points_x(i) =  cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
    points_y(i) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
  }

  return std::make_pair(points_x, points_y);

}


double CalculateCrossTrackError(const Eigen::VectorXd& coeffs) {
  // in car coordinate system x position of the car is always 0.0
  return polyeval(coeffs, 0.0);
}


double CalculateOrientationError(const Eigen::VectorXd& coeffs) {
  // orientation error epsi is calculated using following equations:
  // epsi = psi - arctan(f'(x)). Since in car coordinate system current psi is always 0, we get
  // epsi = -atan(f'(x))
  // f(x) = coeffs[0] + coeffs[1]*x, thus f'(x) = coeffs[1]
  // This results in equation: epsi = -atan(coeffs[1])
  return -atan(coeffs[1]);
}

const Eigen::VectorXd GetLatencyCompensatedState(const double current_velocity,
  const double current_cte, const double current_epsi, const double current_delta,
  const double current_acceleration, const double latency) {

 // use the steering angle as orientation for predicting car position after 'latency' time,
 // starting from coordinates {0,0}, since we are in car coordinate system now
 double psi = current_delta;
 double px = current_velocity * cos(psi) * latency;
 double py = current_velocity * sin(psi) * latency;
 double cte = current_cte + current_velocity * sin(current_epsi) * latency;
 double epsi = current_epsi + current_velocity * current_delta * latency/Lf;
 psi = psi + current_velocity * current_delta * latency/Lf;
 double velocity = current_velocity + current_acceleration * latency;

 Eigen::VectorXd state(6);
 // in car coordinate system current position and orientation are always 0
 state << px, py, psi, velocity, cte, epsi;
 return state;
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double velocity = j[1]["speed"];
          velocity = mph2mps(velocity); //convert from miles per hour to miles per second

          double delta = j[1]["steering_angle"]; // Note! in simulator yaw  values. need negation before usage!
          double acceleration = j[1]["throttle"];


          // transform trajectory points from map coordinate system to vehicle coordinate system
          const std::pair<Eigen::VectorXd, Eigen::VectorXd> reference_trajectory = TransformTrajectoryMapToVehicle(px, py, psi, ptsx, ptsy);
          const Eigen::VectorXd& reference_points_x = reference_trajectory.first;
          const Eigen::VectorXd& reference_points_y = reference_trajectory.second;

          // fit a 3rd order polynomial from trajectory points
          const Eigen::VectorXd coeffs = polyfit(reference_points_x, reference_points_y, 3);

          // calculate cte
          const double cte = CalculateCrossTrackError(coeffs);

          // calculate ephi
          const double epsi = CalculateOrientationError(coeffs);

          // steering angle is negated to compensate inverter yaw of simulator
          const Eigen::VectorXd latency_compensated_state = GetLatencyCompensatedState(
            velocity, cte, epsi, -delta, acceleration, ACTUATORS_LATENCY/1000);

          // compute optimal trajectory for vehicle
          const MpcOutput mpc_output = mpc.Solve(latency_compensated_state, coeffs);


          json msgJson;

          // normalize calculated steer value (in radians) to [-1, 1] scale before sending back to simulator.
          msgJson["steering_angle"] = mpc_output.delta/(deg2rad(MAX_TURN_ANGLE));

          // level of throttle, in [-1, 1] scale
          msgJson["throttle"] = mpc_output.acceleration;

          // predicted trajectory points(in vehicle's coordinate system), will be displayes as Green line in simulator
          msgJson["mpc_x"] = mpc_output.X;
          msgJson["mpc_y"] = mpc_output.Y;

          // reference trajectory points (in vehicle's coordinate system). will be displayed as Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (unsigned i=0 ; i < reference_points_x.size(); ++i) {
            next_x_vals.push_back(reference_points_x(i));
            next_y_vals.push_back(reference_points_y(i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency.  The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(ACTUATORS_LATENCY));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
