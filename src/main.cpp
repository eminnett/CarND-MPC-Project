#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
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
          vector<double> ptsx   = j[1]["ptsx"];
          vector<double> ptsy   = j[1]["ptsy"];
          double px             = j[1]["x"];
          double py             = j[1]["y"];
          double psi            = j[1]["psi"];
          double v              = j[1]["speed"];
          double steer_value    = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // Transform the coordinate space.
          double x_rotation = cos(-psi);
          double y_rotation = sin(-psi);
          for (int i = 0; i < ptsx.size(); i++) {

            double x_transaltion = ptsx[i] - px;
            double y_translation = ptsy[i] - py;

            ptsx[i] = x_transaltion * x_rotation - y_translation * y_rotation;
            ptsy[i] = x_transaltion * y_rotation + y_translation * x_rotation;
          }

          const int n_x = 6; // The size of the state vector.

          // This method of mapping a standard vector to an Eigen vector was
          // found here: https://forum.kde.org/viewtopic.php?f=74&t=94839#p194926
          Eigen::Map<Eigen::VectorXd> ptsx_vector(&ptsx[0], n_x);
          Eigen::Map<Eigen::VectorXd> ptsy_vector(&ptsy[0], n_x);

          // Fit the waypoints to a third degree polynomial.
          auto fit_coefficients = polyfit(ptsx_vector, ptsy_vector, 3);
          // Calculate the cross track and orientation errors from the polynomial fit.
          double cte            = polyeval(fit_coefficients, 0);
          double epsilon_psi    = -atan(fit_coefficients[1]);

          Eigen::VectorXd state(n_x);

          // The vehicle state using the vehicles reference frame.
          state << 0, 0, 0, v, cte, epsilon_psi;

          // Pass the polynomial fit and vehicle state into the MPC to determine
          // the control solution. The solution will be the control inputs that
          // minimise the cost function given the vehicle state and third degree
          // polynomial that describes the desired path of the vehicle.
          auto mpc_solution = mpc.Solve(state, fit_coefficients);


          // Extract the first set of points of the plynomial fit so they can be
          // drawn in the simulator.
          vector<double> next_x_vector;
          vector<double> next_y_vector;

          const int num_points = 20;
          const int distance_between_points = 5;
          for (int i = 1; i < num_points; i++) {
            double point_x = distance_between_points * i;
            double point_y = polyeval(fit_coefficients, distance_between_points * i);
            next_x_vector.push_back(point_x);
            next_y_vector.push_back(point_y);
          }

          // Extract points for the MPC solution so they can be darwn
          // in the simulator.
          vector<double> mpc_x_vector;
          vector<double> mpc_y_vector;

          for (int i = 2; i < mpc_solution.size() - 1; i += 2) {
            mpc_x_vector.push_back(mpc_solution[i]);
            mpc_y_vector.push_back(mpc_solution[i + 1]);
          }

          json msgJson;
          msgJson["steering_angle"] = mpc_solution[0];
          msgJson["throttle"]       = mpc_solution[1];
          msgJson["next_x"]         = next_x_vector;
          msgJson["next_y"]         = next_y_vector;
          msgJson["mpc_x"]          = mpc_x_vector;
          msgJson["mpc_y"]          = mpc_y_vector;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
