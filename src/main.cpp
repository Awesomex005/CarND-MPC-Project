#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

using namespace std;

#define LATENCY_IN_MS 100

// for convenience
using json = nlohmann::json;

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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double mph2ms(double x) { return x * 0.44704; }

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

// Evaluate derivative of a polynomial.
double poly_derivative_eval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1 ; i < coeffs.size(); i++) {
    result += (i) * coeffs[i] * pow(x, (i-1));
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

void g2v_coordinate_transform(vector<double> &trans_ptsx, vector<double> &trans_ptsy,
                              vector<double> &ptsx, vector<double> &ptsy, double px, double py, double psi){
  double trans_ptsx_;
  double trans_ptsy_;
  for(int i=0; i<ptsx.size(); i++){
      trans_ptsx_ =      cos(psi)*(ptsx[i] - px) + sin(psi)*(ptsy[i] - py);
      trans_ptsy_ = (-1)*sin(psi)*(ptsx[i] - px) + cos(psi)*(ptsy[i] - py);
      trans_ptsx.push_back(trans_ptsx_);
      trans_ptsy.push_back(trans_ptsy_);
  }
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
    //cout << sdata << endl;
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
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          /* Transform waypoints from global coordinate to vehical coordinate. */
          vector<double> trans_ptsx;
          vector<double> trans_ptsy;
          g2v_coordinate_transform(trans_ptsx, trans_ptsy, ptsx, ptsy, px, py, psi);

          /* Fit a polynomial to transformed waypoints. */
          Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd>(trans_ptsx.data(), trans_ptsx.size());
          Eigen::VectorXd yvals = Eigen::Map<Eigen::VectorXd>(trans_ptsy.data(), trans_ptsy.size());
          auto coeffs = polyfit(xvals, yvals, 3);
          //cout << "coeffs" << coeffs << endl;

          /* vehical state， since we are using vehical coordinate, so our state x,y,psi are always 0. */
          double x0 = 0;
          double y0 = 0;
          double psi0 = 0;
          double v0 = mph2ms(v);
          double delta0 = delta;
          double a0 = a;
          double f0 = polyeval(coeffs, x0);
          double psides0 = atan(poly_derivative_eval(coeffs, x0));
          double cte0 = y0 - f0;
          double epsi0 = psi0 - psides0;
          while (epsi0 > M_PI) epsi0 -= 2.*M_PI;
          while (epsi0 <-M_PI) epsi0 += 2.*M_PI;

          /* Take account of lantency, predict the state before actuator take effect. */
          double latency = LATENCY_IN_MS*1.0 / 1000.0; // ms to s
          double x1 = (x0 + v0 * cos(psi0) * latency);
          double y1 = (y0 + v0 * sin(psi0) * latency);
          double psi1 = (psi0 + v0 * delta0 / Lf * latency);
          double v1 = (v0 + a0 * latency);
          double cte1 = (y0 - f0 + v0 * sin(epsi0) * latency);
          double epsi1 = (psi0 - psides0 + v0 * delta0 / Lf * latency);

          Eigen::VectorXd state(6);
          state << x1, y1, psi1, v1, cte1, epsi1;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          auto vars = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

          // the simulator use a steer_valute between -1 ~ 1, and turn left is map to negative vaule.
          steer_value = (-1) * vars[0]/deg2rad(25.0);
          throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = trans_ptsx;
          vector<double> next_y_vals;

          for(int i=0; i<next_x_vals.size(); i++){
            next_y_vals.push_back(polyeval(coeffs, next_x_vals[i]));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          //this_thread::sleep_for(chrono::milliseconds(LATENCY_IN_MS));
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
