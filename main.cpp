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

Eigen::MatrixXd MapToCarTransform(double x, double y, double psi, const vector<double> & gptsx, const vector<double> & gptsy) {
    assert(gptsx.size() == gptsy.size());
    unsigned len = gptsx.size();

    auto waypoints = Eigen::MatrixXd(2,len);

    for (auto i=0; i<len ; ++i){
      waypoints(0,i) =   cos(psi) * (gptsx[i] - x) + sin(psi) * (gptsy[i] - y);
      waypoints(1,i) =  -sin(psi) * (gptsx[i] - x) + cos(psi) * (gptsy[i] - y);
    }

    return waypoints;

  }

vector<vector<double>> BuildGreenLine(Eigen::VectorXd state, std::vector<double> results){
	std::vector<double> mpc_x_vals = {state[0]};
	std::vector<double> mpc_y_vals = {state[1]};
	for (int i = 2; i < results.size(); i+=2) {
		mpc_x_vals.push_back(results[i]);
		mpc_y_vals.push_back(results[i+1]);
	}
	return {mpc_x_vals,mpc_y_vals};
}

vector<vector<double>> BuildYellowLine(Eigen::VectorXd coeffs, double x_poly, int length ){
	std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    for (int i = 1; i < length; i++) {
    	next_x_vals.push_back(x_poly*i);
    	next_y_vals.push_back(polyeval(coeffs, x_poly * i));
    }
    return {next_x_vals, next_y_vals};
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
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // Need Eigen vectors for polyfit
          Eigen::MatrixXd waypoints = MapToCarTransform(px,py,psi,ptsx,ptsy);
		  Eigen::VectorXd lptsx = waypoints.row(0);
		  Eigen::VectorXd lptsy = waypoints.row(1);

          /*
          * Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          * Simulator has 100ms latency, so will predict state at that point in time.
          * This will help the car react to where it is actually at by the point of actuation.
          */

          // Fits a 3rd-order polynomial to the above x and y coordinates
          auto coeffs = polyfit(lptsx, lptsy, 3);

          // Calculates the cross track error
          // Because points were transformed to vehicle coordinates, x & y equal 0 below.
          // 'y' would otherwise be subtracted from the polyeval value
          double cte = polyeval(coeffs, 0);

          // Calculate the orientation error
          // Derivative of the polyfit goes in atan() below
          // Because x = 0 in the vehicle coordinates, the higher orders are zero
          // Leaves only coeffs[1]
          double epsi = -atan(coeffs[1]);



          // Predict state after latency using update equations
          // x, y and psi are all zero after transformation above
          double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
          const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
          double pred_psi = 0.0 + v * -delta / Lf * dt;
          double pred_v = v + a * dt;
          double pred_cte = cte + v * sin(epsi) * dt;
          double pred_epsi = epsi + v * -delta / Lf * dt;

          // Feed in the predicted state values
          Eigen::VectorXd state(6);
          state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

          // Solve for new actuations (and to show predicted x and y in the future)
          auto results = mpc.Solve(state, coeffs);

          // Calculate steering and throttle
          // Steering must be divided by deg2rad(25) to normalize within [-1, 1].
          // Multiplying by Lf takes into account vehicle's turning ability
          double steer_value = results[0] / (deg2rad(25) * Lf);
          double throttle_value = results[1];

          // Send values to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory -- Green Line
          auto grn_trajectory = BuildGreenLine(state, results);
          auto mpc_x_vals = grn_trajectory[0];
          auto mpc_y_vals = grn_trajectory[1];


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          auto yel_trajectory = BuildYellowLine(coeffs,2.5, 25);
          auto next_x_vals = yel_trajectory[0];
          auto next_y_vals = yel_trajectory[1];

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car doesn't actuate the commands instantly.
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
