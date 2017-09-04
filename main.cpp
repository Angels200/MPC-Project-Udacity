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

			  // Affine transformation. Translate to car coordinate system then rotate to the car's orientation.

			  Eigen::MatrixXd waypoints = MapToCarTransform(px,py,psi,ptsx,ptsy);
			  Eigen::VectorXd lptsx = waypoints.row(0);
			  Eigen::VectorXd lptsy = waypoints.row(1);

			  // fit a 3rd order polynomial to the waypoints
			  auto coeffs = polyfit(lptsx, lptsy, 3);

			  // get cross-track error from fit
			  double cte = polyeval(coeffs, 0); ;

			  // get orientation error from fit
			  double epsi = -atan(coeffs[1]);

			  // state in vehicle coordinates: x,y and orientation are always zero
			  Eigen::VectorXd state(6);
			  state << 0, 0, 0, v, cte, epsi;

			  // compute the optimal trajectory
			  Solution sol = mpc.Solve(state, coeffs);

			  double steer_value = sol.Delta.at(latency_ind);
			  double throttle_value= sol.A.at(latency_ind);
			  mpc.delta_prev = steer_value;
			  mpc.a_prev = throttle_value;

			  json msgJson;
			  // mathematically positive angles are negative in the simulator, therefore we have to feed the negative steer_value.
			  // WARNING: the current simulator expects angles as a fraction of the max angle, here 25 degrees, not radians! It must be in the range [-1,1].
			  // 25 degrees in radians are 0.436332.
			  msgJson["steering_angle"] = -steer_value/0.436332;
			  msgJson["throttle"] = throttle_value;

			  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
			  // the points in the simulator are connected by a Green line
			  cout << " x           " << px << endl;
			  cout << " y           " << py << endl;
			  cout << " psi         " << psi << endl;
			  cout << " v           " << v << endl;
			  cout << " cte         " << cte << endl;
			  cout << " epsi        " << epsi << endl;
			  cout << " steer_value " << steer_value << endl ;
			  cout << " throttle    " << throttle_value << endl ;

			  // Display the MPC predicted trajectory
			  msgJson["mpc_x"] = sol.X;
			  msgJson["mpc_y"] = sol.Y;

			  //Display the waypoints/reference line
			  vector<double> next_x_vals;
			  vector<double> next_y_vals;

			  for (unsigned i=0 ; i < ptsx.size(); ++i) {
				next_x_vals.push_back(lptsx(i));
				next_y_vals.push_back(lptsy(i));
			  }
			  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
			  // the points in the simulator are connected by a Yellow line

			  msgJson["next_x"] = next_x_vals;
			  msgJson["next_y"] = next_y_vals;

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
			  int milliseconds_latency = 100;
			  this_thread::sleep_for(chrono::milliseconds(milliseconds_latency));
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