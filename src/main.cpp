#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// because I don't want to rewrite half of this file.
using namespace std;

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

int main() {
  uWS::Hub h;

  // Reference velocity.
  const double REF_V = 50;
  // Time delta between steps.
  const double STEP_DT = 0.1;
  // Number of steps to model.
  const size_t N_STEPS = 10;

  // N_STEPS * STEP_DT = 1 (second) - this is how far in future model would
  // look.

  Penalties penalties;
  penalties.cte = 1500;
  penalties.psie = 1500;

  Navigator navigator(REF_V, STEP_DT, N_STEPS, penalties);

  h.onMessage([&navigator](uWS::WebSocket<uWS::SERVER> ws, char *data,
                           size_t length, uWS::OpCode opCode) {
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
          const vector<double> ptsx = j[1]["ptsx"];
          const vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];

          navigator.Update(ptsx, ptsy, px, py, -psi, v);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          // steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25]
          // instead of [-1, 1].
          msgJson["steering_angle"] = -navigator.GetSteerValue() / deg2rad(25);
          // Throttle must be in [-1, 1] range
          msgJson["throttle"] = navigator.GetThrottleValue();

          // Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = navigator.GetMpcXVals();
          msgJson["mpc_y"] = navigator.GetMpcYVals();

          // Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = navigator.GetNextXVals();
          msgJson["next_y"] = navigator.GetNextYVals();

          const auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          // this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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
