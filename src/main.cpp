#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Navigator.h"
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
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Reference velocity.
  const double REF_V = 50;

  // Latency in seconds.
  const double LATENCY = 0.1;

  // These two values below seems optimal. Not ehough steps - car would try to
  // optimize for local position. too many steps - would hard to fit polynomial
  // of 3rd order, so it would try to cut corners. This is also affected by
  // latency. When activation and measurement is instant,
  // it is easy to estimate far in future (higher N_STEPS), but when latency is
  // effectively increasing the STEP_DT, it is better to not have that far
  // horizon.

  // Number of steps to model.
  const size_t N_STEPS = 10;
  // Time delta between steps. This gives N_STEPS * STEP_DT = 1 seconds horizon
  // to predict, which should be enough. There is no much sense to keep STEP_DT
  // less than LATENCY, as first activation would correspond to the car position
  // before actual activation would occur.
  const double STEP_DT = 0.1;

  // These parameters actually started up with heavily penalizing cte and psie
  // (penalty of 2000), and much less penalizing the steer gap (100). What
  // happens is when there is no latency, these values work perfectly. When
  // latency is introduced, car is oscillating and eventually falls out of the
  // track.
  // By heavily penalizing the activations gap, and less penalizing actual
  // errors, car become stable again.
  //
  // Higher target velocity would require these values to be adjusted.
  // E.g.: cte=100, psie=100, v=1, steer=50, acc=1, steer_gap=15000,
  // acc_gap=1000 works well for target velocity 50. But for 70 car starts
  // oscillating (with latency). Also for higher speeds, having small latency of
  // actuators should be very important.

  Penalties penalties;
  penalties.cte = 100;
  penalties.psie = 100;
  // Keeping speed is less important than careful driving.
  penalties.v = 4;

  // How much to dump activations. Allow steering heavier, but carefully (see
  // steer_gap below).
  penalties.steer = 30;
  penalties.acc = 10;

  // Try to not do abrupt steering.
  penalties.steer_gap = 20000;
  penalties.acc_gap = 500;

  Navigator navigator(REF_V, STEP_DT, N_STEPS, penalties, LATENCY);

  h.onMessage([&navigator, &LATENCY](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                     size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          const std::vector<double> ptsx = j[1]["ptsx"];
          const std::vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double steer = j[1]["steering_angle"];
          const double acc = j[1]["throttle"];

          navigator.Update(ptsx, ptsy, px, py, psi, v, -steer, acc);

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
          if (LATENCY) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(LATENCY * 1000)));
          }
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
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
