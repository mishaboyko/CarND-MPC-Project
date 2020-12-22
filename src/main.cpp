#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Since the Trajectory is passed in the global coordinate system,
 * we have to transform them to the vehicle coordinate system, to caclulate the actuator values.
 */
inline void convert_to_vehicle_system(std::vector<double> &pts_x, std::vector<double> &pts_y,double x, double y, double psi){
   for( size_t i = 0; i < pts_x.size(); ++i){
            double dx = pts_x[i] - x;
            double dy = pts_y[i] - y;
            pts_x[i] = dx * cos(-psi) - dy * sin(-psi);
            pts_y[i] = dy * cos(-psi) + dx * sin(-psi);
   }
}

/**
 * Compute the next state based on Kinematic motion model
 */
VectorXd compute_next_state(const VectorXd &state, double steering_angle, double throttle){
  VectorXd next_state(state.size());

  // Overrride x, y and angle, since we're in the local coordinate system.
  auto x = 0.0;
  auto y = 0.0;
  auto psi = 0.0;
  auto v = state(3);
  auto cte = state(4);
  auto epsi = state(5);

  // quations for the model:
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * DT
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * DT
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * DT
  // v_[t+1] = v[t] + a[t] * DT
  next_state(0) = x + v * cos(psi) * DT;
  next_state(1) = y + v * sin(psi) * DT;
  next_state(2) = psi - v / Lf * steering_angle * DT;
  next_state(3) = v + throttle * DT;
  next_state(4) = cte + v * sin(epsi) * DT;
  next_state(5) = epsi + next_state(2);

  return next_state;
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
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Reference trajectory (x and y coordinates), representing waypoints.
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];
          convert_to_vehicle_system(ptsx, ptsy, px, py, psi);

          double* ptr_ptsx = &ptsx[0];
          double* ptr_ptsy = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> pts_x(ptr_ptsx, ptsx.size());
          Eigen::Map<Eigen::VectorXd> pts_y(ptr_ptsy, ptsy.size());

          // The 3rd grade polynomial is fitted to the waypoints in vehicle coordinate system.
          auto coeffs = polyfit(pts_x, pts_y, 3);

          // evaluate y values of given x coordinate, a.k.a. calculate Cross Track Error.
          // Since we are in the vehicle coord system, always hold: (x,y) = (0,0). Thus:
          // 1. Reference point for polyeval() is x = 0
          // 2. No need to subsctract y cooridnate from the polyeval, since y = 0 is our reference anyhow.
          auto cte = polyeval(coeffs, 0);

          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = - atan(coeffs[1]);

          VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          auto next_state = compute_next_state(state, steering_angle, throttle);

          auto vars = mpc.Solve(next_state, coeffs);

          json msgJson;

          // Apply deg2rad(25) to the steering value.
          // Otherwise the values will be in between [-25, 25] instead of [-1, 1], where +/-25 degrees is our steering constraint.
          msgJson["steering_angle"] = - vars[0] / deg2rad(25);
          msgJson["throttle"] = vars[1];

          /*
           * mpc_x,mpc_y, next_x, & next_y points are displayed in reference to the vehicle's coordinate system.
           * The x axis always points in the direction of the car’s heading and the y axis points to the left of the car.
           * If you want to display a point 10 units directly in front of the car, you could set next_x = {10.0} and next_y = {0.0}.
           * Remember that the server returns waypoints using the map's coordinate system, which is different than the car's coordinate system.
e          */

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;;

          for (size_t i = 2; i < vars.size(); i+=2)
          {
              mpc_x_vals.push_back(vars[i]);
              mpc_y_vals.push_back(vars[i+1]);
          }

          /**
           *   The MPC Predicted trajectory is the vehicle's coordinate system.
           *   It is represented by a Green line.
           */
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          /**
           *   Display the waypoints/reference line by a Yellow line
           *   The points are in the vehicle's coordinate system.
           */
          msgJson["next_x"] = ptsx;
          msgJson["next_y"] = ptsy;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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