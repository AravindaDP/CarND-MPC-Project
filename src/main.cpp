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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // Store time at start of processing received data for latency estimation
    std::chrono::high_resolution_clock::time_point t_start =
                                      std::chrono::high_resolution_clock::now();

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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.44704;

          double steer_value = j[1]["steering_angle"]; // current steer (rad)
          steer_value *= -1.0; // flip direction from sim to match motion eqn's
          double throttle_value = j[1]["throttle"]; // current throttle [-1, 1]

          // Adjust predicted veh position to be after estimated latency time
          // using motion equations:
          //   x = x + v * cos(psi) * dt
          //   y = y + v * sin(psi) * dt
          //   psi = psi + v / Lf * delta * dt
          //   v = v + a * dt
          const double Lf = 2.67;
          const double latency = mpc.ave_latency_ms_ / 1000;
          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;
          psi = psi + v / Lf * steer_value * latency;
          v = v + throttle_value * latency;

          for(int i = 0; i < ptsx.size(); i++){
            //shift car reference angle to 90 degrees
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi));
            ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));
          }

          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          //calculate cte and epsi
          double cte = -polyeval(coeffs, 0);
          //double epsi = psi - atan(coeffs[1] + 2*px*coeffs[2] + 3*coeffs[3]*pow(px, 2));
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          auto vars = mpc.Solve(state, coeffs);

          steer_value = vars[0];
          throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * Add (x,y) points to list here, points are in reference to 
           * the vehicle's coordinate system the points in the simulator are 
           * connected by a Green line
           */
          for(int i = 2; i < vars.size(); i++){
            if(i%2 == 0){
              mpc_x_vals.push_back(vars[i]);
            }
            else{
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * Add (x,y) points to list here, points are in reference to 
           * the vehicle's coordinate system the points in the simulator are 
           * connected by a Yellow line
           */
          double poly_inc = 2.5;
          int num_points = 25;
          for(int i = 1; i < num_points; i++){
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


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

          // Estimate actual ave latency (processing time + actuator latency)
          auto cur_latency_ms =
                  std::chrono::duration_cast<std::chrono::milliseconds>
                  (std::chrono::high_resolution_clock::now() - t_start).count();

          // Smooth stored latency with exponential moving average
          constexpr int n_sm = 3;
          mpc.ave_latency_ms_ = mpc.ave_latency_ms_ * (n_sm - 1)/n_sm
                                 + cur_latency_ms * 1/n_sm;
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