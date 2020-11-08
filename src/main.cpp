#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "car.h"
#include "my_car.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int max_lane = 2;
  MyCar my_car;
  my_car.lane = 1;
  my_car.ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &max_lane, &my_car]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          my_car.x = j[1]["x"];
          my_car.y = j[1]["y"];
          my_car.s = j[1]["s"];
          my_car.d = j[1]["d"];
          my_car.yaw = j[1]["yaw"];
          my_car.speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          my_car.end_path_s = j[1]["end_path_s"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          my_car.prev_path_size = prev_size;

          if (prev_size > 0) {
            my_car.s = my_car.end_path_s;
          }

          double target_vel = 49.99;

          auto lane_to_car = std::multimap<int, Car>();
          for (int i = 0; i < sensor_fusion.size(); i++) { 
            auto sf = sensor_fusion[i];
            int d = sf[6];
            int lane = d / 4;
            if (lane >= 0 && lane <= max_lane) {
                Car c;
                c.d = d;
                c.vx = sf[3];
                c.vy = sf[4];
                c.s = sf[5];
                c.lane = lane;
                c.speed = sqrt(c.vx * c.vx + c.vy * c.vy);
                c.s_projected = c.s + (double) prev_size * .02 * c.speed;

              lane_to_car.insert(std::make_pair(lane, c));
            }
          }

          double lane_center_d = my_car.lane * 4 + 2;

          double distance_from_lane = abs(lane_center_d - my_car.d);
          bool is_changing_lane = distance_from_lane > 0.5;

          auto ret = lane_to_car.equal_range(my_car.lane);
          for (auto itr = ret.first; itr != ret.second; itr++) {
            auto carInFront = itr->second;
            if (my_car.isInMyLaneAndTooClose(carInFront)) {
              bool changed_lane = false;
              if (my_car.lane > 0 && !is_changing_lane) {
                int new_lane = my_car.lane - 1;
                bool is_safe_to_change = true;
                auto ret = lane_to_car.equal_range(new_lane);
                for (auto itr = ret.first; itr != ret.second; itr++) {
                  auto car = itr->second;
                  if (!my_car.isSafeToChangeTheLane(carInFront, car)) {
                    is_safe_to_change = false;
                    break;
                  }
                }
                if (is_safe_to_change) {
                  my_car.lane = new_lane;
                  changed_lane = true;
                }
              }

              if (!changed_lane && my_car.lane < max_lane && !is_changing_lane) {
                int new_lane = my_car.lane + 1;
                bool is_safe_to_change = true;
                auto ret = lane_to_car.equal_range(new_lane);
                for (auto itr = ret.first; itr != ret.second; itr++) {
                  auto car = itr->second;
                  if (!my_car.isSafeToChangeTheLane(carInFront, car)) {
                    is_safe_to_change = false;
                    break;
                  }

                }
  
                if (is_safe_to_change) {
                  my_car.lane = new_lane;
                  changed_lane = true;
                }
              }

              if (!changed_lane) {
                // We could not change the lane - adjust the car so we do not
                // run into the car we could not overtake.
                target_vel = carInFront.speed * 2.24;
              }
            }
          }

          double diff = target_vel - my_car.ref_vel;
          double abs_diff = abs(diff);
          if (abs(diff) > .224) {
            double sign = 1;
            if (diff < 0) {
              sign = -1;
            }
            my_car.ref_vel += (sign * .224);
          } else if (abs_diff > 0.1) {
            // Make sure we are always slightly below the target velocity
            my_car.ref_vel += (diff - 0.1);
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = my_car.x;
          double ref_y = my_car.y;
          double ref_yaw = deg2rad(my_car.yaw);
          
          if (prev_size < 2) {
            double prev_car_x = my_car.x - cos(my_car.yaw);
            double prev_car_y = my_car.y - sin(my_car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(my_car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(my_car.y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(my_car.s + 30, 2 + 4 * my_car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(my_car.s + 60, 2 + 4 * my_car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(my_car.s + 90, 2 + 4 * my_car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }
  
          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            double N = (target_dist/(.02 * my_car.ref_vel / 2.24));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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