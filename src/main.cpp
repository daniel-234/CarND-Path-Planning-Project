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

  // Define a lane variable to be able to switch quickly from a lane to another
  int lane = 1;
  // Velocity to target in MPH
  double ref_velocity = 49.5;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_velocity, &lane]
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
          //
          // The data JSON object is the data provided from the Simulator to the C++ Program
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          //
          // The previous list of x points previously given to the simulator
          auto previous_path_x = j[1]["previous_path_x"];
          // The previous list of y points previously given to the simulator
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          //
          // The previous list's last point's frenet s value
          double end_path_s = j[1]["end_path_s"];
          // The previous list's last point's frenet d value
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // To help the car transition, we can use the previous points returned by the simulator. 
          // Find the previous path size to decide how to calculate the new points, based on the fact 
          // that we could either follow the path the car is already in or extrapolate the next points
          // from the car's angle if it has just started to move. 
          int previous_size = previous_path_x.size();


          // The sensor fusion data from the simulator is reporting the data from all the other cars
          // in the road, that have valus for s, d, x, y, vx and vy values. 
          // We can use those values to track the other cars on the same side of the road and decide how our 
          // car should behave. 

          // If we have the value of the previous s position of the car, use that as the car value to 
          // compare it to other cars s value. 
          if (previous_size > 0) {
            car_s = end_path_s;
          }

          bool is_too_close = false;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            // Check if the car is in our lane.
            // The d value is the 7th value of a car's data measurements returned by the simulator. 
            float d = sensor_fusion[i][6]; 
            // Check if any can is in the same lane of our car plus a range of +/- 2 meters, in case
            // we're moving to another lane. 
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
              double other_car_vx = sensor_fusion[i][3];
              double other_car_vy = sensor_fusion[i][4];
              double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
              double other_car_s_position = sensor_fusion[i][5];

              // Project this car that's on the same lane in the future, based on previous s points and 
              // its actual velocity. This way we could avoid hitting it. 
              other_car_s_position += ((double)previous_size * 0.02 * other_car_speed);
              // Check if the other car is in front of us and the gap between the two is less than 30 meters.
              if ((other_car_s_position > car_s) && (other_car_s_position - car_s) < 30) {
                // Lower our car's reference velocity to 29.5 MPH.
                //ref_velocity = 29.5;
                ref_velocity = other_car_speed;
              }
            }
          }



          // Create a list of widely spaced waypoints, evenly spaced at 30 meters, that will be interpolated
          // with a spline to create a smooth path for the car.
          vector<double> list_of_anchor_x_points;
          vector<double> list_of_anchor_y_points;

          // Get a reference for the car's starting point.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If the previous path has less than 2 points, use the car state (starting point and angle) to 
          // generate two points.
          if (previous_size < 2) {
            // We could create a previous point by using the car angle to generate a path tangent to the car.
            double previous_car_x = car_x - cos(car_yaw);
            double previous_car_y = car_y - sin(car_yaw);

            // Add the previous car point obtained from the car's angle. 
            list_of_anchor_x_points.push_back(previous_car_x);
            list_of_anchor_y_points.push_back(previous_car_y);
            // Add the current car state to the list.
            list_of_anchor_x_points.push_back(car_x);
            list_of_anchor_y_points.push_back(car_y);
          } else {
            // If there are more points in the previous points list, use the last point and the second
            // to last to compute the path tangent to the car.
            //
            // Last point: yaw is the curent value given by the simulator. 
            ref_x = previous_path_x[previous_size - 1];
            ref_y = previous_path_y[previous_size - 1];
            // Second to last point.
            double previous_ref_x = previous_path_x[previous_size - 2];
            double previous_ref_y = previous_path_y[previous_size - 2];
            // Comnpute the angle of the car at previous point by taking it as if it drew 
            ref_yaw = atan2(ref_y - previous_ref_y, ref_x - previous_ref_x);

            // Add the reference points to the list that will be interpolated. 
            list_of_anchor_x_points.push_back(previous_ref_x);
            list_of_anchor_y_points.push_back(previous_ref_y);

            list_of_anchor_x_points.push_back(ref_x);
            list_of_anchor_y_points.push_back(ref_y);
          }

          for (int i = 0; i < list_of_anchor_x_points.size(); i++) {
            std::cout << "first x: " << list_of_anchor_x_points[i] << ", first y: " << list_of_anchor_y_points[i] << std::endl;
          }

          // After the two reference points, we push 3 more points as anchor for the spline that will interpolate them
          // and generate a full list of points for the car to follow. 
          // Add 3 more points, evenly spaced at 30 meters from each other and from the beginning of the map reference.
          // The car is travelling at the center of a lane (2 meters from the lane line). Every lane is 4 meters wide, so
          // the lane variable will place the car in the selected lane number (from 0 to 2, where 0 is the lane on the left).
          vector<double> first_waypoint = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> second_waypoint = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> third_waypoint = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
          list_of_anchor_x_points.push_back(first_waypoint[0]);
          list_of_anchor_y_points.push_back(first_waypoint[1]);
            
          list_of_anchor_x_points.push_back(second_waypoint[0]);
          list_of_anchor_y_points.push_back(second_waypoint[1]);
            
          list_of_anchor_x_points.push_back(third_waypoint[0]);
          list_of_anchor_y_points.push_back(third_waypoint[1]);
          
          for (int i = 0; i < list_of_anchor_x_points.size(); i++) {
            std::cout << "spaced x: " << list_of_anchor_x_points[i] << ", spaced y: " << list_of_anchor_y_points[i] << std::endl;
          }

          // Transform the 5 points that we'll take as anchor to local car coordinates. 
          // The last point of the previous path is put at (0, 0) with an angle at 0 degrees. 
          // The other 4 points only change their angle to 0 degrees. This way even if the car is in a curved road,
          // we could treat it as if there is no angle (as if looking from its point of view).
          for (int i = 0; i < list_of_anchor_x_points.size(); i++) {
            // Shift the car reference angle to 0 degrees
            double shift_x = list_of_anchor_x_points[i] - ref_x;
            double shift_y = list_of_anchor_y_points[i] - ref_y;

            list_of_anchor_x_points[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            list_of_anchor_y_points[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

            std::cout << "x: " << list_of_anchor_x_points[i] << ", y: " << list_of_anchor_y_points[i] << std::endl;
          }

          // Create a spline
          tk::spline s;

          // Set (x, y) points to the spline
          s.set_points(list_of_anchor_x_points, list_of_anchor_y_points);

          // Define the lists of (x, y) points that the planner will use.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start from the previous path points that were left from last cycle. Using information from 
          // the previous path ensures that there is a smooth transition from cycle to cycle. 
          // Every iteration the planner will output the same number of points. Maybe the simulator, in the
          // cycle time, is only able to go through 5 of these points, so there are still (total - 5) that 
          // haven't been visited by the car. Instead of recreating the path from scratch every single time, we 
          // add points to it and work with what was left from the last time. So in the next cycle, we could just
          // add 5 points as the last ones and get a new path. 
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // To calculate the points that the car should visit, we can fix a point in the horizon that the car would reach. 
          // If we use a reference where the car direction at the starting point is the origin of the x axis with a 0 degree
          // angle, we can assign the x value we want to that point, for example 30 meters. 
          // By knowing that x value, we can get the y value from the spline we declared at line 193.
          // Taking the car's status as reference and the target point as opposite angle, we can paint a triangle where the
          // hypotenuse is the distance to the target and the opposite side is the x axis. 
          //
          // As every 0.02 seconds the car visits a new point, we can find the number of points that the car would visit before
          // reaching the target by using the formula: n (points) * 0.02 (sec/point) * velocity (meters/second) = distance (meters)
          //
          // The number of points the hypotenuse is broken into is the same for the x axis. Once we know the x values of these
          // n points, we can feed them to the spline and get the y value for each of them. 

          // Target point coordinates. 
          double target_x = 30.0; 
          // Get the y value by passing the target s into the spline. 
          double target_y = s(target_x);
          // Target distance.
          double target_distance = sqrt(target_x * target_x + target_y * target_y);

          // Variable that takes into account the fact that we started at the origin. 
          double x_add_on = 0;

          // Compute the rest of the points the path planner will output. 
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            // Compute the number of points. As we need the velocity to be in meters per second, we divide the reference velocity by 2.24
            double n = (target_distance / (0.02 * ref_velocity / 2.24));
            // The x coordinate of a point is the x coordinate of the previous x point plus the x coordinate of the x value 
            // divided by the number of points (n).
            double x_point = x_add_on + target_x / n;
            // To compute the corresponding y point, we use the spline on the x coordinate. 
            double y_point = s(x_point); 
            // Save the x coordinate as reference for the next loop iteration (see line 243 for its use).
            x_add_on = x_point;

            // Convert the coordinate system from local coordinates to global coordinates by rotating and shifting.
            
            // Save this point's x and y coordinates
            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate from the car's coordinate system to the map coordinate sysyem. 
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            // Push the values to the list of output points. 
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          for (int i = 0; i < next_x_vals.size(); i++) {
            std::cout << "x: " << next_x_vals[i] << ", y: " << next_y_vals[i] << std::endl;
          }         

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