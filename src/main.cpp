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

  //lane
  int lane =1;
  //reference velocity
  double ref_vel=  49.5;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy , &lane, &ref_vel]
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
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          //std::cout<<"Prev"<<prev_size<< std::endl;

          //HERE the code for Sensor Fusion

          //if  something
          if(prev_size>0){
            car_s = end_path_s;
          }

          bool too_close = false;

          for (int i=0; i< sensor_fusion.size();i++)
          {
             float d = sensor_fusion[i][6];
             if(d<(2+4*lane+2)&&d>(2+4*lane-2))  //the car is in our lane
             {  
               double vx = sensor_fusion[i][3];
               double vy = sensor_fusion[i][4];

               double check_speed = distance(0,0,vx,vy); //calculate the magnitude
               double check_car_s = sensor_fusion[i][5];

               check_car_s += ((double)prev_size*0.02*check_speed); //project the s value outwards in time

               if((check_car_s>car_s) && ((check_car_s-car_s)<30)){ //if the car is too near

               ref_vel=29.5;

               }


             }




          }









          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          //std::cout<<sensor_fusion<<std::endl;

          //First Straigth Path
  /*         double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
               next_x_vals.push_back(car_x + (dist_inc*i)*cos(deg2rad(car_yaw)));
               next_y_vals.push_back(car_y + (dist_inc*i)*sin(deg2rad(car_yaw)));
           } 
    */       

         //Circular Path
        /*  double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for (int i = 0; i < path_size; ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
           }

          if (path_size == 0) {  //No previous points so get the value from localization
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
            } 
          else {  //previous points so get values from last point 
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
           }

         double dist_inc = 0.5;
         for (int i = 0; i < 50-path_size; ++i) {    
                next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
           }
         */

          //Experiment with Frenet  
   /*       double dist_inc = 0.4;// 0.5;
          for (int i = 0; i < 50; ++i) {
             double next_s = car_s+(i+1)*(dist_inc);
             double next_d = 6; //middle lane
             //calculate x,y
             vector<double> next_point;
             next_point= getXY(next_s, next_d, map_waypoints_s, 
                         map_waypoints_x,map_waypoints_y);
             next_x_vals.push_back(next_point[0]);
             next_y_vals.push_back(next_point[1]);     
           }
*/
//Other
/*
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
               double next_s = car_s+(i+1)*dist_inc;
               double next_d = 6; 
               vector<double> xy= getXY(next_s,next_d,
                                         map_waypoints_s,map_waypoints_x,map_waypoints_y);

               next_x_vals.push_back(xy[0]);
               next_y_vals.push_back(xy[1]);
           }
*/

// spline

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);  // the angle in degrees



          if(prev_size<2){  //use the car as starting reference
           //use the two points that make the path tangent

           double prev_car_x = car_x - cos(car_yaw);
           double prev_car_y = car_y - sin(car_yaw);

           ptsx.push_back(prev_car_x);
           ptsx.push_back(car_x);

           ptsy.push_back(prev_car_y);
           ptsy.push_back(car_y);
          }
          else{  //use the previous path's end as starting reference
             //Redefine refence state as previous path end point
 
            ref_x = previous_path_x[prev_size-1];  //end point
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Generate 3 (three) points 
          // evenly spaced , ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane) ,
                                          map_waypoints_s,map_waypoints_x,map_waypoints_y );
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane) ,
                                          map_waypoints_s,map_waypoints_x,map_waypoints_y );
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane) ,
                                          map_waypoints_s,map_waypoints_x,map_waypoints_y );                                                                

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

         //Now we do a transformation for the points to the car point of view

         for(int i=0; i< ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y* sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y* cos(0 - ref_yaw));
         }

         //spline
         tk::spline s;
         //add points
         s.set_points(ptsx,ptsy);

         //the we put the values in next_x_vals and next_y_vals
         //first the previous points
         for(int i=0;i<(previous_path_x.size());i++){
           next_x_vals.push_back(previous_path_x[i]);
           next_y_vals.push_back(previous_path_y[i]);
         }

         double target_x = 30.0;
         double target_y = s(target_x);
         double target_dist = distance(0,0,target_x,target_y);

         double x_add_on= 0;

         double N = (target_dist/(0.02*ref_vel/2.24));
         double mini_target= target_x/N;

         for(int i=0; i<= 50-previous_path_x.size();i++){
               
            //double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on +mini_target; //(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //go to global coords
            x_point =(x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+ y_ref* cos(ref_yaw));

           x_point += ref_x;
           y_point += ref_y;

           next_x_vals.push_back(x_point);
           next_y_vals.push_back(y_point);
         }


//END
// passing
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