#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
// x,y is the current position 
// maps_x and maps_y are the waypoints
// returns the index to the closes waypoint so the closest waypoint is
// maps_x[r],maps_y[r]
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint (that means always looking forward)
// x,y is the current position
// perhaps theta is the current heading??
// maps_x and maps_y are the waypoints
// it actually returns the same as closest way point  if the difference in angles
// is less than 90
// if it is greater, (that means it is behind you??) it retuns the next one or 0
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y); //gets the index to the closes waypoint to x,y

  double map_x = maps_x[closestWaypoint]; //this is the waypoint
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x)); //here calculate the heading

  double angle = fabs(theta-heading);  // the difference in angle 
  angle = std::min(2*pi() - angle, angle); //correction

  if (angle > pi()/2) { //if the angle is greater than 90 (behind you)
    ++closestWaypoint;  //return the next one
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y); //generates the "next" waypoint

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


bool avoid_collisions(vector<vector<double>> sensor_fusion, int &lane, int prev_size, double tip,double a_s){

   bool left_road_open= true, right_road_open= true;
   bool too_close=false;

   double left_space=7000;
   double right_space=7000;

   if(lane==0)
        left_road_open = false;
   if(lane==2)
        right_road_open = false;

   //Calculate proximity and availability
   for (int i=0; i< sensor_fusion.size();i++){ // For all the cars
        float d = sensor_fusion[i][6];  //the d of the car
        //get the velocity 
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];

        double check_speed = distance(0,0,vx,vy); //calculate the magnitude of speed
        double check_car_s = sensor_fusion[i][5];  //The longitudinal position of the car

        check_car_s += ((double)prev_size*0.02*check_speed); //project the s value outwards in time

        //TODO continue implementing the too close and left right space algorithm

        double left_distance, right_distance;
        //if(d<(2+4*lane+2)&&d>(2+4*lane-2))

        //if(d<(2+4*lane-2)&&(d>(4*lane-4 ))&&lane>0)

        if (d<(2+4*lane-2)&&(d>(4*lane-4 ))&&lane>0){    // car is the left neighbor and egocar is not in 0
             //-check the position of the car  (done in check_car_s)
             if(check_car_s>tip){  //car is ahead
                left_distance = check_car_s-tip;
                if(left_distance<30){   //lef_distance < 30
                  // too close
                  left_road_open=false;
                }
                else{
                  if(left_distance<left_space){
                    left_space=left_distance;
                  }
                }

             }
             else{  //the car is behind
              //left_distance = check_car_s-tip;  //negative
              left_distance = a_s- check_car_s ;
              if(left_distance<30){
                 left_road_open = false;
              }
              //we dont modify left_space here
             }

        }
        else if(d>(2+4*lane+2)&&(d<(4*lane+8))&&lane<2){  // car is in the right neighbor and  egocar is not 2
          //-check the position of the check_car
          if(check_car_s>tip){  //car is ahead
             right_distance =  check_car_s-tip; 
             if(right_distance<30){
               right_road_open = false;
             }
             else{
               if(right_distance<right_space){
                 right_space= right_distance;
               }
             }
          }
          else{// car is behind
             right_distance = a_s -check_car_s;//  check the distance
             if(right_distance<30){
               right_road_open = false;
             }
          }
        }

        else if(d<(2+4*lane+2)&&d>(2+4*lane-2)) {
          //check if it is too close 
           if((check_car_s>tip) && ((check_car_s-tip)<30)){
             too_close=true;
           }

        }


   }



  // It is too close so let's decide wheter to change lanes
  if(too_close){
       if((lane==0) && (right_road_open)){
         lane=1;  //Move to the right
       }
       else if((lane==2) && (left_road_open)){
         lane=1;  //Move to the left
       }
       else if (lane==1){
          if((left_road_open)&&(right_road_open)){
            //Choose
              if(left_space>right_space){
                lane=0;
              }
              else{
                lane=2;
              }
          }
          else if(left_road_open){
            lane = 0;
          }
          else if(right_road_open){
            lane = 2;
          }
       }


  }



   return too_close;
}

#endif  // HELPERS_H