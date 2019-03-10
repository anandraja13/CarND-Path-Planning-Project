#ifndef PLANNER_H
#define PLANNER_H

#include <array>

#include "helpers.h"

class trajectoryPlanner {

    std::array<double, 6> current_state;
    double desired_speed;
    double time_step;
    double plan_time;
    double num_steps;

    const std::vector<double> &map_waypoints_s;
    const std::vector<double> &map_waypoints_dx;
    const std::vector<double> &map_waypoints_dy;

    public:
    trajectoryPlanner(
                const std::vector<double> &_map_waypoints_s, 
                const std::vector<double> &_map_waypoints_dx, 
                const std::vector<double> &_map_waypoints_dy,
                const double _time_step, const double _plan_time)
    : map_waypoints_s(_map_waypoints_s), 
      map_waypoints_dx(_map_waypoints_dx),
      map_waypoints_dy(_map_waypoints_dy), 
      time_step(_time_step), plan_time(_plan_time) {
        desired_speed = mph2mps(50.0);
        num_steps     = (int) plan_time / time_step;
    }

    void setCurrentState(const double x, const double y, 
                         const double s, const double d, 
                         const double yaw, const double speed) {
        current_state[0] = x;
        current_state[1] = y;
        current_state[2] = s;
        current_state[3] = d;
        current_state[4] = yaw;
        current_state[5] = speed;
    }

    void plan(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals) {
        
        next_x_vals.reserve(num_steps);
        next_y_vals.reserve(num_steps);

        double dist_inc = desired_speed * time_step;
        double lane_width = 4;

        // Make the car stay in the same lane
        for (int n = 0; n < num_steps; n++) {
            double next_s = current_state[2] + dist_inc* (n+1);
            double next_d = 1.5 * lane_width;
            std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

            next_x_vals.push_back( xy[0] );
            next_y_vals.push_back( xy[1] );
        }

         /*   
        // Make the car move in a straight line
        for (int n = 0; n < num_steps; n++) {
            next_x_vals.push_back( current_state[0] + dist_inc * n * cos(deg2rad(current_state[4])) );
            next_y_vals.push_back( current_state[1] + dist_inc * n * sin(deg2rad(current_state[4])) );
        }
        */
    }

};

#endif //PLANNER_H