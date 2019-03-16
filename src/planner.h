#ifndef PLANNER_H
#define PLANNER_H

#include <array>
#include <vector>
#include <iterator>
#include <cmath>

#include "spline.h"
#include "helpers.h"

// Class responsible for trajectory planning
class TrajectoryPlanner {

    // Struct for managing car state
    struct CarState {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    CarState current_state;

    // Time step between successive waypoints in seconds
    double time_step;

    // Planning horizon in seconds
    double plan_time;

    // Number of time steps in planning horizon
    double num_steps;

    // Speed controller is trying to reach
    double desired_speed;
    double ideal_speed;
    double accel;

    // Current lane id, possible values 0, 1, and 2
    int curr_lane;

    // Map waypoints data
    const std::vector<double> &map_waypoints_s;
    const std::vector<double> &map_waypoints_x;
    const std::vector<double> &map_waypoints_y;

    // Unused portion of previous path
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    // Frenet for end of path
    double end_path_s;
    double end_path_d;

    // Sensor fusion data
    std::vector<std::vector<double>> sensor_fusion;

    // Transition to new state
    void transition_state(int num_prev, double car_s) {
        // Analyze sensor fusion data
        double too_close_dist = 30.0;

        // Determine what state car should transition to
        bool too_close = false;
        bool change_lanes_left = true;
        bool change_lanes_right = true;

        for (auto sf : sensor_fusion)
        {
            double sf_d = sf[6];
            // If the vehicle is in the ego lane and is too close, set too_close to true
            if (in_ego_lane(curr_lane, sf_d))
            {
                double vx(sf[3]), vy(sf[4]);
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s(sf[5]);

                check_car_s += time_step * double(num_prev) * check_speed;

                if ((check_car_s > car_s) && (check_car_s - car_s < too_close_dist))
                {
                    too_close = true;
                }
            }
            // Else, if the vehicle is to the left of the ego lane and is close by, set change_lanes_left to true
            else if (left_of_ego(curr_lane, sf_d))
            {
                double vx(sf[3]), vy(sf[4]);
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s(sf[5]);

                check_car_s += time_step * double(num_prev) * check_speed;

                if (std::fabs(check_car_s - car_s) < too_close_dist)
                {
                    change_lanes_left = false;
                }
            }
            // Else, if the vehicle is to the right of the ego lane and is close by, set change_lanes_right to true
            else if (right_of_ego(curr_lane, sf_d))
            {
                double vx(sf[3]), vy(sf[4]);
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s(sf[5]);

                check_car_s += time_step * double(num_prev) * check_speed;

                if (std::fabs(check_car_s - car_s) < too_close_dist)
                {
                    change_lanes_right = false;
                }
            }
        }

        // If the car is too close,
        if (too_close)
        {
            // first try to change lanes to the left
            if (change_lanes_left && curr_lane > 0)
            {
                curr_lane -= 1;
                desired_speed -= accel;
                // otherwise, try to change lanes to the right
            }
            else if (change_lanes_right && curr_lane < 2)
            {
                curr_lane += 1;
                desired_speed -= accel;
                // or else just slow down
            }
            else
            {
                desired_speed -= accel;
            }
            // If the car is not too close accelerate if we aren't at the ideal speed
        }
        else if (desired_speed < ideal_speed)
        {
            desired_speed += accel;
        }
    }

    public:
    TrajectoryPlanner(
                const std::vector<double> &_map_waypoints_s, 
                const std::vector<double> &_map_waypoints_x, 
                const std::vector<double> &_map_waypoints_y,
                const double _time_step, const double _plan_time)
    : map_waypoints_s(_map_waypoints_s), 
      map_waypoints_x(_map_waypoints_x),
      map_waypoints_y(_map_waypoints_y), 
      time_step(_time_step), plan_time(_plan_time) {

        // Car is stationary to start
        desired_speed = 0.0;

        // Speed limit is 50 miles/hour
        ideal_speed   = mph2mps(49.5);
        accel         = (0.224);

        // Car starts in the middle lane
        curr_lane = 1;

        num_steps = (int) plan_time / time_step;
    }

    // Set current state of the car
    void setCurrentState(const double x, const double y, 
                     const double s, const double d, 
                     const double yaw, const double speed) {
        current_state.x = x;
        current_state.y = y;
        current_state.s = s;
        current_state.d = d;
        current_state.yaw = yaw;
        current_state.speed = speed;
    }

    // Set unused previous path
    void setPreviousPath(const std::vector<double> &prev_path_x, const std::vector<double> &prev_path_y, const double _end_path_s, const double _end_path_d) {
        previous_path_x = prev_path_x;
        previous_path_y = prev_path_y;
        end_path_s      = _end_path_s;
        end_path_d      = _end_path_d;
    }

    // Set sensor fusion data
    void setSensorFusion(const std::vector<std::vector<double>> &_sensor_fusion) {

        sensor_fusion = _sensor_fusion;
    }

    // Plan trajectory
    void plan(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals) {
        
        next_x_vals.reserve(num_steps);
        next_y_vals.reserve(num_steps);

        // Create a vector to hold spline anchor points
        std::vector<double> ptsx;
        std::vector<double> ptsy;

        double prev_x, prev_y, ref_x, ref_y, ref_yaw;
        int num_prev = previous_path_x.size();
        double car_s;
        if (num_prev>0) {
            car_s = end_path_s;
        }

        // Determine what state car should transition to
        bool too_close          = false;
        bool change_lanes_left  = true;
        bool change_lanes_right = true;

        transition_state(num_prev, car_s);

        // Build anchor points for spline fit

        // If there aren't sufficient previous point, extrapolate backwards
        if (num_prev < 2) {
            ref_x   = current_state.x;
            ref_y   = current_state.y;
            ref_yaw = deg2rad(current_state.yaw); 

            prev_x = ref_x - cos(ref_yaw);
            prev_y = ref_y - sin(ref_yaw);

            ptsx.push_back(prev_x);
            ptsx.push_back(current_state.x);

            ptsy.push_back(prev_y);
            ptsy.push_back(current_state.y);
        }
        // Use the last two points on the previous path
        else {
            prev_x = previous_path_x[num_prev-2];
            prev_y = previous_path_y[num_prev-2];

            ref_x   = previous_path_x[num_prev-1];
            ref_y   = previous_path_y[num_prev-1];
            ref_yaw = atan2( ref_y-prev_y, ref_x-prev_x );

            ptsx.push_back(prev_x);
            ptsx.push_back(ref_x); 
            
            ptsy.push_back(prev_y);
            ptsy.push_back(ref_y);
        }

        // Compute look ahead points along the lane at 45,90 and 135 
        double look_dist(45.0);
        int num_looks(3);

        car_s = current_state.s;
        double car_d = laneid2frenet(curr_lane);
        for (int l = 1; l <= num_looks; l++) {
            std::vector<double> look_ahead_pt = getXY(car_s + l*look_dist, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(look_ahead_pt[0]);
            ptsy.push_back(look_ahead_pt[1]);
        }

        // Convert to local frame of reference
        for (int p = 0; p < ptsx.size(); p++) {
            auto local_pt = vehicle2local(ptsx[p], ptsy[p], ref_x, ref_y, ref_yaw);
            ptsx[p] = local_pt[0];
            ptsy[p] = local_pt[1];
        }
        
        // Create the spline fitter using the 5 anchor points
        tk::spline spline_fitter;
        spline_fitter.set_points(ptsx, ptsy);

        // Copy over previous path. This provides continuity to the path
        std::copy(std::begin(previous_path_x), std::end(previous_path_x), std::back_inserter(next_x_vals));
        std::copy(std::begin(previous_path_y), std::end(previous_path_y), std::back_inserter(next_y_vals));

        // Compute the target distance to the 1st look ahead point
        double target_x(look_dist);
        double target_y = spline_fitter(target_x);
        double target_dist = sqrt( target_x*target_x + target_y*target_y );
        double x_add_on = 0.0;

        // For as many points as are left after copying over the previous path,
        // compute and add points along the spline at intervals that maintain the
        // desired speed.
        for (int p = 1; p <= num_steps-num_prev; p++) {

            // Find the number of points to maintain the desired speed
            double N = target_dist / (time_step * desired_speed);

            // Find the point along the spline
            double x_point = x_add_on + target_x/N;
            double y_point = spline_fitter(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Convert the point back to global reference frame
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }
    }

};

#endif //PLANNER_H