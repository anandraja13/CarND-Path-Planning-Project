#ifndef PLANNER_H
#define PLANNER_H

#include <array>
#include <vector>
#include <iterator>

#include "spline.h"
#include "helpers.h"

class TrajectoryPlanner {

    struct CarState {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    CarState current_state;

    double time_step;
    double plan_time;
    double num_steps;

    const std::vector<double> &map_waypoints_s;
    const std::vector<double> &map_waypoints_x;
    const std::vector<double> &map_waypoints_y;

    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;

    std::vector<std::vector<double>> sensor_fusion;

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

        num_steps = (int) plan_time / time_step;
    }

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

    void setPreviousPath(const std::vector<double> &prev_path_x, const std::vector<double> &prev_path_y, const double _end_path_s, const double _end_path_d) {
        previous_path_x = prev_path_x;
        previous_path_y = prev_path_y;
        end_path_s      = _end_path_s;
        end_path_d      = _end_path_d;
    }

    void setSensorFusion(const std::vector<std::vector<double>> &_sensor_fusion) {

        sensor_fusion = _sensor_fusion;
    }

    void plan(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals) {
        
        next_x_vals.reserve(num_steps);
        next_y_vals.reserve(num_steps);

        // Create a vector to hold spline anchor points
        std::vector<double> ptsx;
        std::vector<double> ptsy;

        double desired_speed = mph2mps(49.5);

        double prev_x, prev_y, ref_x, ref_y, ref_yaw;
        int num_prev = previous_path_x.size();
        double car_s;
        if (num_prev>0) {
            car_s = end_path_s;
        }

        bool too_close = false;
        double too_close_dist = 30.0;
        for (auto sf : sensor_fusion) {

            double sf_d = sf[6];
            if (in_my_lane(1, sf_d)) {
                double vx(sf[3]), vy(sf[4]);
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sf[5];

                check_car_s += time_step * double(num_prev) * check_speed;

                if ( (check_car_s > car_s) && (check_car_s-car_s < too_close_dist) ) {
                    desired_speed = mph2mps(29.5);
                    too_close = true;
                }
            }
        }

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

        // Compute look ahead points along the lane at 30,60 and 90
        double look_dist(30.0);
        int num_looks(3);

        car_s = current_state.s;
        double car_d = laneid2frenet(1);
        for (int l = 1; l <= num_looks; l++) {
            std::vector<double> look_ahead_pt = getXY(car_s + l*look_dist, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(look_ahead_pt[0]);
            ptsy.push_back(look_ahead_pt[1]);
        }

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

        /*
        double dist_inc = desired_speed * time_step;

        // Make the car stay in the same lane
        for (int n = 0; n < num_steps; n++) {
            double next_s = current_state.s + dist_inc* (n+1);
            double next_d = laneid2frenet(1);
            std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

            next_x_vals.push_back( xy[0] );
            next_y_vals.push_back( xy[1] );
        }
        */
    }

};

#endif //PLANNER_H