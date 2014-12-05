#ifndef ROBOT_ODOMETRY_ALIGNER_H
#define ROBOT_ODOMETRY_ALIGNER_H

#include <ras_utils/ras_utils.h>
#include <ras_utils/ras_sensor_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_utils/occupancy_map_utils.h>

#define NR_SENSORS_TO_ALIGN_COUNT   10
#define MAXIMUM_ANGLE_CHANGE        M_PI/20.0  // about 10 degrees
#define ANGLE_CHUNK                 M_PI/300.0 // about 0.6 degrees

class RobotOdometryAligner
{

public:
    RobotOdometryAligner() : currently_aligning_(false) {}

    void activate(RAS_Utils::sensors::SensorDistances & sd) {
        currently_aligning_ = true;
        align_count_ = 0;
        if(RAS_Utils::sensors::canFollowRightWall(sd))
        {
            right_wall_ = true;
        } else
        {
            right_wall_ = false;
        }
        total_front_sensor_value_ = 0;
        total_back_sensor_value_ = 0;
    }

    void run(double &v, double &w, double robot_angle, double robot_x_pos, double robot_y_pos, RAS_Utils::sensors::SensorDistances & sd, const nav_msgs::OccupancyGrid & occ_grid)
    {
        final_angle_ = robot_angle;
        //stand still while doing this
        v = 0;
        w = 0;

        if(right_wall_ && RAS_Utils::sensors::canFollowRightWall(sd))
        {
            // Something is strange, abort
           currently_aligning_ = false;
           return;
        }

        if(!right_wall_ && RAS_Utils::sensors::canFollowLeftWall(sd))
        {
            // Something is strange, abort
           currently_aligning_ = false;
           return;
        }

        if(right_wall_)
        {
            total_front_sensor_value_ += sd.right_front_;
            total_back_sensor_value_  += sd.right_back_;
        }

        if(!right_wall_)
        {
            total_front_sensor_value_ += sd.left_front_;
            total_back_sensor_value_ += sd.left_back_;
        }

        align_count_++;


        if(align_count_ == NR_SENSORS_TO_ALIGN_COUNT)
        {
            double new_angle = calculateOptimalAngle(robot_angle, robot_x_pos, robot_y_pos, (total_front_sensor_value_ / NR_SENSORS_TO_ALIGN_COUNT) / 100.0, (total_back_sensor_value_ / NR_SENSORS_TO_ALIGN_COUNT) / 100.0, right_wall_, occ_grid);
            currently_aligning_ = false;
            final_angle_ = new_angle;
        }
    }

    bool isActive()
    {
        return currently_aligning_;
    }

private:
    bool currently_aligning_;
    bool right_wall_;
    int align_count_;
    double total_front_sensor_value_;
    double total_back_sensor_value_;
    double final_angle_;

    double calculateOptimalAngle(double robot_angle, double robot_x_pos, double robot_y_pos, double front_sensor_dist, double back_sensor_dist, bool right_wall, const nav_msgs::OccupancyGrid & occ_grid)
    {
        double lowest_diff = 1000000;
        double best_angle = robot_angle;
        double sensor_diff = front_sensor_dist - back_sensor_dist;
        for(double test_angle = robot_angle - MAXIMUM_ANGLE_CHANGE; test_angle <= robot_angle + MAXIMUM_ANGLE_CHANGE; test_angle += ANGLE_CHUNK)
        {
            double sensor_x_pos, sensor_y_pos;
            calculateSensorXY(sensor_x_pos, sensor_y_pos, robot_x_pos, robot_y_pos, test_angle, right_wall, true);
            double front_distance = getDistanceToWall(sensor_x_pos, sensor_y_pos, test_angle + RAS_Utils::sensors::getShortSensorAngle(right_wall), occ_grid);

            calculateSensorXY(sensor_x_pos, sensor_y_pos, robot_x_pos, robot_y_pos, test_angle, right_wall, false);
            double back_distance = getDistanceToWall(sensor_x_pos, sensor_y_pos, test_angle + RAS_Utils::sensors::getShortSensorAngle(right_wall), occ_grid);

            double map_diff = front_distance - back_distance;

            double sensor_map_diff = fabs(sensor_diff - map_diff);
            if(sensor_map_diff < lowest_diff)
            {
                //found a more accurate angle
                best_angle = test_angle;
                lowest_diff = sensor_map_diff;
            }
        }
    }

    double getDistanceToWall(double sensor_x_pos, double sensor_y_pos, double angle, const nav_msgs::OccupancyGrid & occ_grid)
    {
        double stepper = 0.1;
        for(double current_distance = stepper; current_distance < 30; current_distance += stepper)
        {
            double new_x_pos = sensor_x_pos + cos(angle) * current_distance;
            double new_y_pos = sensor_y_pos + sin(angle) * current_distance;
            bool pos_is_wall = RAS_Utils::occ_grid::isWall(occ_grid, new_x_pos, new_y_pos);
            if(pos_is_wall)
            {
                return current_distance;
            }
        }
        return -1;
    }

     double calculateSensorXY(double & sensor_x_pos, double & sensor_y_pos, double robot_x_pos, double robot_y_pos, double robot_angle, bool right_wall, bool front)
     {
         double sensor_angle = robot_angle + RAS_Utils::sensors::getShortSensorAngleCenterOffset(right_wall, front);

        sensor_x_pos = robot_x_pos + cos(sensor_angle);
        sensor_y_pos = robot_y_pos + sin(sensor_angle);

     }

     double getFinalAngle()
     {
        return final_angle_;
     }
};

#endif // ROBOT_ODOMETRY_ALIGNER_H
