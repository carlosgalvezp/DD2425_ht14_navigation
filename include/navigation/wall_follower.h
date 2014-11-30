#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H
#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <ras_utils/controller.h>
#include <ras_utils/basic_node.h>
#include <navigation/robot_backer.h>
#include <math.h>
#include <ras_utils/ras_sensor_utils.h>

#define MAX_DIST_FRONT_WALL     10      // [cm]
#define MAX_DIST_SIDE_WALL     25       // [cm]

#define DEFAULT_DEBUG_PRINT             true
#define DEFAULT_WANTED_DISTANCE         16.0
#define DEFAULT_KP_W                    0.02    //0.005
#define DEFAULT_KD_W                    0.1     //0.01
#define DEFAULT_KI_W                    0.0     // 0.00001
#define DEFAULT_LINEAR_SPEED            0.13
#define DEFUALT_WALL_IS_RIGHT           true

struct WF_PARAMS
{
   bool debug_print;
   double wanted_distance;
   double kp_w, kd_w, ki_w;
   double wanted_v;
   double kp_d_w, kd_d_w, ki_d_w;
};

class WallFollower : rob::BasicNode
{
public:

    WallFollower() : wanted_distance_recently_set_(false), need_alignment(false) {}
    void setParams(const WF_PARAMS &params)
    {
        debug_print_ = params.debug_print;
        wanted_distance_ = params.wanted_distance;
        wanted_v_ = params.wanted_v;

        controller_align_ = Controller(params.kp_w, params.kd_w, params.ki_w);
        controller_wall_distance_ = Controller(params.kp_d_w, params.kd_d_w, params.ki_d_w);
    }

    void run(double &v, double &w, RAS_Utils::sensors::SensorDistances sd)
    {
        this->sd = sd;

        if(RAS_Utils::sensors::canFollowAWall(sd)) {
            // Set wanted distance
            if(!wanted_distance_recently_set_)
            {
                ROS_ERROR("Setting wanted distance");
                wanted_distance_ = RAS_Utils::sensors::getDistanceToClosestWall(sd);
                wanted_distance_recently_set_ = true;
            }

            // **
            w = alignToWallAndWallDistance();
            v = wanted_v_;
            return;
        }


        // ** Ask boss to decide what to do
        ROS_WARN("Wall follower have no wall to follow. Trying to go straight");
        v = wanted_v_;
        w = 0.0;
        wanted_distance_recently_set_ = false;
    }

    void resetWantedDistance()
    {
        wanted_distance_recently_set_ = false;
    }

private:

    RAS_Utils::sensors::SensorDistances sd;

    bool wanted_distance_recently_set_;

    double wanted_distance_;

    double wanted_v_;

    bool debug_print_;

    bool need_alignment;

    // In which side is wall
    bool wall_is_right_;

    Controller controller_align_;
    Controller controller_wall_distance_;

    bool isWallCloseFront(double d_front);

    /*
        Will try to align to the wall and take distance to wall into account, using parameter w.
        Throws exception if no wall is usable.
    */
    double alignToWallAndWallDistance(double increased_strength = 1.0)
    {
        return alignToWallAndWallDistance(RAS_Utils::sensors::shouldPrioritizeRightWall(sd), increased_strength);
    }

    /*
        Will try to align to the wall to wall, using parameter w.
        Throws exception if no wall is usable.
    */
    double alignToWall(double increased_strength)
    {
        return alignToWall(RAS_Utils::sensors::shouldPrioritizeRightWall(sd), increased_strength);
    }

    /*
        Will try to align using only distance to wall, using parameter w.
        Throws exception if no wall is usable.
    */
    double alignUsingWallDistance(double increased_strength)
    {
        return alignToWallAndWallDistance(RAS_Utils::sensors::shouldPrioritizeRightWall(sd), increased_strength);
    }


    double alignToWallAndWallDistance(bool wall_is_right, double increased_strength) {

        double w_align_to_wall_distance = alignUsingWallDistance(wall_is_right, increased_strength);
        double w_align_to_wall = alignToWall(wall_is_right, increased_strength);

        ROS_WARN("%f: %f", w_align_to_wall_distance, w_align_to_wall);
        return w_align_to_wall_distance + w_align_to_wall;
    }

    void getDistanceFrontAndBack(bool wall_is_right, double &distance_front, double &distance_back, int &sign)
    {
        if(wall_is_right) {
            distance_front = sd.right_front_;
            distance_back = sd.right_back_;
            sign = 1;
        }
        else
        {
            distance_front = sd.left_front_;
            distance_back = sd.left_back_;
            sign = -1;
        }
    }

    double alignUsingWallDistance(bool wall_is_right, double increased_strength)
    {
        if(!RAS_Utils::sensors::canFollowWall(sd, wall_is_right)) {
            std::string error_msg = "Trying to align using wall distance to: " + boost::lexical_cast<std::string>(wall_is_right) + "(true = right wall) while when we can't! Check code!";
            ROS_ERROR("%s",error_msg.c_str());
        }

        int sign;
        double distance_front, distance_back;

        getDistanceFrontAndBack(wall_is_right, distance_front, distance_back, sign);

        double avarage_distance_to_wall = (distance_front + distance_back) / 2.0;

        controller_wall_distance_.setData(wanted_distance_, avarage_distance_to_wall);
        return controller_wall_distance_.computeControl() *  increased_strength * sign;
    }

    double alignToWall(bool wall_is_right, double increased_strength) {
        if(!RAS_Utils::sensors::canFollowAWall(sd)) {
            throw std::runtime_error( "Trying to follow wall:" + boost::lexical_cast<std::string>(wall_is_right) + "(true = right wall)! Check code!" );
        }

        int sign;
        double distance_front, distance_back;

        getDistanceFrontAndBack(wall_is_right, distance_front, distance_back, sign);

        if(debug_print_) ROS_INFO("Following %s wall", (wall_is_right) ? "right" : "left");

        double diff = distance_front - distance_back;
        double delta = diff * increased_strength; // + KP_DIST_WALL*diff_distance_wall;

        controller_align_.setData(0, delta * sign);
        return controller_align_.computeControl();
    }



};
#endif // WALL_FOLLOWER_H
