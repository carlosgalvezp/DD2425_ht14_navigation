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

#define MAX_DIST_FRONT_WALL     10      // [cm]
#define MAX_DIST_SIDE_WALL     25       // [cm]

#define DEFAULT_DEBUG_PRINT             true
#define DEFAULT_WANTED_DISTANCE         16.0
#define DEFAULT_KP_W                    0.02    //0.005
#define DEFAULT_KD_W                    0.1     //0.01
#define DEFAULT_KI_W                    0.0     // 0.00001
#define DEFAULT_LINEAR_SPEED            0.13
#define DEFUALT_WALL_IS_RIGHT           true
#define DEFAULT_STOPPING_ERROR_MARGIN   6.0
#define DEFUALT_STOPPED_TURN_INCREASER  5.0
#define DEFUALT_SLOW_START_INCREASER    0.1     // in percentage, 1 means full throttle from start

#define DANGEROUSLY_CLOSE_LIMIT             7.5
#define DANGEROUSLY_CLOSE_BACKUP_DISTANCE   7
#define DANGEROUSLY_CLOSE_BACKUP_SPEED      -0.1




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

    void run(double &v, double &w, double d_right_front, double d_left_front, double d_right_back, double d_left_back)
    {
        d_right_front_ = d_right_front;
        d_left_front_ = d_left_front;
        d_right_back_ = d_right_back;
        d_left_back_ = d_left_back;

        if(canFollowAWall()) {
            // Set wanted distance
            if(!wanted_distance_recently_set_)
            {
                ROS_ERROR("Setting wanted distance");
                wanted_distance_ = getDistanceToClosestWall();
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

    double d_right_front_, d_left_front_, d_right_back_, d_left_back_;

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
        return alignToWallAndWallDistance(shouldPrioritizeRightWall(), increased_strength);
    }

    /*
        Will try to align to the wall to wall, using parameter w.
        Throws exception if no wall is usable.
    */
    double alignToWall(double increased_strength)
    {
        return alignToWall(shouldPrioritizeRightWall(), increased_strength);
    }

    /*
        Will try to align using only distance to wall, using parameter w.
        Throws exception if no wall is usable.
    */
    double alignUsingWallDistance(double increased_strength)
    {
        return alignToWallAndWallDistance(shouldPrioritizeRightWall(), increased_strength);
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
            distance_front = d_right_front_;
            distance_back = d_right_back_;
            sign = 1;
        }
        else
        {
            distance_front = d_left_front_;
            distance_back = d_left_back_;
            sign = -1;
        }
    }

    double alignUsingWallDistance(bool wall_is_right, double increased_strength)
    {
        if(!canFollowWall(wall_is_right)) {
            std::string error_msg = "Trying to align using wall distance to: " + boost::lexical_cast<std::string>(wall_is_right) + "(true = right wall) while when we can't! Check code!";
            ROS_ERROR(error_msg.c_str());
        }

        int sign;
        double distance_front, distance_back;

        getDistanceFrontAndBack(wall_is_right, distance_front, distance_back, sign);

        double avarage_distance_to_wall = (distance_front + distance_back) / 2.0;

        controller_wall_distance_.setData(wanted_distance_, avarage_distance_to_wall);
        return controller_wall_distance_.computeControl() *  increased_strength * sign;
    }

    double alignToWall(bool wall_is_right, double increased_strength) {
        if(!canFollowAWall()) {
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


    bool canFollowWall(double d_front, double d_back)
    {
        return d_front < MAX_DIST_SIDE_WALL && d_back < MAX_DIST_SIDE_WALL;
    }

    bool canFollowWall(bool right_wall)
    {
        if(right_wall)
        {
            return canFollowRightWall();
        }else
        {
            return canFollowLeftWall();
        }
    }

    bool canFollowAWall()
    {
        return canFollowLeftWall() || canFollowRightWall();
    }

    bool canFollowLeftWall()
    {
        return canFollowWall(d_left_front_, d_left_back_);
    }

    bool canFollowRightWall()
    {
        return canFollowWall(d_right_front_, d_right_back_);
    }

    double getDistanceToLeftWall()
    {
        return  0.5*(d_left_back_ + d_left_front_);
    }

    double getDistanceToRightWall()
    {
        return 0.5*(d_right_back_ + d_right_front_);
    }

    double getDistanceToClosestWall()
    {
        return fmin(getDistanceToLeftWall(), getDistanceToRightWall());
    }

    bool shouldPrioritizeRightWall()
    {
        if(!canFollowRightWall()) return false;
        if(!canFollowLeftWall()) return true;
        return getDistanceToRightWall() < getDistanceToLeftWall();
    }
};
#endif // WALL_FOLLOWER_H
