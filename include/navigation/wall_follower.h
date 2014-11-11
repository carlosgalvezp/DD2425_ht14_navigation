#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H
#include "ros/ros.h"
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Pose2D.h>
#include "ras_utils/controller.h"
#include "ras_utils/basic_node.h"
#include <navigation/robot_turning.h>

#define MAX_DIST_FRONT_WALL     280       // [cm] Change this!!
#define MAX_DIST_SIDE_WALL     20       // [cm]

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

struct WF_PARAMS
{
   bool debug_print;
   double wanted_distance;
   double kp_w, kd_w, ki_w;
   double wanted_v;
   double stopping_error_margin, stopped_turn_increaser, slow_start_increaser;
   double kp_d_w, kd_d_w, ki_d_w;
};

class Wall_follower : rob::BasicNode
{
public:

    Wall_follower();
    void setParams(const WF_PARAMS &params, const RT_PARAMS &rt_params);
    void compute_commands(const geometry_msgs::Pose2D::ConstPtr &odo_msg, const ras_arduino_msgs::ADConverter::ConstPtr& msg , double &v, double &w);

private:
    Robot_turning robot_turner;
    RT_PARAMS rt_params;

    bool debug_print;

    // adc data from IR sensors
    double wanted_distance;

    double stopping_error_margin;

    double stopped_turn_increaser;
    double slow_start_increaser;

    double wanted_v;
    double actual_v;

    double w;
    // In which side is wall
    bool wall_is_right;

    ros::NodeHandle n_;
    ros::Publisher twist_pub_;
    ros::Subscriber adc_sub_;
    // Callback func when adc data recieved
    void compute_commands(double d_front, double d_back, bool wall_is_right, double &v, double &w);

    void addParams();

    double compute_turning_angle(double d_front, double d_right_front, double d_right_back,
                                    double d_left_front, double d_left_back);

    Controller controller_w;
    double kp_w, ki_w, kd_w;
    Controller controller_wall_distance;
    double kp_d_w, kd_d_w, ki_d_w;
    bool turning_;

    bool is_wall_in_front(double d_front);
};
#endif // WALL_FOLLOWER_H
