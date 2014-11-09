#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_utils/controller.h"
#include "ras_utils/basic_node.h"

#include <math.h>

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE 1000

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

class Wall_follower : rob::BasicNode
{
public:

    Wall_follower();
    void run();

private:
    bool debug_print;

    // adc data from IR sensors
    double distance_front;
    double distance_back;

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
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);

    void addParams();

    Controller controller_w;
    double kp_w, ki_w, kd_w;
};

int main (int argc, char* argv[])
// ** Init node
{
    ros::init(argc, argv, "wall_follower");

    // ** Create wall follower object
    Wall_follower wf;

    // ** Run
    wf.run();
}

Wall_follower::Wall_follower() : distance_front(0), distance_back(0), w(0), actual_v(0)
{
    addParams();
    print_params();

    // Publisher
    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", QUEUE_SIZE);
    // Subscriber
    adc_sub_ = n_.subscribe("/arduino/adc", QUEUE_SIZE,  &Wall_follower::adcCallback, this);

    controller_w = Controller(kp_w,kd_w,ki_w, 10);
}

void Wall_follower::addParams()
{
    add_param("wf/debug_print", debug_print, DEFAULT_DEBUG_PRINT);
    add_param("wf/wanted_distance", wanted_distance, DEFAULT_WANTED_DISTANCE);
    add_param("wf/W/KP", kp_w, DEFAULT_KP_W);
    add_param("wf/W/KD", kd_w, DEFAULT_KD_W);
    add_param("wf/W/KI", ki_w, DEFAULT_KI_W);
    add_param("wf/linear_speed", wanted_v, DEFAULT_LINEAR_SPEED);
    add_param("wf/wall_is_right", wall_is_right, DEFUALT_WALL_IS_RIGHT);
    add_param("wf/stopping_error_margin", stopping_error_margin, DEFAULT_STOPPING_ERROR_MARGIN);
    add_param("wf/stopped_delta_increaser", stopped_turn_increaser, DEFUALT_STOPPED_TURN_INCREASER);
    add_param("wf/slow_start_increaser", slow_start_increaser, DEFUALT_SLOW_START_INCREASER);
}

void Wall_follower::run()
{
    // ** Publish data
    ros::Rate loop_rate(PUBLISH_RATE);

    double delta;
    double diff;
    double avarage_distance_to_wall;
    while(ros::ok())
    {
        avarage_distance_to_wall = (distance_front + distance_back) / 2.0;
        diff = distance_front - distance_back;
        delta = diff + (avarage_distance_to_wall - wanted_distance);

        if(wall_is_right)
        {
            controller_w.setData(0, delta);
            w = controller_w.computeControl();
        }
        else
        {
            controller_w.setData(0, -delta);
            w = controller_w.computeControl();
        }

        if(fabs(diff) > stopping_error_margin) {
            //we are way of from our wanted direction, stop the motion forward!
            //also increase the power of turning for the robot, needed cause otherwise the robot is too weak.
            actual_v = 0;
            w *= stopped_turn_increaser;
        } else {
            actual_v = fmin(actual_v + wanted_v * slow_start_increaser, wanted_v);
        }

        geometry_msgs::Twist msg;

        msg.linear.x = actual_v;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = w;

        if(debug_print) {
            print("wall_is_right", wall_is_right);
            std::vector<std::string> info({"v", "w", "distance_front", "distance_back", "Avarage_distance", "Wanted_distance", "Diff", "Delta"});
            std::vector<double> data({actual_v, w, distance_front, distance_back, avarage_distance_to_wall, wanted_distance, diff, delta});
            print(info, data);
        }

        twist_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Exiting...\n";
}

void Wall_follower::adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg)
{

    if(wall_is_right)
    {
        distance_front = RAS_Utils::shortSensorToDistanceInCM(msg->ch4);
        distance_back = RAS_Utils::shortSensorToDistanceInCM(msg->ch3);
    }
    else
    {
        distance_front = RAS_Utils::shortSensorToDistanceInCM(msg->ch1);
        distance_back = RAS_Utils::shortSensorToDistanceInCM(msg->ch2);
    }
}
