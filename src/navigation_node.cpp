#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_utils/controller.h"
#include "ras_utils/basic_node.h"
#include <ras_srv_msgs/Command.h>
#include <ras_utils/ras_names.h>
#include <navigation/wall_follower.h>

#define QUEUE_SIZE 10
#define PUBLISH_RATE 10

class Navigation : rob::BasicNode
{
public:

    Navigation();
    void run();

private:
    // ** Publishers and subscribers
    ros::Publisher twist_pub_;
    ros::Subscriber adc_sub_;

    // ** Services
    ros::ServiceClient srv_out_;
    ros::ServiceServer srv_in_;

    // ** Callback func when adc data recieved
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);
    // Service callback
    bool srvCallback(ras_srv_msgs::Command::Request &req, ras_srv_msgs::Command::Response &resp);

    void addParams();

    Controller controller_w;
    Wall_follower wall_follower;
    double kp_w, ki_w, kd_w;

    int mode_;
    ras_arduino_msgs::ADConverter::ConstPtr adc_data_;
};

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, NODE_NAVIGATION);

    // ** Create wall follower object
    Navigation navigation;

    // ** Run
    navigation.run();
}

Navigation::Navigation() : mode_(RAS_Names::Navigation_Modes::NAVIGATION_WALL_FOLLOW)
{
    addParams();
    print_params();

    // Publisher
    twist_pub_ = n.advertise<geometry_msgs::Twist>(TOPIC_MOTOR_CONTROLLER_TWIST, QUEUE_SIZE);
    // Subscriber
    adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC, QUEUE_SIZE,  &Navigation::adcCallback, this);
    // Service callback
    srv_in_ = n.advertiseService(SRV_NAVIGATION_IN, &Navigation::srvCallback, this);

    controller_w = Controller(kp_w,kd_w,ki_w, 10);
}

void Navigation::addParams()
{
    WF_PARAMS params;
    add_param("wf/debug_print", params.debug_print, DEFAULT_DEBUG_PRINT);
    add_param("wf/wanted_distance", params.wanted_distance, DEFAULT_WANTED_DISTANCE);
    add_param("wf/W/KP", params.kp_w, DEFAULT_KP_W);
    add_param("wf/W/KD", params.kd_w, DEFAULT_KD_W);
    add_param("wf/W/KI", params.ki_w, DEFAULT_KI_W);
    add_param("wf/linear_speed", params.wanted_v, DEFAULT_LINEAR_SPEED);
//    add_param("wf/wall_is_right", wall_is_right, DEFUALT_WALL_IS_RIGHT);
    add_param("wf/stopping_error_margin", params.stopping_error_margin, DEFAULT_STOPPING_ERROR_MARGIN);
    add_param("wf/stopped_delta_increaser", params.stopped_turn_increaser, DEFUALT_STOPPED_TURN_INCREASER);
    add_param("wf/slow_start_increaser", params.slow_start_increaser, DEFUALT_SLOW_START_INCREASER);
    wall_follower.setParams(params);
}

void Navigation::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);
    while(ros::ok())
    {
        double v, w;
        // ** Compute velocity commands
        switch(mode_)
        {
            case RAS_Names::Navigation_Modes::NAVIGATION_WALL_FOLLOW:
                ROS_INFO("[Navigation] Wall following");
                wall_follower.compute_commands(adc_data_, v, w);
                break;

            case RAS_Names::Navigation_Modes::NAVIGATION_GO_OBJECT:
                ROS_INFO("[Navigation] Go to object");

            break;

            case RAS_Names::Navigation_Modes::NAVIGATION_STOP:
                ROS_INFO("[Navigation] STOP");
                v = 0;
                w = 0;
                break;
        }

        // ** Publish
        geometry_msgs::Twist msg;

        msg.linear.x = v;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = w;

        twist_pub_.publish(msg);
        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Exiting...\n";
}

void Navigation::adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg)
{
    adc_data_ = msg;
}

bool Navigation::srvCallback(ras_srv_msgs::Command::Request &req, ras_srv_msgs::Command::Response &resp)
{
    ROS_INFO("Navigation receives command: %ld", req.command);
    mode_ = req.command;
    resp.result = 1;
    return true;
}
