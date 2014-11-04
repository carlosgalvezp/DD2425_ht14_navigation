#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_utils/controller.h"

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE 1000
#define DEFAULT_WANTED_DISTANCE 15.0

class Wall_follower
{
public:

    Wall_follower(const ros::NodeHandle& n);
    void run();

private:
    // adc data from IR sensors
    double distance_front;
    double distance_back;

    double wanted_distance;

    double v;
    double w;
    // In which side is wall
    bool wall_is_left;

    ros::NodeHandle n_;
    ros::Publisher twist_pub_;
    ros::Subscriber adc_sub_;
    // Callback func when adc data recieved
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);

    Controller controller_w;
    double kp_w, ki_w, kd_w;
};

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle n;

    // ** Create wall follower object
    Wall_follower wf(n);

    // ** Run
    wf.run();
}

Wall_follower::Wall_follower(const ros::NodeHandle &n)
    : n_(n)
{
    // initial values
    distance_front = 0;
    distance_back = 0;
    wanted_distance = DEFAULT_WANTED_DISTANCE;
    w = 0;

    // Publisher
    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", QUEUE_SIZE);
    // Subscriber
    adc_sub_ = n_.subscribe("/arduino/adc", QUEUE_SIZE,  &Wall_follower::adcCallback, this);

    // params from launch file
    n_.getParam("Wall_follower/W/KP", kp_w);
    n_.getParam("Wall_follower/W/KD", kd_w);
    n_.getParam("Wall_follower/W/KI", ki_w);

    n_.getParam("Wall_follower/linear_speed", v);
    n_.getParam("Wall_follower/wall_is_left", wall_is_left);

    controller_w = Controller(kp_w,kd_w,ki_w, 10);
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

        if(wall_is_left)
        {
            controller_w.setData(0, -delta);
            w = controller_w.computeControl();
        }
        else
        {
            controller_w.setData(0, delta);
            w = controller_w.computeControl();
        }

        geometry_msgs::Twist msg;

        msg.linear.x = v;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = w;

        std::cout <<"Commands (v,w): "<< v << ","<<w
                 << " distance data: " << distance_front <<" "<< distance_back <<std::endl;

        twist_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Exiting...\n";
}

void Wall_follower::adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg)
{

    if(wall_is_left)
    {
        distance_front = RAS_Utils::shortSensorToDistanceInCM(msg->ch1);
        distance_back = RAS_Utils::shortSensorToDistanceInCM(msg->ch2);
    }
    else
    {
        distance_front = msg->ch4;
        distance_back = msg->ch3;
    }
}
