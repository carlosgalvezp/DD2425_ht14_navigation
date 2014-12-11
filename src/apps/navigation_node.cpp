#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_utils/controller.h>
#include <ras_utils/basic_node.h>
#include <ras_srv_msgs/Command.h>
#include <ras_utils/ras_names.h>
#include <navigation/wall_follower.h>
#include <navigation/navigator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Bool.h>

#include <ras_utils/graph/graph.h>
#include <ras_utils/graph/dfs_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

#define QUEUE_SIZE 1
#define PUBLISH_RATE 50

class Navigation : rob::BasicNode
{
public:

    Navigation();
    void run();


private:
    // ** Publishers and subscribers
    ros::Publisher twist_pub_;
    ros::Publisher pose2d_pub_;
    ros::Publisher path_pub_;
    ros::Publisher localize_pub_;

    ros::Subscriber adc_sub_;
    ros::Subscriber odo_sub_;
    ros::Subscriber obj_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber map_cost_sub_;

    // ** Services
    ros::ServiceClient srv_out_;
    ros::ServiceServer srv_in_;

    // ** Callback func when adc data recieved
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);
    void odoCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void objCallback(const std_msgs::Bool::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void mapCostCallback(const std_msgs::Int64MultiArray::ConstPtr &msg);
    // Service callback
    bool srvCallback(ras_srv_msgs::Command::Request &req, ras_srv_msgs::Command::Response &resp);

    void addParams();

    void displayPathRviz(const std::vector<geometry_msgs::Point> &path);

    Controller controller_w;
    Navigator navigator_;
    double kp_w, ki_w, kd_w;

    WF_PARAMS wf_params;
    RT_PARAMS rt_params;
    RAF_PARAMS raf_params;

    int mode_;
    ras_arduino_msgs::ADConverter::ConstPtr adc_data_;
    geometry_msgs::Pose2D::ConstPtr odo_data_;
    std_msgs::Bool::ConstPtr obj_data_;
    nav_msgs::OccupancyGrid::ConstPtr map_data_;
    std_msgs::Int64MultiArray::ConstPtr map_cost_data_;

    int phase_;
    std::queue<geometry_msgs::Point> getObjectsToRetrieve();
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
    path_pub_  = n.advertise<visualization_msgs::MarkerArray>(TOPIC_MARKERS, 1);
    localize_pub_ = n.advertise<std_msgs::Bool>(TOPIC_LOCALIZATION, 1);

    // Subscriber
    adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC, 1,  &Navigation::adcCallback, this);
    odo_sub_ = n.subscribe(TOPIC_ODOMETRY, 1, &Navigation::odoCallback, this);
    obj_sub_ = n.subscribe(TOPIC_OBSTACLE, 1, &Navigation::objCallback, this);
    map_sub_ = n.subscribe(TOPIC_MAP_OCC_GRID_THICK, 1, &Navigation::mapCallback, this);
    map_cost_sub_ = n.subscribe(TOPIC_MAP_COST, 1, &Navigation::mapCostCallback, this);
    // Service callback
    srv_in_ = n.advertiseService(SRV_NAVIGATION_IN, &Navigation::srvCallback, this);

    controller_w = Controller(kp_w,kd_w,ki_w, 10);
}

void Navigation::addParams()
{


    add_param("wf/debug_print", wf_params.debug_print, DEFAULT_DEBUG_PRINT);
    add_param("wf/wanted_distance", wf_params.wanted_distance, DEFAULT_WANTED_DISTANCE);
    add_param("wf/W/KP", wf_params.kp_w, DEFAULT_KP_W);
    add_param("wf/W/KD", wf_params.kd_w, DEFAULT_KD_W);
    add_param("wf/W/KI", wf_params.ki_w, DEFAULT_KI_W);
    add_param("wf/linear_speed", wf_params.wanted_v, DEFAULT_LINEAR_SPEED);
    add_param("wf/D_W/KP", wf_params.kp_d_w, 0.0);
    add_param("wf/D_W/KD", wf_params.kd_d_w, 0.0);
    add_param("wf/D_W/KI", wf_params.ki_d_w, 0.0);

    add_param("Robot_turning/W/KP", rt_params.kp_w, 0.45);
    add_param("Robot_turning/W/KD", rt_params.kd_w, 0.3);
    add_param("Robot_turning/W/KI", rt_params.ki_w, 0.003);

    add_param("Robot_angle_follower/W/KP", raf_params.kp_w, 0.3);
    add_param("Robot_angle_follower/W/KD", raf_params.kd_w, 0.0);
    add_param("Robot_angle_follower/W/KI", raf_params.ki_w, 0.000);
    add_param("wf/linear_speed", raf_params.wanted_v, DEFAULT_LINEAR_SPEED);
    add_param(PARAM_PHASE, phase_, 0);
    add_param(PARAM_ROBOT_VELOCITY, raf_params.wanted_v, raf_params.wanted_v);
    navigator_.setParams(wf_params, rt_params, raf_params, phase_);
}

void Navigation::run()
{
    if(phase_ = 1)
    {
        navigator_.setObjectsToRetrieve(getObjectsToRetrieve());
    }

    ros::Rate loop_rate(PUBLISH_RATE);
    while(ros::ok())
    {
        double v, w;

        // ** Compute velocity commands
        switch(mode_)
        {

            case RAS_Names::Navigation_Modes::NAVIGATION_WALL_FOLLOW:
//                ROS_INFO("[Navigation] Wall following");
                navigator_.computeCommands(odo_data_, adc_data_, obj_data_, map_data_, map_cost_data_, v, w);
                break;

            case RAS_Names::Navigation_Modes::NAVIGATION_GO_OBJECT:
                ROS_INFO("[Navigation] Go to object");

            break;

            case RAS_Names::Navigation_Modes::NAVIGATION_STOP:
//                ROS_INFO("[Navigation] STOP");
                v = 0;
                w = 0;
                break;
        }


        /*
            // TESTING THE PATH
        if(map_data_ != nullptr) {
            std::vector<geometry_msgs::Point> results = RAS_Utils::occ_grid::bfs_search::getClosestUnknownPath(*map_data_, odo_data_->x, odo_data_->y);

            displayPathRviz(results);
            ROS_INFO("%i", results.size());
        }
        */
        double max_w = 1.5;
        if (fabs(w) > max_w)
        {
            w = max_w * RAS_Utils::sign(w);
        }

        // ** Publish
        geometry_msgs::Twist msg;

        displayPathRviz(navigator_.getPath());

        msg.linear.x = v;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = w;

        twist_pub_.publish(msg);

        if(navigator_.shouldLocalize())
        {
            std_msgs::Bool msg;
            msg.data = true;
            localize_pub_.publish(msg);
        }

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

void Navigation::odoCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    odo_data_ = msg;
}

void Navigation::objCallback(const std_msgs::Bool::ConstPtr& msg)
{
    obj_data_ = msg;
}

void Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_data_ = msg;
}

void Navigation::mapCostCallback(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
    map_cost_data_ = msg;
}

bool Navigation::srvCallback(ras_srv_msgs::Command::Request &req, ras_srv_msgs::Command::Response &resp)
{
    ROS_INFO("Navigation receives command: %ld", req.command);
    mode_ = req.command;
    resp.result = 1;
    return true;
}

void Navigation::displayPathRviz(const std::vector<geometry_msgs::Point> &path)
{
    visualization_msgs::MarkerArray msg_array;
    msg_array.markers.resize(1);

    visualization_msgs::Marker &msg = msg_array.markers[0];
    msg.points.resize(path.size());

    msg.header.frame_id = COORD_FRAME_WORLD;
    msg.header.stamp = ros::Time();
    msg.ns = "Path";
    msg.id = 0;
    msg.action = visualization_msgs::Marker::ADD;

    // This is not required for LINE
//    tf::Quaternion q;
//    q.setRPY(0,0,theta);
//    marker_arrow.pose.position.x = x;
//    marker_arrow.pose.position.y = y;
//    marker_arrow.pose.position.z = 0.05;

//    marker_arrow.pose.orientation.x = q.x();
//    marker_arrow.pose.orientation.y = q.y();
//    marker_arrow.pose.orientation.z = q.z();
//    marker_arrow.pose.orientation.w = q.w();

    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.0;


    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;

//    ROS_INFO("%u", path.size());
    for(std::size_t i = 0; i < path.size(); ++i)
    {
        msg.points[i] = path[i];
    }

    path_pub_.publish(msg_array);
}

std::queue<geometry_msgs::Point> Navigation::getObjectsToRetrieve()
{
    std::queue<geometry_msgs::Point> object_points;

    std::ifstream read_file(RAS_Names::OBJECT_BEST_PATH_PATH);
    int id;
    double x, y;
    while (read_file >> id >> x >> y)
    {
        geometry_msgs::Point new_point;
        new_point.x = x;
        new_point.y = y;
        if(fabs(x) > 0.001 || fabs(y) > 0.001) {
            // We skip the start and end points that are (0,0)
            object_points.push(new_point);
        }
    }
    read_file.close();
    return object_points;
}