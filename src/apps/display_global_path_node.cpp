#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ras_utils/basic_node.h>

#include <ras_utils/ras_names.h>
#include <ras_utils/graph/graph.h>

#include <fstream>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    1

class DisplayGlobalPath : rob::BasicNode
{
public:
    DisplayGlobalPath();
    void run();

private:

    ros::Publisher path_pub_;
    std::vector<geometry_msgs::Point> path_;

    void readPath(const std::string &file_path, std::vector<geometry_msgs::Point> &global_path);
    void displayPath(const std::vector<geometry_msgs::Point> &global_path);
};

int main(int argc, char **argv) {
    // ** Launch normal node
    ros::init(argc, argv, "display_global_path_node");

    DisplayGlobalPath d;
    d.run();

    return 0;
}

// ===============================================================================
// ===============================================================================
DisplayGlobalPath::DisplayGlobalPath()
{
    path_pub_ = n.advertise<visualization_msgs::MarkerArray>(TOPIC_MARKERS, 1);

    readPath(RAS_Names::OBJECT_BEST_PATH_PATH, path_);
}

void DisplayGlobalPath::run()
{

    ros::Rate r(PUBLISH_RATE);

    while(ros::ok())
    {
        displayPath(path_);

        r.sleep();
        ros::spinOnce();
    }

}

void DisplayGlobalPath::readPath(const std::string &file_path, std::vector<geometry_msgs::Point> &global_path)
{
    std::ifstream read_file(file_path);
    int id;
    double x, y;
    while (read_file >> id >> x >> y)
    {
        geometry_msgs::Point new_point;
        new_point.x = x;
        new_point.y = y;
        new_point.z = 0.5;
        if(fabs(x) > 0.001 || fabs(y) > 0.001) {
            // We skip the start and end points that are (0,0)
            global_path.push_back(new_point);
        }
    }
    read_file.close();
}

void DisplayGlobalPath::displayPath(const std::vector<geometry_msgs::Point> &global_path)
{
    visualization_msgs::MarkerArray msg_array;
    msg_array.markers.resize(1);

    visualization_msgs::Marker &msg = msg_array.markers[0];
    msg.points.resize(global_path.size());

    msg.header.frame_id = COORD_FRAME_WORLD;
    msg.header.stamp = ros::Time();
    msg.ns = "Global Path";
    msg.id = 0;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.0;


    msg.color.a = 1.0;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;

    for(std::size_t i = 0; i < global_path.size(); ++i)
    {
        msg.points[i] = global_path[i];
    }

    path_pub_.publish(msg_array);
}

