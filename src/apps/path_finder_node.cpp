#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <ras_utils/ras_names.h>
#include <ras_utils/basic_node.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int64MultiArray.h>
#include <ras_utils/occupancy_map_utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

class PathFinderNode : rob::BasicNode
{
public:
    PathFinderNode() : unknown_searching_(true)
    {
        path_pub_  = n.advertise<visualization_msgs::MarkerArray>(TOPIC_MARKERS, 1);

        odo_sub_ = n.subscribe(TOPIC_ODOMETRY, 1, &PathFinderNode::odoCallback, this);
        point_sub_ = n.subscribe(TOPIC_PATH_FINDER_POINT, 1, &PathFinderNode::pointCallback, this);
        map_sub_ = n.subscribe(TOPIC_MAP_OCC_GRID_THICK, 1, &PathFinderNode::mapCallback, this);
        map_cost_sub_ = n.subscribe(TOPIC_MAP_COST, 1, &PathFinderNode::mapCostCallback, this);
    }

    void run()
    {

        ros::Rate loop_rate(PUBLISH_RATE);
        while(ros::ok())
        {
            if(odo_data_ != nullptr && map_data_ != nullptr && map_cost_data_ != nullptr)
            {
                setRobotFrontPos();
                if(unknown_searching_)
                {
                    calculateUnknownPath(*map_data_, *map_cost_data_);
                } else
                {
                    calculatePathToPoint(*map_data_, *map_cost_data_, *point_data_);
                }

                displayPathRviz(path_);

            }


            // ** Sleep
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
private:

    ros::Publisher path_pub_;

    ros::Subscriber point_sub_;
    ros::Subscriber odo_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber map_cost_sub_;

    geometry_msgs::Point::ConstPtr point_data_;
    geometry_msgs::Pose2D::ConstPtr odo_data_;
    nav_msgs::OccupancyGrid::ConstPtr map_data_;
    std_msgs::Int64MultiArray::ConstPtr map_cost_data_;

    std::vector<geometry_msgs::Point> path_;

    bool unknown_searching_;

    double robot_x_pos_;
    double robot_y_pos_;
    double robot_angle_;
    double robot_front_x_pos_;
    double robot_front_y_pos_;

    void setRobotFrontPos()
    {
        double distance_ahead = 0.09;
        robot_front_x_pos_ = robot_x_pos_ + cos(robot_angle_) * distance_ahead;
        robot_front_y_pos_ = robot_y_pos_ + sin(robot_angle_) * distance_ahead;
    }

    void pointCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        unknown_searching_ = false;
        point_data_ = msg;
    }

    void odoCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        odo_data_ = msg;
        robot_x_pos_ = odo_data_->x;
        robot_y_pos_ = odo_data_->y;
        robot_angle_ = odo_data_->theta;

    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_data_ = msg;
    }

    void mapCostCallback(const std_msgs::Int64MultiArray::ConstPtr& msg)
    {
        map_cost_data_ = msg;
    }

    void displayPathRviz(const std::vector<geometry_msgs::Point> &path)
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
    void calculateUnknownPath(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid)
    {
 //       purgePath();

       // if(timeToComputeNewPath() || path_.size() == 0)
       // {
            path_ = RAS_Utils::occ_grid::bfs_search::getClosestUnknownPath(occ_grid, cost_grid, robot_front_x_pos_, robot_front_y_pos_);

            if(path_.size() != 0) {
                path_ = RAS_Utils::occ_grid::bfs_search::getPathFromTo(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_, path_.back().x, path_.back().y);
            }
            if(path_.size() == 0)
            {
                path_ = RAS_Utils::occ_grid::bfs_search::getClosestUnknownPath(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_);
            }

            if(path_.size() == 0 && !RAS_Utils::occ_grid::isFree(occ_grid, robot_x_pos_, robot_y_pos_))
            {
                // special check for slow loading of map.
                // Just add a path that is simple the robot pos
                geometry_msgs::Point point;
                point.x = robot_x_pos_;
                point.y = robot_y_pos_;
                path_.push_back(point);
            }
        //    calculateTimeUntilNextPath();
       // }
    }

    void calculatePathToPoint(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid, const geometry_msgs::Point & to_point)
    {
        path_ = RAS_Utils::occ_grid::bfs_search::getPathFromTo(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_, to_point.x, to_point.y);
    }



};


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_finder_node");

    PathFinderNode pfn;

    pfn.run();

    return 0;
}
