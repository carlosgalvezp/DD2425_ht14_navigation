#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int64MultiArray.h>

#include <ras_utils/ras_utils.h>
#include <ras_utils/occupancy_map_utils.h>

#include <navigation/robot_backer.h>
#include <navigation/robot_turner.h>
#include <navigation/robot_stopper.h>
#include <navigation/wall_follower.h>
#include <navigation/robot_aligner.h>
#include <navigation/robot_angle_follower.h>
//#include <navigation/robot_odometry_aligner.h>

#include <stack>
#include <queue>
#include <math.h>

#include <cstdlib>

#define COMMAND_STOP                    "stop"
#define COMMAND_EXPLORER_TURN           "explorer_turn"
#define COMMAND_BACKUP                  "backup"
#define COMMAND_DANGER_CLOSE_BACKING    "danger_close_backing"
#define COMMAND_ALIGN                   "align"
#define COMMAND_ALIGN_POS_DIR           "align_pos_dir"

#define DANGEROUSLY_CLOSE_LIMIT             6.0
#define DANGEROUSLY_CLOSE_BACKUP_DISTANCE   5
#define DANGEROUSLY_CLOSE_BACKUP_SPEED      -0.1

#define PATH_GRID_POINT_TO_FOLLOW 8 // follow the 10'nth points (means roughly look maximum 10 cm ahead)

#define FIRST_PHASE     0
#define SECOND_PHASE    1

#define REALIGN_POS_TIMER 15

class Navigator
{
public:

    Navigator() : wantedDistanceRecentlySet_(false), going_home_(false), finished_(false), use_path_follower_(false), realign_timer_(ros::WallTime::now()) {}

    void setParams(WF_PARAMS wf_params, RT_PARAMS rt_params, RAF_PARAMS raf_params, int phase)
    {
        wall_follower_.setParams(wf_params);
        robot_turner_.setParams(rt_params);
        robot_angle_follower_.setParams(raf_params);
        phase_ = phase;
    }

    void setObjectsToRetrieve(std::queue<geometry_msgs::Point> objects_to_retrieve)
    {
        objects_to_retrieve_ = objects_to_retrieve;
        if(objects_to_retrieve.size() != 0)
        {
            current_object_point_ = objects_to_retrieve_.front();
            objects_to_retrieve_.pop();
        } else {
            current_object_point_.x = 0;
            current_object_point_.y = 0;
        }
    }

    void computeCommands(const geometry_msgs::Pose2D::ConstPtr &odo_msg,
                                         const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                         const std_msgs::Bool::ConstPtr &obj_msg,
                                         const nav_msgs::OccupancyGrid::ConstPtr & map_msg,
                                         const std_msgs::Int64MultiArray::ConstPtr & map_cost_msg,
                                         double &v, double &w)
    {


        if (adc_msg == nullptr || odo_msg == nullptr || map_msg == nullptr || map_cost_msg == nullptr)
        {
            ROS_ERROR("adc_msg or odo_msg or map_msg or map_cost_msg are null!");
            return;
        }


        if(finished_) {
            v = 0;
            w = 0;
            return;
        }
        //Save all input
        {
            vision_detected_obj_in_front_ = (obj_msg != nullptr && obj_msg->data) ? true : false;

            sd = RAS_Utils::sensors::SensorDistances(
               adc_msg->ch8,
               adc_msg->ch7,
               adc_msg->ch4,
               adc_msg->ch3,
               adc_msg->ch1,
               adc_msg->ch2);

            /*
            ROS_INFO("Sensors %.3f, %.3f, %.3f, %.3f, %.3f ", dist_front_large_range_, d_right_front_, d_right_back_, d_left_front_, d_left_back_);
            ROS_INFO("Wanted distance: ", wanted_distance);
            */
            robot_x_pos_ = odo_msg->x;
            robot_y_pos_ = odo_msg->y;
            robot_angle_ = odo_msg->theta;
            setRobotFrontPos();
        }

        //  If something is currently running, let it work
        if(currentlyRunningCommand(v, w)) return;


        if(command_stack_.size() == 0)
        {
            // We have no more commands to run. Run logic to add commands
            if(phase_ == FIRST_PHASE) {
                explorePhase(v, w, *map_msg, *map_cost_msg);
            } else if(phase_ == SECOND_PHASE)
            {
                retrieveObjects(v, w, *map_msg, *map_cost_msg);
            }
        }


        if(command_stack_.size() != 0)
        {
            // Since we are doing a special command, we can reset the wall wall followers distance value
             wall_follower_.resetWantedDistance();
            // This should always be the case, but just to make sure.
            activateStackedCommand(v, w);

        }
    }

    void setRobotFrontPos()
    {
        double distance_ahead = 0.09;
        robot_front_x_pos_ = robot_x_pos_ + cos(robot_angle_) * distance_ahead;
        robot_front_y_pos_ = robot_y_pos_ + sin(robot_angle_) * distance_ahead;
    }

    std::vector<geometry_msgs::Point> & getPath()
    {
        return path_;
    }

private:

    struct CommandInfo {
        std::string command;
        std::vector<int> args;
        CommandInfo(std::string command, std::vector<int> args = std::vector<int>()) : command(command), args(args) {}
    };

    RAS_Utils::sensors::SensorDistances sd;
    double robot_x_pos_, robot_y_pos_, robot_front_x_pos_, robot_front_y_pos_;
    double robot_angle_;
    bool vision_detected_obj_in_front_;

    RobotStopper robot_stopper_;
    RobotBacker robot_backer_;
    RobotTurner robot_turner_;
    RobotAligner robot_aligner_;
    RobotAngleFollower robot_angle_follower_;
   // RobotOdometryAligner robot_odometry_aligner_;
    WallFollower wall_follower_;

    std::vector<geometry_msgs::Point> path_;

    std::stack<CommandInfo> command_stack_;

    bool wantedDistanceRecentlySet_;

    bool going_home_;
    bool finished_;
    bool use_path_follower_;

    std::vector<geometry_msgs::Point> wall_follower_points_;

    int phase_;

    ros::WallTime realign_timer_;

    // Just for second phase
    std::queue<geometry_msgs::Point> objects_to_retrieve_;
    geometry_msgs::Point current_object_point_;

    void turnCommandCombo()
    {
        command_stack_.push(CommandInfo(COMMAND_STOP));
        command_stack_.push(CommandInfo(COMMAND_ALIGN));
        command_stack_.push(CommandInfo(COMMAND_STOP));
        command_stack_.push(CommandInfo(COMMAND_EXPLORER_TURN));
        command_stack_.push(CommandInfo(COMMAND_STOP));
    }

    ros::WallTime temp_time;


    void explorePhase(double &v, double &w, const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid)
    {

        if(!going_home_){
            if(use_path_follower_) {
            calculateUnknownPath(occ_grid, cost_grid);
                if(path_.size() == 0){
                    going_home_ = true;
                    system("espeak 'The bomb has been planted");
                }
            }
            else
            {
                // Not using wall_follower, clear the path
                path_.clear();
            }
        }

        /*
        if(shouldReAlignPosAndDir())
        {
            command_stack_.push(CommandInfo(COMMAND_ALIGN_POS_DIR));
            command_stack_.push(CommandInfo(COMMAND_ALIGN));
            command_stack_.push(CommandInfo(COMMAND_STOP));
            return;
        }*/

        if(going_home_) {
            calculateHomePath(occ_grid, cost_grid);
        }

        if(going_home_ && path_.size() < 5)
        {
            system("espeak 'Terrorists win");
            finished_ = true;
        }


        if(isWallDangerouslyCloseToWheels()) // && fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
        {
            if(!use_path_follower_)
            {
                testIfWeShouldActivatePathFollower();
            }

            // ALWAYS check this first, this is our most important check for not hitting a wall
            wantedDistanceRecentlySet_ = false;
            // Stop the robot. Then back up some distance
            ROS_ERROR("!!! Dangerously close to wheels !!!");

            command_stack_.push(CommandInfo(COMMAND_DANGER_CLOSE_BACKING));
            command_stack_.push(CommandInfo(COMMAND_STOP));
            return;
        }



        double wanted_angle = getWantedAngle();


        if(isWallCloseInFront())
        {
            if(!use_path_follower_)
            {
                testIfWeShouldActivatePathFollower();
            }

            if(!use_path_follower_ || fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
            {
                // Wall straight ahead, and we are going almost straight to it, force a turn because we probably have a unknown wall ahead that we need to detect.
                wantedDistanceRecentlySet_ = false;
                // Stop the robot! And afterwards, start the rotating!
                turnCommandCombo();
                return;
            }
        }


        if(use_path_follower_) {
            robot_angle_follower_.run(v, w, robot_angle_, wanted_angle);
            std::vector<std::string> strings = {"v", "w", "robot_angle", "wanted_angle"};
            std::vector<double> values  = {v, w, robot_angle_, wanted_angle};
            RAS_Utils::print(strings, values);
        } else
        {
            wall_follower_.run(v, w, sd);
        }
    }


    bool shouldReAlignPosAndDir()
    {
        ros::WallTime current_t = ros::WallTime::now();
        if( (current_t.toSec() - realign_timer_.toSec()) && RAS_Utils::sensors::canFollowAWall(sd))
        {
            return true;
        }
    }

    void retrieveObjects(double &v, double &w, const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid)
    {
        calculatePathToPoint(occ_grid, cost_grid, current_object_point_.x, current_object_point_.y);

        if(currentObjectHasBeenRetrieved())
        {
            if(going_home_)
            {
                // we are home
                v = 0;
                w = 0;
                return;
            }
            if(objects_to_retrieve_.size() > 0) {
                current_object_point_ = objects_to_retrieve_.front();
                objects_to_retrieve_.pop();
            } else
            {
                current_object_point_.x = 0;
                current_object_point_.y = 0;
                going_home_ = true;
            }
        }


        if(isWallDangerouslyCloseToWheels()) // && fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
        {
            // ALWAYS check this first, this is our most important check for not hitting a wall
            wantedDistanceRecentlySet_ = false;
            // Stop the robot. Then back up some distance
            ROS_ERROR("!!! Dangerously close to wheels !!!");
            turnCommandCombo();
            command_stack_.push(CommandInfo(COMMAND_DANGER_CLOSE_BACKING));
            command_stack_.push(CommandInfo(COMMAND_STOP));
            return;
        }

        double wanted_angle = getWantedAngle();


        if(isWallCloseInFront())
        {
            if(!use_path_follower_ || fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
            {
                // Wall straight ahead, and we are going almost straight to it, force a turn because we probably have a unknown wall ahead that we need to detect.
                wantedDistanceRecentlySet_ = false;
                // Stop the robot! And afterwards, start the rotating!
                turnCommandCombo();
                return;
            }
        }

        robot_angle_follower_.run(v, w, robot_angle_, wanted_angle);
        std::vector<std::string> strings = {"v", "w", "robot_angle", "wanted_angle"};
        std::vector<double> values  = {v, w, robot_angle_, wanted_angle};
        RAS_Utils::print(strings, values);
    }


    bool currentObjectHasBeenRetrieved() // TODO: should make sure that we actually have seen the object.
    {
        // right now only checks that we are really close to object detection position
        if(path_.size() < 5)
        {
            return true;
        }
        return false;
    }

    void testIfWeShouldActivatePathFollower()
    {
        geometry_msgs::Point new_point;
        new_point.x = robot_x_pos_;
        new_point.y = robot_y_pos_;
        for(geometry_msgs::Point point : wall_follower_points_)
        {
            if(sqrt(pow(new_point.x - point.x, 2) + pow(new_point.y - point.y, 2)) < 0.1)
            {
                system("espeak 'Switching to path following");
                use_path_follower_ = true;
            }
        }
        wall_follower_points_.push_back(new_point);
    }

    double getWantedAngle()
    {
        if(path_.size() <= 0)
        {
            // Should not happen, but for safety so we dont throw error;
            return robot_angle_;
        }

        geometry_msgs::Point to_point = path_[fmin(PATH_GRID_POINT_TO_FOLLOW, path_.size() - 1)];

        return atan2(to_point.y - robot_y_pos_,  to_point.x - robot_x_pos_);
    }

    void calculateHomePath(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid)
    {
        path_ = RAS_Utils::occ_grid::bfs_search::getPathFromTo(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_, 0, 0);
    }

    void calculatePathToPoint(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid, double to_x, double to_y)
    {
        path_ = RAS_Utils::occ_grid::bfs_search::getPathFromTo(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_, to_x, to_y);
    }


    void calculateUnknownPath(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid)
    {

        if(path_.size() > 20)
        {
            // Pretty long to the unknown area, no need to update where to go, just how to go there.

            geometry_msgs::Point to_point = path_[path_.size() - 1];
            if(RAS_Utils::occ_grid::isUnknown(occ_grid, to_point.x, to_point.y))
            {
                // But also only do this if the point is still a unknown point
                calculatePathToPoint(occ_grid, cost_grid, to_point.x, to_point.y);
                if(path_.size() > 0) {
                    // We only accept this path if we actually could get there. Otherwise contrinue and find a new path
                    return;
                }
            }
        }

        if(!RAS_Utils::occ_grid::isFree(occ_grid, robot_front_x_pos_, robot_front_y_pos_))
        {
            // Because of drift the front pos is a wall or unknown. Lets instead directly use the center robot pos
            path_ = RAS_Utils::occ_grid::bfs_search::getClosestUnknownPath(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_);
        } else
        {
            path_ = RAS_Utils::occ_grid::bfs_search::getClosestUnknownPath(occ_grid, cost_grid, robot_front_x_pos_, robot_front_y_pos_);
            path_ = RAS_Utils::occ_grid::bfs_search::getPathFromTo(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_, path_.back().x, path_.back().y);
        }
    }



    void activateStackedCommand(double &v, double &w)
    {
        CommandInfo command_info = command_stack_.top();
        command_stack_.pop();
        ROS_ERROR("Activated command: %s ", command_info.command.c_str());
        activateStackedCommand(command_info, v, w);
    }

    void activateStackedCommand(CommandInfo command_info, double &v, double &w)
    {
        std::string command = command_info.command;
        std::vector<int> args = command_info.args;


        if(command == COMMAND_STOP) robot_stopper_.activate(v, w);
        else if(command == COMMAND_EXPLORER_TURN) activateExplorerTurner(robot_angle_);
        else if(command == COMMAND_DANGER_CLOSE_BACKING) robot_backer_.activate(DANGEROUSLY_CLOSE_BACKUP_SPEED, DANGEROUSLY_CLOSE_BACKUP_DISTANCE);
        else if(command == COMMAND_ALIGN) robot_aligner_.activate();
       // else if(command == COMMAND_ALIGN_POS_DIR) robot_odometry_aligner_.activate(sd);
        else ROS_ERROR("!!! UNKNOWN COMMAND IN STACK !!!");
    }

    bool currentlyRunningCommand(double &v, double &w) {
         // Just sleep if sleep was activated last iteration
         robot_stopper_.sleepIfStopped();

        if(currentlyBacking(v, w)) return true;

        if(currentlyTurning(v, w)) return true;

        if(currentlyAligning(v, w)) return true;
        return false;
    }


    bool currentlyAligning(double &v, double &w)
    {
        if(robot_aligner_.isActive()) {
            robot_aligner_.run(v, w, sd);
            return true;
        }
        return false;
    }

    /**
        Will return true if we are still backing. False otherwise.
        When finished it will stop the robot immidiatly and add a standard turn command to the queue
    */
    bool currentlyBacking(double &v, double &w)
    {
        if (robot_backer_.isActive())
        {
            // We are currently backing up some distance, let it do its shit!
            ROS_WARN("Backing");
            if(!robot_backer_.isStartPosSet())
            {
                // Needed in order to make sure that the robot is truly stopped when taking the first readings.
                robot_backer_.setStartPos(robot_x_pos_, robot_y_pos_);
            }

            robot_backer_.run(robot_x_pos_, robot_y_pos_, v, w);

            return true;
        }
        return false;
    }

    bool currentlyTurning(double &v, double &w)
    {
        if(robot_turner_.isRotating()){
            // The turner is currently rotating
            ROS_WARN("Turning");
            robot_turner_.run(robot_angle_, v, w);
            return true;
        }
        return false;
    }

    void activateExplorerTurner(double angle_right_now)
    {
        ROS_INFO("!!! Initiate turning !!!");
        double delta_angle = computeExplorerTurningAngle();
        double base_angle = angle_right_now;
        robot_turner_.activate(base_angle, delta_angle);

    }

    double computeExplorerTurningAngle()
    {
        // Decide whether to turn +-90º or 180º
        double turn_angle;
        if(sd.left_front_ > MAX_DIST_SIDE_WALL && sd.left_back_ > MAX_DIST_SIDE_WALL)
        {
            turn_angle = M_PI/2.0;
        }
        else if(sd.right_front_ > MAX_DIST_SIDE_WALL && sd.right_back_ > MAX_DIST_SIDE_WALL)
        {
            turn_angle = -M_PI/2.0;
        }
        else {
            turn_angle = M_PI;
        }
        return turn_angle;
    }

    bool isWallDangerouslyCloseToWheels()
    {
        return sd.right_front_ < DANGEROUSLY_CLOSE_LIMIT || sd.left_front_ < DANGEROUSLY_CLOSE_LIMIT;
    }


    bool isWallCloseInFront( ) {
        return sd.front_ < MAX_DIST_FRONT_WALL || vision_detected_obj_in_front_;
    }

};

#endif // NAVIGATOR_H
