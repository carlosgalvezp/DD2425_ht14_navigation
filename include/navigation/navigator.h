#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ras_utils/ras_utils.h>
#include <ras_utils/occupancy_map_utils.h>

#include <navigation/robot_backer.h>
#include <navigation/robot_turner.h>
#include <navigation/robot_stopper.h>
#include <navigation/wall_follower.h>
#include <navigation/robot_aligner.h>
#include <navigation/robot_angle_follower.h>
//#include <navigation/robot_odometry_aligner.h>
#include <std_msgs/String.h>

#include <stack>
#include <queue>
#include <math.h>
#include <algorithm>

#include <cstdlib>

#define COMMAND_STOP                    "stop"
#define COMMAND_EXPLORER_TURN           "explorer_turn"
#define COMMAND_BACKUP                  "backup"
#define COMMAND_DANGER_CLOSE_BACKING    "danger_close_backing"
#define COMMAND_ALIGN                   "align"
#define COMMAND_ALIGN_POS_DIR           "align_pos_dir"
#define COMMAND_SPECIFIC_TURN           "specific_turn"

#define DANGEROUSLY_CLOSE_LIMIT             7.0
#define DANGEROUSLY_CLOSE_BACKUP_DISTANCE   5
#define DANGEROUSLY_CLOSE_BACKUP_SPEED      -0.1

#define PATH_GRID_POINT_TO_FOLLOW 8 // follow the 10'nth points (means roughly look maximum 10 cm ahead)

#define FIRST_PHASE     0
#define SECOND_PHASE    1

#define REALIGN_POS_TIMER 15

#define MAX_DIST_FRONT_WALL     6      // [cm]

class Navigator
{
public:

    Navigator() : waiting_for_path_to_next_object_(false), waiting_for_path_to_next_object_step_2_(false), close_node(false), looking_at_object_(false), going_home_spoken_(false), localize(false), seconds_until_recompute_path_(-1), wantedDistanceRecentlySet_(false), going_home_(false), finished_(false), localize_timer_(ros::WallTime::now())
    {
        latest_path_update_time_ = ros::WallTime::now();
    }

    void setParams(WF_PARAMS wf_params, RT_PARAMS rt_params, RAF_PARAMS raf_params, int phase)
    {
        robot_turner_.setParams(rt_params);
        robot_angle_follower_.setParams(raf_params);
        phase_ = phase;
    }

    void setObjectsToRetrieve(std::queue<geometry_msgs::Point> objects_to_retrieve)
    {
        looking_at_object_ = true;
        objects_to_retrieve_ = objects_to_retrieve;
        if(objects_to_retrieve.size() != 0)
        {
            object_to_look_at_ = objects_to_retrieve_.front();
            objects_to_retrieve_.pop();
        } else {
            object_to_look_at_.x = 0;
            object_to_look_at_.y = 0;
        }
        geometry_msgs::Point home_point;
        home_point.x = 0;
        home_point.y = 0;
        objects_to_retrieve_.push(home_point);
    }

    void computeCommands(const nav_msgs::OccupancyGrid::ConstPtr & map_msg,
                                         const geometry_msgs::Pose2D::ConstPtr &odo_msg,
                                         const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                         const std_msgs::Bool::ConstPtr &obj_msg,
                                         const visualization_msgs::MarkerArray::ConstPtr & path_msg,
                                         double &v, double &w)
    {


        if (adc_msg == nullptr || odo_msg == nullptr || path_msg == nullptr || map_msg == nullptr)
        {
            ROS_ERROR("adc_msg or odo_msg or path_msg are null!");
            return;
        }

        path_ = path_msg->markers[0].points;

        {
            // Special handshake for second phase
            if(waiting_for_path_to_next_object_)
            {
                ROS_WARN("Waiting first shake: %lu", path_.size());

                //
                if(path_.size() > 1)
                {
                    // Still no path
                    v = 0;
                    w = 0;
                    return;
                } else
                {
                    ROS_WARN("Start second second shake: %lu", path_.size());
                    waiting_for_path_to_next_object_ = false;
                    waiting_for_path_to_next_object_step_2_ = true;
                    object_to_look_at_ = objects_to_retrieve_.front();
                    objects_to_retrieve_.pop();
                    ROS_ERROR("Objects left: %lu, X: %.3f Y %.3f", objects_to_retrieve_.size(), object_to_look_at_.x, object_to_look_at_.y);
                }
            }

            if(waiting_for_path_to_next_object_step_2_)
            {

                ROS_WARN("Waiting second shake: %lu", path_.size());

                if(path_.size() <= 1)
                {
                    // Correct path not retrieved
                    v = 0;
                    w = 0;
                    return;
                } else
                {
                    ROS_WARN("Second shake done: %lu", path_.size());
                    // Now we can continue
                    waiting_for_path_to_next_object_step_2_ = false;
                }
            }
        }

        purgePath();

     //   ros::WallTime temp_time = ros::WallTime::now();

        if(going_home_ && !going_home_spoken_)
        {
            ROS_ERROR("We are now going home");
            std_msgs::String msg;
            msg.data = "I am going home now, hopefully";
            this->speaker_pub_.publish(msg);
            going_home_spoken_ = true;
        }


        if(finished_) {
            ROS_ERROR("We are back home");
            system("espeak 'I am back home!'");
            close_node = true;
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
        }

        //  If something is currently running, let it work
        if(currentlyRunningCommand(v, w)) return;


        if(command_stack_.size() == 0)
        {
            // We have no more commands to run. Run logic to add commands
            if(phase_ == FIRST_PHASE) {
                explorePhase(v, w, *map_msg);
            } else if(phase_ == SECOND_PHASE)
            {
                retrieveObjects(v, w, *map_msg);
            }
        }


        if(command_stack_.size() != 0)
        {
            // This should always be the case, but just to make sure.
            activateStackedCommand(v, w);

        }

        //std::cout << "total: " << RAS_Utils::time_diff_ms(temp_time, ros::WallTime::now()) << std::endl;
    }

    void objectDetected(const geometry_msgs::Point & object_point)
    {
        if(phase_ == SECOND_PHASE)
        {
            // To nothing if we are doing phase 2
            return;
        }

        for(geometry_msgs::Point & old_point : visited_object_locations)
        {
            if(RAS_Utils::euclidean_distance(old_point.x, old_point.y, object_point.x, object_point.y) < 0.15)
            {
                // We have already seen this object... I think
                return;
            }
        }

        // New object to look at!
        looking_at_object_ = true;
        visited_object_locations.push_back(object_point);

        double distance = RAS_Utils::euclidean_distance(robot_x_pos_, robot_y_pos_, object_point.x, object_point.y) - 0.1;
        double angle = atan2(object_point.y - robot_y_pos_, object_point.x - robot_x_pos_);

        object_to_look_at_.x = robot_x_pos_ + cos(angle)*distance;
        object_to_look_at_.y = robot_y_pos_ + sin(angle)*distance;
    }


    std::vector<geometry_msgs::Point> & getPath()
    {
        return path_;
    }

    bool shouldLocalize()
    {
        if((ros::WallTime::now().toSec() - localize_timer_.toSec() > 10.0)  && RAS_Utils::sensors::canFollowAWall(sd))
        {
            return true;
        }
        return false;
    }

    void resetLocalizeTimer()
    {
        localize_timer_ = ros::WallTime::now();
    }

    bool isGoingHome()
    {
        return going_home_;
    }

    bool lookingAtObject()
    {
        return looking_at_object_;
    }

    const geometry_msgs::Point & getObjectToLookAt()
    {
        return object_to_look_at_;
    }

    bool closeNode()
    {
        return close_node;
    }

    bool isHandShaking()
    {
        return waiting_for_path_to_next_object_ || waiting_for_path_to_next_object_step_2_;
    }

    void setSpeakerPublisher(const ros::Publisher &pub)
    {
        this->speaker_pub_ = pub;
    }

private:

    struct CommandInfo {
        std::string command;
        std::vector<double> args;
        CommandInfo(std::string command, std::vector<double> args = std::vector<double>()) : command(command), args(args) {}
    };

    ros::Publisher speaker_pub_;

    RAS_Utils::sensors::SensorDistances sd;
    double robot_x_pos_, robot_y_pos_;
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
    bool going_home_spoken_;
    bool finished_;
    bool close_node;

    std::vector<geometry_msgs::Point> wall_follower_points_;

    int phase_;

    ros::WallTime localize_timer_;
    bool localize;

    ros::WallTime latest_path_update_time_;
    double seconds_until_recompute_path_;

    // Just for second phase
    std::queue<geometry_msgs::Point> objects_to_retrieve_;

    std::vector<geometry_msgs::Point> visited_object_locations;
    geometry_msgs::Point object_to_look_at_;

    bool looking_at_object_;
    bool waiting_for_path_to_next_object_;
    bool waiting_for_path_to_next_object_step_2_;

    void turnCommandCombo()
    {
        command_stack_.push(CommandInfo(COMMAND_STOP));
        command_stack_.push(CommandInfo(COMMAND_ALIGN));
        command_stack_.push(CommandInfo(COMMAND_STOP));
        command_stack_.push(CommandInfo(COMMAND_EXPLORER_TURN));
        command_stack_.push(CommandInfo(COMMAND_STOP));
    }

    ros::WallTime temp_time;


    void explorePhase(double &v, double &w, const nav_msgs::OccupancyGrid & occ_grid)
    {

        if(looking_at_object_)
        {
            if(path_.size() < 20)
            {
                looking_at_object_ = false;
                v = 0;
                w = 0;
                return;
            }
        }

        if(!going_home_)
        {
            if(path_.size() == 0)
            {
                going_home_ = true;
                v = 0;
                w = 0;
                return;
            }
        }

        //if(going_home_) {
        //    calculateHomePath(occ_grid, cost_grid);
        //}

        if(going_home_ && isCloseToHome())
        {
            finished_ = true;
            v = 0;
            w = 0;
            return;
        }





        if(isWallDangerouslyCloseToWheels()) // && fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
        {
          //  updatePathNextIteration();
            // ALWAYS check this first, this is our most important check for not hitting a wall
          //  wantedDistanceRecentlySet_ = false;
            // Stop the robot. Then back up some distance
            ROS_ERROR("!!! Dangerously close to wheels !!!");

            command_stack_.push(CommandInfo(COMMAND_DANGER_CLOSE_BACKING));
            command_stack_.push(CommandInfo(COMMAND_STOP));
            return;
        }



        double wanted_angle = getWantedAngle(occ_grid);


        if(isWallCloseInFront())
        {

            if(fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
            {
               // updatePathNextIteration();
                // Wall straight ahead, and we are going almost straight to it, force a turn because we probably have a unknown wall ahead that we need to detect.
               // wantedDistanceRecentlySet_ = false;
                // Stop the robot! And afterwards, start the rotating!
                turnCommandCombo();
                return;
            }
        }


        robot_angle_follower_.run(v, w, robot_angle_, wanted_angle);
    }

    void retrieveObjects(double &v, double &w, const nav_msgs::OccupancyGrid & occ_grid)
    {

        //calculatePathToPoint(occ_grid, cost_grid, current_object_point_.x, current_object_point_.y);

        if(currentObjectHasBeenRetrieved())
        {
            if(going_home_)
            {
                ROS_ERROR("We are home");
                // we are home
                finished_ = true;
                v = 0;
                w = 0;
                return;
            }
            if(objects_to_retrieve_.size() > 1) {
                // Add turning towards object

                std::vector<double> command_args;
                double wanted_angle = atan2(object_to_look_at_.y - robot_y_pos_,  object_to_look_at_.x - robot_x_pos_);
                command_args.push_back(wanted_angle);

                command_stack_.push(CommandInfo(COMMAND_STOP));
                command_stack_.push(CommandInfo(COMMAND_STOP));
                command_stack_.push(CommandInfo(COMMAND_STOP));
                command_stack_.push(CommandInfo(COMMAND_SPECIFIC_TURN, command_args));


                ROS_WARN("%lu", objects_to_retrieve_.size());
                ROS_ERROR("New object!");
                waiting_for_path_to_next_object_ = true;
                object_to_look_at_.x = -5.0;
                object_to_look_at_.y = -5.0;

                v = 0;
                w = 0;


                return;
            } else
            {
                ROS_ERROR("Going home!");
                v = 0;
                w = 0;
                waiting_for_path_to_next_object_ = true;
                object_to_look_at_.x = -5.0;
                object_to_look_at_.y = -5.0;
                going_home_ = true;
                return;
            }

            ROS_ERROR("STOP!");
        }


        if(isWallDangerouslyCloseToWheels()) // && fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
        {
            // ALWAYS check this first, this is our most important check for not hitting a wall

            // Stop the robot. Then back up some distance
            ROS_ERROR("!!! Dangerously close to wheels !!!");
            command_stack_.push(CommandInfo(COMMAND_DANGER_CLOSE_BACKING));
            command_stack_.push(CommandInfo(COMMAND_STOP));
            return;
        }

        double wanted_angle = getWantedAngle(occ_grid);


        if(isWallCloseInFront())
        {

            if(fabs(RAS_Utils::normalize_angle(wanted_angle - robot_angle_)) < M_PI/7)
            {
                // Wall straight ahead, and we are going almost straight to it, force a turn because we probably have a unknown wall ahead that we need to detect.
                wantedDistanceRecentlySet_ = false;
                // Stop the robot! And afterwards, start the rotating!
                turnCommandCombo();
                return;
            }
        }

        robot_angle_follower_.run(v, w, robot_angle_, wanted_angle);

    }


    bool currentObjectHasBeenRetrieved() // TODO: should make sure that we actually have seen the object.
    {
        // right now only checks that we are really close to object detection position
        if(path_.size() < 18)
        {
            return true;
        }

        return false;
    }

    double getWantedAngle(const nav_msgs::OccupancyGrid & occ_grid)
    {

        ros::WallTime temp_time = ros::WallTime::now();
        if(path_.size() <= 0)
        {
            // Should not happen, but for safety so we dont throw error;
            return robot_angle_;
        }

        geometry_msgs::Point to_point = getPointToFollow(occ_grid);

//        std::cout  << "Wanted angle: " << RAS_Utils::time_diff_ms(temp_time, ros::WallTime::now()) << std::endl;

        return atan2(to_point.y - robot_y_pos_,  to_point.x - robot_x_pos_);
    }

    geometry_msgs::Point getPointToFollow(const nav_msgs::OccupancyGrid & occ_grid)
    {
        int best_point = -1;
        bool point_is_good = true;
        for(int i = 0; i < path_.size() && point_is_good; i++)
        {
            geometry_msgs::Point & point = path_[i];

            double distance = RAS_Utils::euclidean_distance(robot_x_pos_, robot_y_pos_, point.x, point.y);
            double angle = atan2(point.y - robot_y_pos_,  point.x - robot_x_pos_);

            for(double part_dist = 0; part_dist <= distance; part_dist += 0.01)
            {
                double x_pos = robot_x_pos_ + cos(angle) * part_dist;
                double y_pos = robot_y_pos_ + sin(angle) * part_dist;

                if(!RAS_Utils::occ_grid::isFree(occ_grid, x_pos, y_pos))
                {
                    point_is_good = false;
                    break;
                }
                std::vector<double> extra_x_points;
                std::vector<double> extra_y_points;
                double m_away_from_wall = 0.06;
                extra_x_points.push_back(-m_away_from_wall);
                extra_y_points.push_back(-m_away_from_wall);

                extra_x_points.push_back(-m_away_from_wall);
                extra_y_points.push_back(m_away_from_wall);

                extra_x_points.push_back(m_away_from_wall);
                extra_y_points.push_back(-m_away_from_wall);

                extra_x_points.push_back(m_away_from_wall);
                extra_y_points.push_back(m_away_from_wall);

                // Thick "thicker line"
                for(double extra_x : extra_x_points)
                {
                    for(double extra_y : extra_y_points)
                    {
                        if(!RAS_Utils::occ_grid::isFree(occ_grid, x_pos + extra_x, y_pos + extra_y))
                        {
                            point_is_good = false;
                        }
                    }
                }

            }
            if(point_is_good)
            {
                best_point++;
            }

        }
        best_point = best_point - 10;
//        ROS_INFO("Best point: %u", best_point);

        return path_[std::max(best_point, std::min(8, (int)path_.size() - 1))];
    }

    void calculateHomePath(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid)
    {
        calculatePathToPoint(occ_grid, cost_grid, 0, 0);
    }

    void calculatePathToPoint(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid, double to_x, double to_y)
    {
        purgePath();
      //  if(timeToComputeNewPath())
    //    {
            path_ = RAS_Utils::occ_grid::bfs_search::getPathFromTo(occ_grid, cost_grid, robot_x_pos_, robot_y_pos_, to_x, to_y);
            calculateTimeUntilNextPath();
      //  }
    }

    void purgePath()
    {
//         ROS_INFO("Entered Purge");
        int closest_index = 0;
        double closest_distance = -1;
        double current_distance;
        for(int i = 0; i < 25 && i < path_.size(); i++)
        {
            geometry_msgs::Point & point = path_[i];
            current_distance = RAS_Utils::euclidean_distance(robot_x_pos_, robot_y_pos_, point.x, point.y);
            if(current_distance < closest_distance || closest_distance == -1)
            {
                closest_index = i;
                closest_distance = current_distance;
            }
        }

        // Remove the points we have allready passed
        if(closest_index != 0)
        {
//            ROS_WARN("Purged: %u", closest_index);
            path_.erase(path_.begin(), path_.begin() + closest_index);
        }
//        ROS_INFO("Exited Purge");
    }

    void calculateTimeUntilNextPath()
    {
//        ROS_ERROR("!!! NEW PATH !!!");
        latest_path_update_time_ = ros::WallTime::now();
        seconds_until_recompute_path_ = path_.size() / 50.0;
    }



    void updatePathNextIteration()
    {
        seconds_until_recompute_path_ = -1; // Recompute path the next time
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
        std::vector<double> args = command_info.args;


        if(command == COMMAND_STOP) robot_stopper_.activate(v, w);
        else if(command == COMMAND_EXPLORER_TURN) activateExplorerTurner(robot_angle_);
        else if(command == COMMAND_SPECIFIC_TURN) activateSpecificTurner(robot_angle_, args[0]);
        else if(command == COMMAND_DANGER_CLOSE_BACKING) robot_backer_.activate(DANGEROUSLY_CLOSE_BACKUP_SPEED, DANGEROUSLY_CLOSE_BACKUP_DISTANCE);
        else if(command == COMMAND_ALIGN) robot_aligner_.activate();
       // else if(command == COMMAND_ALIGN_POS_DIR) robot_odometry_aligner_.activate(sd);
        else ROS_ERROR("!!! UNKNOWN COMMAND IN STACK !!!");
    }

    bool currentlyRunningCommand(double &v, double &w) {
         // Just sleep if sleep was activated last iteration


        if(robot_stopper_.sleepIfStopped()) return true;

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

    void activateSpecificTurner(double angle_right_now, double wanted_angle)
    {
        ROS_INFO("!!! Initiate object turning !!!");
        robot_turner_.activate(angle_right_now, wanted_angle - angle_right_now);
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

    bool isCloseToHome()
    {
        return (fabs(robot_x_pos_) < 0.15 && fabs(robot_y_pos_) < 0.15);
    }

};

#endif // NAVIGATOR_H
