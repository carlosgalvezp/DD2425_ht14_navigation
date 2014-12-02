#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ras_utils/ras_utils.h>
#include <ras_utils/occupancy_map_utils.h>

#include <navigation/robot_backer.h>
#include <navigation/robot_turner.h>
#include <navigation/robot_stopper.h>
#include <navigation/wall_follower.h>
#include <navigation/robot_aligner.h>

#include <stack>

#define COMMAND_STOP                    "stop"
#define COMMAND_EXPLORER_TURN           "explorer_turn"
#define COMMAND_BACKUP                  "backup"
#define COMMAND_DANGER_CLOSE_BACKING    "danger_close_backing"
#define COMMAND_ALIGN                   "aling"

#define DANGEROUSLY_CLOSE_LIMIT             7.5
#define DANGEROUSLY_CLOSE_BACKUP_DISTANCE   7
#define DANGEROUSLY_CLOSE_BACKUP_SPEED      -0.1

class Navigator
{
public:

    Navigator() : wantedDistanceRecentlySet_(false) {}

    void setParams(WF_PARAMS wf_params, RT_PARAMS rt_params)
    {
        wall_follower_.setParams(wf_params);
        robot_turner_.setParams(rt_params);
    }

    void computeCommands(const geometry_msgs::Pose2D::ConstPtr &odo_msg,
                                         const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                         const std_msgs::Bool::ConstPtr &obj_msg,
                                         const nav_msgs::OccupancyGrid::ConstPtr & map_msg,
                                         double &v, double &w)
    {
        if (adc_msg == nullptr || odo_msg == nullptr)
        {
            ROS_ERROR("adc_msg or odo_msg are null!");
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
            exploreRandomly(v, w);
        }


        if(command_stack_.size() != 0)
        {
            // Since we are doing a special command, we can reset the wall wall followers distance value
            wall_follower_.resetWantedDistance();
            // This should always be the case, but just to make sure.
            activateStackedCommand(v, w);

        }
    }

private:

    struct CommandInfo {
        std::string command;
        std::vector<int> args;
        CommandInfo(std::string command, std::vector<int> args = std::vector<int>()) : command(command), args(args) {}
    };

    RAS_Utils::sensors::SensorDistances sd;
    double robot_x_pos_, robot_y_pos_;
    double robot_angle_;
    bool vision_detected_obj_in_front_;

    RobotStopper robot_stopper_;
    RobotBacker robot_backer_;
    RobotTurner robot_turner_;
    RobotAligner robot_aligner_;
    WallFollower wall_follower_;

    std::stack<CommandInfo> command_stack_;

    bool wantedDistanceRecentlySet_;


    void exploreRandomly(double &v, double &w)
    {
        auto turnCommandCombo = [&command_stack_](){
                command_stack_.push(CommandInfo(COMMAND_STOP));
                command_stack_.push(CommandInfo(COMMAND_ALIGN));
                command_stack_.push(CommandInfo(COMMAND_STOP));
                command_stack_.push(CommandInfo(COMMAND_EXPLORER_TURN));
                command_stack_.push(CommandInfo(COMMAND_STOP));
        };


        if(isWallDangerouslyCloseToWheels())
        {
            wantedDistanceRecentlySet_ = false;
            // Stop the robot. Then back up some distance
            ROS_ERROR("!!! Dangerously close to wheels !!!");
            turnCommandCombo();
            command_stack_.push(CommandInfo(COMMAND_DANGER_CLOSE_BACKING));
            command_stack_.push(CommandInfo(COMMAND_STOP));
            return;
        }

        if(isWallCloseInFront())
        {
            wantedDistanceRecentlySet_ = false;
            // Stop the robot! And afterwards, start the rotating!
            turnCommandCombo();
            return;
        }

        // Nothing special in front of us, just run wall_follower
        wall_follower_.run(v, w, sd);
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
