#include <navigation/wall_follower.h>

Wall_follower::Wall_follower() : actual_v(0), turning_(false)
{
    controller_w = Controller(kp_w,kd_w,ki_w, 10);
}

void Wall_follower::setParams(const WF_PARAMS &params)
{
    debug_print = params.debug_print;
    wanted_distance = params.wanted_distance;
    kp_w = params.kp_w;
    kd_w = params.kd_w;
    ki_w = params.ki_w;
    wanted_v = params.wanted_v;
    stopping_error_margin = params.stopping_error_margin;
    stopped_turn_increaser = params.stopped_turn_increaser;
    slow_start_increaser = params.slow_start_increaser;
}

void Wall_follower::compute_commands(const ras_arduino_msgs::ADConverter::ConstPtr &msg, double &v, double &w)
{
    if (msg != nullptr)
    {
        // ** Check if we need to turn (we have a wall in front of us)
        double dist_front_large_range = msg->ch8;
        ROS_ERROR("CONVERT LONG RANGE SENSOR TO CM!");

        double d_right_front = RAS_Utils::shortSensorToDistanceInCM(msg->ch4);
        double d_right_back  = RAS_Utils::shortSensorToDistanceInCM(msg->ch3);
        double d_left_front  = RAS_Utils::shortSensorToDistanceInCM(msg->ch1);
        double d_left_back   = RAS_Utils::shortSensorToDistanceInCM(msg->ch2);

	ROS_INFO("Sensors %.3f, %.3f, %.3f, %.3f, %.3f ", dist_front_large_range, d_right_front, d_right_back, d_left_front, d_left_back);
        if(check_turn(dist_front_large_range,d_right_front,d_right_back, d_left_front, d_left_back))
        {
            ROS_INFO("Have to turn");
        }
        else
        {
            // ** Check if there is wall to follow
            if( (d_left_front < MAX_DIST_SIDE_WALL && d_left_back < MAX_DIST_SIDE_WALL) ||
                (d_right_front < MAX_DIST_SIDE_WALL && d_right_back < MAX_DIST_SIDE_WALL)  )
            {
                // ** Check which wall is closest
                double avg_d_wall_right = 0.5*(d_right_back + d_right_front);
                double avg_d_wall_left  = 0.5*(d_left_back + d_left_front);

                wall_is_right = (avg_d_wall_right < avg_d_wall_left);
                double distance_front, distance_back;
                if(wall_is_right)
                {
                    ROS_INFO("Following right wall");
                    distance_front = d_right_front;
                    distance_back = d_right_back;
                }
                else
                {
                    ROS_INFO("Following left wall");
                    distance_front = d_left_front;
                    distance_back = d_left_back;
                }
                // ** Compute commands
                compute_commands(distance_front, distance_back, wall_is_right,v,w);
            }
            else{
                // ** Ask boss to decide what to do
                ROS_ERROR("(ASK BRAIN) COMPLETE THIS PART!");
            }
        }
    }
}

bool Wall_follower::check_turn(double d_front, double d_right_front, double d_right_back,
                                double d_left_front, double d_left_back)
{
    if(d_front > MAX_DIST_FRONT_WALL) // Change this, should be <
    {
        // Decide whether to turn +-90º or 180º
        double turn_angle;
        if(d_left_front > MAX_DIST_SIDE_WALL && d_left_back > MAX_DIST_SIDE_WALL)
        {
            turn_angle = M_PI/2.0;
        }
        else if(d_right_front > MAX_DIST_SIDE_WALL && d_right_back > MAX_DIST_SIDE_WALL)
        {
            turn_angle = -M_PI/2.0;
        }
        else
            turn_angle = M_PI;

	ROS_ERROR("Turn %.3f rads. Missing Turner object", turn_angle);
        return true;        
//        Turner.set_turn(turn_angle);
    }
    return false;
}

void Wall_follower::compute_commands(double distance_front, double distance_back, bool wall_is_right,
                                     double &v, double &w)
{
    double delta;
    double diff;
    double avarage_distance_to_wall;

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
    v = actual_v;
}
