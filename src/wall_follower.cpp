#include <navigation/wall_follower.h>

Wall_follower::Wall_follower() : actual_v(0), turning_(false), stopped(false), set_wanted_distance(false)
{
    controller_w = Controller(kp_w,kd_w,ki_w, 10);
}

void Wall_follower::setParams(const WF_PARAMS &params, const RT_PARAMS &rt_params)
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
    this->rt_params = rt_params;
    controller_w = Controller(kp_w, kd_w, ki_w);

    kp_d_w = params.kp_d_w;
    kd_d_w = params.kd_d_w;
    ki_d_w = params.ki_d_w;
    controller_wall_distance = Controller(kp_d_w, kd_d_w, ki_d_w);
}

void Wall_follower::compute_commands(const geometry_msgs::Pose2D::ConstPtr &odo_msg, const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg, double &v, double &w)
{
    if (adc_msg != nullptr && odo_msg != nullptr)
    {
        if (stopped) 
        {
            ROS_WARN("Sleeping");
            ros::Duration(1.0).sleep();
            stopped = false;
        }
        if (robot_turner.isRotating())
        {
            ROS_WARN("Turning");
            robot_turner.compute_commands(odo_msg, v, w);
            if(!robot_turner.isRotating()) // Finish rotatin -> stop for again
            {
                v = 0;
                w = 0;
                stopped = true;
                set_wanted_distance = false; // Re compute the wanted distance after rotation
            }
            return;
        }
        // ** Check if we need to turn (we have a wall in front of us)
        double dist_front_large_range = RAS_Utils::longSensorToDistanceInCM(adc_msg->ch8);

        double d_right_front = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch4);
        double d_right_back  = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch3);
        double d_left_front  = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch1);
        double d_left_back   = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch2);

        ROS_INFO("Sensors %.3f, %.3f, %.3f, %.3f, %.3f ", dist_front_large_range, d_right_front, d_right_back, d_left_front, d_left_back);
        ROS_INFO("Wanted distance: ", wanted_distance);
        if(is_wall_in_front(dist_front_large_range))
        {
            ROS_INFO("!!! Start turning !!!");
            // Stop the robot! For now...
            turning_ = true;
            double delta_angle = compute_turning_angle(dist_front_large_range,d_right_front,d_right_back, d_left_front, d_left_back);
            double base_angle = odo_msg->theta;
            robot_turner = Robot_turning(rt_params);
            robot_turner.init(base_angle, delta_angle);
            v = 0;
            w = 0;
            stopped = true;
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

                // Set wanted distance
                if(!set_wanted_distance)
                {
                    ROS_ERROR("Setting wanted distance");
                    wanted_distance = wall_is_right ? avg_d_wall_right : avg_d_wall_left;
                    set_wanted_distance = true;
                }

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
                v = 0.15;
                w = 0.0;
                set_wanted_distance = false;
            }
        }
    }
}

bool Wall_follower::is_wall_in_front(double d_front) {
    return d_front < MAX_DIST_FRONT_WALL;
}

double Wall_follower::compute_turning_angle(double d_front, double d_right_front, double d_right_back,
                                double d_left_front, double d_left_back)
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
    else {
        turn_angle = M_PI;
    }
    return turn_angle;
}

void Wall_follower::compute_commands(double distance_front, double distance_back, bool wall_is_right,
                                     double &v, double &w)
{
    double delta;
    double diff;
    double avarage_distance_to_wall;
    double diff_distance_wall = avarage_distance_to_wall -wanted_distance ;
    double KP_DIST_WALL = 0.0; //0.002

    avarage_distance_to_wall = (distance_front + distance_back) / 2.0;
    diff = distance_front - distance_back;
    delta = diff;// + KP_DIST_WALL*diff_distance_wall;

    controller_wall_distance.setData(wanted_distance, avarage_distance_to_wall);
    double w_wall = controller_wall_distance.computeControl();

    if(wall_is_right)
    {
        controller_w.setData(0, delta);
        w = controller_w.computeControl() + w_wall ;
    }
    else
    {
        controller_w.setData(0, -delta);
        w = controller_w.computeControl() - w_wall;
    }


    ROS_INFO("v:%.3f w:%.3f Avg_dist_wall:%.3f Diff:%.3f Delta:%.3fo", v, w, avarage_distance_to_wall, diff, delta);


    /*

    if(fabs(diff) > stopping_error_margin) {
        //we are way of from our wanted direction, stop the motion forward!
        //also increase the power of turning for the robot, needed cause otherwise the robot is too weak.
        actual_v = 0;
        w *= stopped_turn_increaser;
    } else {
        actual_v = fmin(actual_v + wanted_v * slow_start_increaser, wanted_v);
    }
    */
    v = wanted_v;  //actual_v;

}
