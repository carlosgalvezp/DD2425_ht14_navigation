#include <navigation/wall_follower.h>

Wall_follower::Wall_follower() : actual_v(0), stopped(false), wanted_distance_recently_set(false)
{
    controller_align = Controller(kp_w,kd_w,ki_w, 10);
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
    controller_align = Controller(kp_w, kd_w, ki_w);

    kp_d_w = params.kp_d_w;
    kd_d_w = params.kd_d_w;
    ki_d_w = params.ki_d_w;
    controller_wall_distance = Controller(kp_d_w, kd_d_w, ki_d_w);
}

void Wall_follower::compute_commands(const geometry_msgs::Pose2D::ConstPtr &odo_msg,
                                     const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                     const std_msgs::Bool::ConstPtr &obj_msg,
                                     double &v, double &w)
{
    if (adc_msg == nullptr || odo_msg == nullptr)
    {
        ROS_ERROR("adc_msg or odo_msg are null!");
        return;
    }

    if (stopped)
    {
        // We just stopped, always sleep a little after a stop
        ROS_WARN("Sleeping");
        ros::Duration(1.0).sleep();
        stopped = false;
        // Done sleeping, now continue
    }

    if (robot_backer.is_backing())
    {
        // We are currently backing up some distance, let it do its shit!
        ROS_WARN("Backing");
        if(!robot_backer.is_start_pos_set())
        {
            // Needed in order to make sure that the robot is truly stopped when taking the first readings.
            robot_backer.set_start_pos(odo_msg->x, odo_msg->y);
        }

        robot_backer.compute_commands(odo_msg->x, odo_msg->y, v, w);
        if(!robot_backer.is_backing())  // Finish backing -> stop again
        {
            stop_robot(v, w);
            start_turning_next_interval(odo_msg->theta);
        }
        return;
    }


    if (robot_turner.isRotating())
    {
        // The turner is currently rotating, let it work!
        ROS_WARN("Turning");
        robot_turner.compute_commands(odo_msg, v, w);
        if(!robot_turner.isRotating()) // Finish rotatin -> stop for again
        {
            stop_robot(v, w);
            wanted_distance_recently_set = false; // Re compute the wanted distance after rotation
        }
        return;
    }


    // Save input from adc!
    {
        // ** Check if we need to turn (we have a wall in front of us)
        dist_front_large_range = RAS_Utils::longSensorToDistanceInCM(adc_msg->ch8);

        d_right_front = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch4);
        d_right_back  = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch3);
        d_left_front  = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch1);
        d_left_back   = RAS_Utils::shortSensorToDistanceInCM(adc_msg->ch2);
    }

    ROS_INFO("Sensors %.3f, %.3f, %.3f, %.3f, %.3f ", dist_front_large_range, d_right_front, d_right_back, d_left_front, d_left_back);
    ROS_INFO("Wanted distance: ", wanted_distance);

    if(is_wall_dangerously_close_to_wheels())
    {
        // Stop the robot. Then back up some distance
        ROS_ERROR("!!! Dangerously close to wheels !!!");
        start_backing_next_interval(); // TODO: We might want to set this value after we have stopped
        stop_robot(v, w);
        return;
    }


    bool vision_detected_wall = (obj_msg != nullptr && obj_msg->data) ? true : false;

    if(is_wall_close_front() || vision_detected_wall)
    {
        // Stop the robot! And afterwards, start the rotating!
        start_turning_next_interval(odo_msg->theta);
        stop_robot(v, w);
        return;
    }


    // ** Check if there is wall to follow
    if(can_follow_a_wall())
    {
        // Set wanted distance
        if(!wanted_distance_recently_set)
        {
            ROS_ERROR("Setting wanted distance");
            wanted_distance = get_distance_to_closest_wall();
            wanted_distance_recently_set = true;
        }

        // **
        w = align_to_wall_and_wall_distance();
        v = wanted_v;
        return;
    }


    // ** Ask boss to decide what to do
    ROS_ERROR("(ASK BRAIN) COMPLETE THIS PART!");
    v = wanted_v;
    w = 0.0;
    wanted_distance_recently_set = false;
}

bool Wall_follower::is_wall_dangerously_close_to_wheels()
{
    return d_right_front < DANGEROUSLY_CLOSE_LIMIT || d_left_front < DANGEROUSLY_CLOSE_LIMIT;
}

void Wall_follower::start_backing_next_interval()
{
    robot_backer.init(-0.1, 6);
}

void Wall_follower::start_turning_next_interval(double angle_right_now)
{
    ROS_INFO("!!! Initiate turning !!!");
    double delta_angle = compute_turning_angle();
    double base_angle = angle_right_now;
    robot_turner = Robot_turning(rt_params);
    robot_turner.init(base_angle, delta_angle);

}

void Wall_follower::stop_robot(double &v, double &w)
{
    v = 0;
    w = 0;
    stopped = true;
}

double Wall_follower::compute_turning_angle()
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

/*
    Will try to align to the wall and take distance to wall into account, using parameter w.
    Throws exception if no wall is usable.
*/
double Wall_follower::align_to_wall_and_wall_distance(double increased_strength)
{
    return align_to_wall_and_wall_distance(should_prioritize_right_wall(), increased_strength);
}

/*
    Will try to align to the wall to wall, using parameter w.
    Throws exception if no wall is usable.
*/
double Wall_follower::align_to_wall(double increased_strength)
{
    return align_to_wall(should_prioritize_right_wall(), increased_strength);
}

/*
    Will try to align using only distance to wall, using parameter w.
    Throws exception if no wall is usable.
*/
double Wall_follower::align_using_wall_distance(double increased_strength)
{
    return align_to_wall_and_wall_distance(should_prioritize_right_wall(), increased_strength);
}


double Wall_follower::align_to_wall_and_wall_distance(bool wall_is_right, double increased_strength) {

    double w_align_to_wall_distance = align_using_wall_distance(wall_is_right, increased_strength);
    double w_align_to_wall = align_to_wall(wall_is_right, increased_strength);

    ROS_WARN("%f: %f", w_align_to_wall_distance, w_align_to_wall);
    return w_align_to_wall_distance + w_align_to_wall;
}

void Wall_follower::get_distance_front_and_back(bool wall_is_right, double &distance_front, double &distance_back, int &sign)
{
    if(wall_is_right) {
        distance_front = d_right_front;
        distance_back = d_right_back;
        sign = 1;
    }
    else
    {
        distance_front = d_left_front;
        distance_back = d_left_back;
        sign = -1;
    }
}

double Wall_follower::align_using_wall_distance(bool wall_is_right, double increased_strength)
{
    if(!can_follow_wall(wall_is_right)) {
        throw std::runtime_error( "Trying to align using wall distance to: " + boost::lexical_cast<std::string>(wall_is_right) + "(true = right wall) while when we can't! Check code!" );
    }

    int sign;
    double distance_front, distance_back;

    get_distance_front_and_back(wall_is_right, distance_front, distance_back, sign);

    double avarage_distance_to_wall = (distance_front + distance_back) / 2.0;

    controller_wall_distance.setData(wanted_distance, avarage_distance_to_wall);
    return controller_wall_distance.computeControl() *  increased_strength * sign;
}

double Wall_follower::align_to_wall(bool wall_is_right, double increased_strength) {
    if(!can_follow_a_wall()) {
        throw std::runtime_error( "Trying to follow wall:" + boost::lexical_cast<std::string>(wall_is_right) + "(true = right wall)! Check code!" );
    }

    int sign;
    double distance_front, distance_back;

    get_distance_front_and_back(wall_is_right, distance_front, distance_back, sign);

    if(debug_print) ROS_INFO("Following %s wall", (wall_is_right) ? "right" : "left");

    double diff = distance_front - distance_back;
    double delta = diff * increased_strength; // + KP_DIST_WALL*diff_distance_wall;

    controller_align.setData(0, delta * sign);
    return controller_align.computeControl();
}


// Will return true when it feels it is aligned, or return true directly if it can not align
//TODO:: Not compleated yet!
bool Wall_follower::while_standing_still_align_wall() {
    if(!can_follow_a_wall()) {
        return true;
    }

    if(can_follow_left_wall())
    {
        //we can align to the left wall!


    } else if(can_follow_right_wall())
    {
        //we can align to the right wall!
    } else {
        throw std::runtime_error( "can_follow_a_wall and/or can_follow_wall not working! Check code" );
    }

    return false;
}

bool can_follow_wall(double d_front, double d_back)
{
    return d_front < MAX_DIST_SIDE_WALL && d_back < MAX_DIST_SIDE_WALL;
}

bool Wall_follower::can_follow_wall(bool right_wall)
{
    if(right_wall)
    {
        return can_follow_right_wall();
    }else
    {
        return can_follow_left_wall();
    }
}

bool Wall_follower::can_follow_a_wall()
{
    return can_follow_left_wall() || can_follow_right_wall();
}

bool Wall_follower::can_follow_left_wall()
{
    return ::can_follow_wall(d_left_front, d_left_back);
}

bool Wall_follower::can_follow_right_wall()
{
    return ::can_follow_wall(d_right_front, d_right_back);
}

bool Wall_follower::is_wall_close_front( ) {
    return dist_front_large_range < MAX_DIST_FRONT_WALL;
}

double Wall_follower::get_distance_to_left_wall()
{
    return  0.5*(d_left_back + d_left_front);
}

double Wall_follower::get_distance_to_right_wall()
{
    return 0.5*(d_right_back + d_right_front);
}

double Wall_follower::get_distance_to_closest_wall()
{
    return fmin(get_distance_to_left_wall(), get_distance_to_right_wall());
}

bool Wall_follower::should_prioritize_right_wall()
{
    double avg_d_wall_right = 0.5*(d_right_back + d_right_front);
    double avg_d_wall_left  = 0.5*(d_left_back + d_left_front);
    return get_distance_to_right_wall() < get_distance_to_left_wall();
}


