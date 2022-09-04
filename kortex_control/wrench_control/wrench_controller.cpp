#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <arthur_planning/arthur_traj.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <kortex_driver/SendWrenchCommand.h>
#include <boost/bind.hpp>
#include <vector>
#include <array>

/**
 * @brief Sends wrench command to kortex_driver service SendWrenchCommand
 *
 * @param wrench_commander client to send wrench command through
 * @param frame the frame of the wrench
 * @param mode the mode of the wrench command
 * @param duration the duration of the wrench command
 * @param wrench the wrench
 * @return true
 * @return false
 */
bool sendWrench(ros::ServiceClient wrench_commander, const int frame, const int mode, const int duration, const float wrench[6])
{
    kortex_driver::WrenchCommand wrench_command;
    wrench_command.duration = duration;
    wrench_command.mode = mode;
    wrench_command.reference_frame = frame;

    kortex_driver::WrenchCommand::_wrench_type w;
    w.force_x = wrench[0];
    w.force_y = wrench[1];
    w.force_z = wrench[2];
    w.torque_x = wrench[3];
    w.torque_y = wrench[4];
    w.torque_z = wrench[5];
    wrench_command.wrench = w;

    kortex_driver::SendWrenchCommand srv;
    srv.request.input = wrench_command;
    return wrench_commander.call(srv);
}

// sends a wrench command to gen3 to hold the current position
bool pauseControls(ros::ServiceClient wrench_commander, ros::Publisher reamer_commander, double *reamerVel)
{
    std_msgs::Int16 reamer_msg;
    reamer_msg.data = 0;
    reamer_commander.publish(reamer_msg);
    
    kortex_driver::WrenchCommand wrench_command;
    wrench_command.duration = 0;
    wrench_command.mode = 1;
    wrench_command.reference_frame = 1;

    kortex_driver::WrenchCommand::_wrench_type w;
    w.force_x = 0;
    w.force_y = 0;
    w.force_z = 0;
    w.torque_x = 0;
    w.torque_y = 0;
    w.torque_z = 0;
    wrench_command.wrench = w;

    kortex_driver::SendWrenchCommand srv;
    srv.request.input = wrench_command;
    // ROS_INFO("Reamer Velocity: %f", (*reamerVel));
    return wrench_commander.call(srv) && ((*reamerVel) <= 0.01 && (*reamerVel) >= -0.01);
}

void pelvisTFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, std::array<double, 6> *xyzrpy, tf::Matrix3x3 *rot) {
    tf::Quaternion q = tf::Quaternion((*msg).pose.orientation.x, (*msg).pose.orientation.y, (*msg).pose.orientation.z, (*msg).pose.orientation.w);
    (*rot) = tf::Matrix3x3(q).inverse();
    // (*xyzrpy)[0] = (*msg).pose.position.x;
    // (*xyzrpy)[1] = (*msg).pose.position.y;
    // (*xyzrpy)[2] = (*msg).pose.position.z;
    (*rot).getRPY((*xyzrpy)[3], (*xyzrpy)[4], (*xyzrpy)[5]);
    double H[4][4] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            H[i][j] = (*rot).inverse()[i][j];
        }
    }
    H[0][3] = (*msg).pose.position.x;
    H[1][3] = (*msg).pose.position.y;
    H[2][3] = (*msg).pose.position.z;
    H[3][3] = 1;
    double relPos[4] = {0, 0, -0.1, 1};
    double rotatedRelPos[4] = {0.0};
    rotatedRelPos[3] = 1;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            rotatedRelPos[i] += H[i][j] * relPos[j];
        }
    }
    (*xyzrpy)[0] = rotatedRelPos[0];
    (*xyzrpy)[1] = rotatedRelPos[1];
    (*xyzrpy)[2] = rotatedRelPos[2];
}

// Updates the ee pose wrt to the base_link
void tfCallback(const geometry_msgs::Transform::ConstPtr &msg, double xyzrpy[6], std::array<double, 6> *retract_xyzrpy, tf::Matrix3x3 *rot, double velocities[6], ros::Time *time, const bool *dynamicComp, const bool *finished, tf::Quaternion *q)
{
    // extract pose from msg
    double pose[6];
    pose[0] = (*msg).translation.x;
    pose[1] = (*msg).translation.y;
    pose[2] = (*msg).translation.z;
    (*q) = tf::Quaternion((*msg).rotation.x, (*msg).rotation.y, (*msg).rotation.z, (*msg).rotation.w);
    // save orientation as rotation matrix for rotating torques later
    (*rot) = tf::Matrix3x3((*q)).inverse();
    (*rot).getRPY(pose[3], pose[4], pose[5]);

    // calculate velocities
    ros::Duration elapsed = ros::Time::now() - (*time);
    double dt = elapsed.toSec();
    velocities[0] = (pose[0] - xyzrpy[0]) / dt;
    velocities[1] = (pose[1] - xyzrpy[1]) / dt;
    velocities[2] = (pose[2] - xyzrpy[2]) / dt;
    velocities[3] = (pose[3] - xyzrpy[3]) / dt;
    velocities[4] = (pose[4] - xyzrpy[4]) / dt;
    velocities[5] = (pose[5] - xyzrpy[5]) / dt;

    // save pose from msg into xyzrpy vector
    xyzrpy[0] = pose[0];
    xyzrpy[1] = pose[1];
    xyzrpy[2] = pose[2];
    xyzrpy[3] = pose[3];
    xyzrpy[4] = pose[4];
    xyzrpy[5] = pose[5];
    // ROS_INFO("Dynamic comp: %d", (*dynamicComp));
    if (!(*dynamicComp) && !(*finished)) {
        // ROS_INFO("Updating retract xyzrpy");
        // double relPos[3] = {0, 0, -0.01};
        // double rotatedRelPos[3] = {0.0};
        // for (int i = 0; i < 3; i++)
        // {
        //     for (int j = 0; j < 3; j++)
        //     {
        //         rotatedRelPos[i] += (*rot).inverse()[i][j] * relPos[j];
        //     }
        // }
        // (*retract_xyzrpy)[0] = rotatedRelPos[0];
        // (*retract_xyzrpy)[1] = rotatedRelPos[1];
        // (*retract_xyzrpy)[2] = rotatedRelPos[2];
        // (*retract_xyzrpy)[3] = pose[3];
        // (*retract_xyzrpy)[4] = pose[4];
        // (*retract_xyzrpy)[5] = pose[5];

        double H[4][4] = {0};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                H[i][j] = (*rot).inverse()[i][j];
            }
        }
        H[0][3] = (*msg).translation.x;
        H[1][3] = (*msg).translation.y;
        H[2][3] = (*msg).translation.z;
        H[3][3] = 1;
        double relPos[4] = {0, 0, -0.05, 1};
        double rotatedRelPos[4] = {0.0};
        rotatedRelPos[3] = 1;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                rotatedRelPos[i] += H[i][j] * relPos[j];
            }
        }
        (*retract_xyzrpy)[0] = rotatedRelPos[0];
        (*retract_xyzrpy)[1] = rotatedRelPos[1];
        (*retract_xyzrpy)[2] = rotatedRelPos[2];
        (*retract_xyzrpy)[3] = pose[3];
        (*retract_xyzrpy)[4] = pose[4];
        (*retract_xyzrpy)[5] = pose[5];
    }
    (*time) = ros::Time::now();
}

// Checks to see if there is a new trajectory. If yes, then update trajectory vector
void trajCallback(const arthur_planning::arthur_traj::ConstPtr &msg, std::vector<std::array<double, 6>> *traj, int *current_waypoint, int *trajNum, std::vector<tf::Matrix3x3> *desRot, ros::ServiceClient wrench_commander, ros::Publisher reamer_commander, bool *planned, double *reamerVel)
{
    // if new trajectory, do the following
    if ((*msg).trajNum != (*trajNum))
    {
        // stop wrench commands, and pause robot at current spot
        while (!pauseControls(wrench_commander, reamer_commander, reamerVel))
        {
            ROS_INFO("Trying to pause Reaming Operation!");
            // ROS_INFO("Reamer Vel trajCallback: %f", (*reamerVel));
            ros::spinOnce();
        }
        ROS_INFO("Pausing Reaming Operation for 3 seconds");

        // clear trajectory vector
        (*traj).clear();
        (*desRot).clear();
        // for each pose in trajectory, store it in array and push into trajectory vector
        int len = (*msg).cartesian_states.poses.size();
        for (int i = 0; i < len; i++)
        {
            std::array<double, 6> a = {0};
            a[0] = (*msg).cartesian_states.poses[i].position.x;
            a[1] = (*msg).cartesian_states.poses[i].position.y;
            a[2] = (*msg).cartesian_states.poses[i].position.z;
            tf::Quaternion q = tf::Quaternion((*msg).cartesian_states.poses[i].orientation.x, (*msg).cartesian_states.poses[i].orientation.y, (*msg).cartesian_states.poses[i].orientation.z, (*msg).cartesian_states.poses[i].orientation.w);
            (*desRot).push_back(tf::Matrix3x3(q).inverse());
            (*desRot).back().getRPY(a[4], a[5], a[6]);
            (*traj).push_back(a);
        }
        (*current_waypoint) = 0;     // reset waypoint back to 0
        (*trajNum) = (*msg).trajNum; // save current trajectory number for detecting new trajectory
        (*planned) = true;
        // pause control node for 3 seconds
        ros::Duration(3).sleep();
    }
}

void dynamicCompTrigger(const std_msgs::Bool::ConstPtr &msg, bool *dynamicComp, bool *startPlanner, bool *finished, bool *planned, ros::ServiceClient wrench_commander, ros::Publisher reamer_commander, double *reamerVel) {
    if ((*msg).data) {
        (*dynamicComp) = true;
        (*startPlanner) = false;
        // (*finished) = false;
        (*planned) = false;
        // TODO: Stop reamer
        while (!pauseControls(wrench_commander, reamer_commander, reamerVel))
        {
            ROS_INFO("Trying to pause Reaming Operation!");
            // ROS_INFO("Reamer Vel dynCompTrig: %f", (*reamerVel));
            ros::spinOnce();
        }
    }
}

void reamerVelCallback(const std_msgs::Float64::ConstPtr &msg, double *reamerVel) {
    (*reamerVel) = (*msg).data;
}

void forceSensorCallback(const geometry_msgs::Vector3::ConstPtr &msg, double force[3]) {
    force[0] = (*msg).x;
    force[1] = (*msg).y;
    force[2] = (*msg).z;
}

/**
 * @brief Get the norms of the dx vector.
 *
 * @param norms the norms of the dx vector. First element is norm of position error; second element is norm of orientation error
 * @param dx the error of the pose between current ee pose and trajectory ee pose
 */
void calculateNorms(double norms[2], const double dx[6])
{
    double accum[2] = {0.0};
    for (int i = 0; i < 3; i++)
    {
        accum[0] += dx[i] * dx[i];
        accum[1] += dx[i + 3] * dx[i + 3];
    }
    norms[0] = sqrt(accum[0]);
    norms[1] = sqrt(accum[1]);
}

/**
 * @brief Get the norms of the wrench vector.
 *
 * @param norms the norms of the wrench vector. First element is norm of forces; second element is norm of torques
 * @param wrench the wrench to apply
 */
void calculateNorms(float norms[2], const float wrench[6])
{
    float accum[2] = {0.0};
    for (int i = 0; i < 3; i++)
    {
        accum[0] += wrench[i] * wrench[i];
        accum[1] += wrench[i + 3] * wrench[i + 3];
    }
    norms[0] = sqrt(accum[0]);
    norms[1] = sqrt(accum[1]);
}

void calculateDX(double dx[6], const std::array<double, 6> x1, const double x2[6], const tf::Matrix3x3 desiredRot, const tf::Matrix3x3 currentRot)
{
    for (int i = 0; i < 3; i++)
    {
        dx[i] = x1[i] - x2[i];
    }

    tf::Matrix3x3 transRot = currentRot * desiredRot.inverse();
    transRot.getRPY(dx[3], dx[4], dx[5]);
}

void calculateFullWrench(float wrench[6], const float Kp[6], const float Ki[6], const float Kd[6], const tf::Matrix3x3 rot, const double dx[6], const double velocities[6], float accumError[6], ros::Time *time, const float maxForce, const float maxTorque)
{
    // Calculate wrenches using PD control
    // Wrenches will be in base_link frame
    for (int i = 0; i < 3; i++)
    {
        wrench[i] = Kp[i] * dx[i] + Ki[i]*accumError[i] - Kd[i] * velocities[i];
    }

    double rototatedAngVel[3] = {0.0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rototatedAngVel[i] += rot[i][j] * velocities[j + 3];
        }
    }
    // ROS_INFO("Ang Vel: %f, %f, %f", rototatedAngVel[0], rototatedAngVel[1], rototatedAngVel[2]);
    for (int i = 3; i < 6 ; i++)
    {
        wrench[i] = Kp[i] * dx[i] + Ki[i]*accumError[i] - Kd[i] * rototatedAngVel[i-3];
    }

    ros::Duration elapsed = ros::Time::now() - (*time);
    double dt = elapsed.toSec();

    for (int i = 0; i < 6; i++) {
        if (Ki[i] != 0) {
            accumError[i] += dx[i]*dt;
            if (i < 3) {
                accumError[i] = std::max(-1*maxForce/Ki[i], std::min(maxForce/Ki[i], accumError[i]));
            } else {
                accumError[i] = std::max(-1*maxTorque/Ki[i], std::min(maxTorque/Ki[i], accumError[i]));
            }
        }
    }
    
    float norms[2] = {0.0};
    calculateNorms(norms, wrench);
    if (norms[0] > maxForce)
    {
        for (int i = 0; i < 3; i++)
        {
            wrench[i] = maxForce * wrench[i] / norms[0];
        }
    }
    if (norms[1] > maxTorque)
    {
        for (int i = 3; i < 6; i++)
        {
            wrench[i] = maxTorque * wrench[i] / norms[1];
        }
    }
    (*time) = ros::Time::now();
}

void calculateOrientationWrench(float wrench[6], const float Kp[6], const float Ki[6], const float Kd[6], const tf::Matrix3x3 rot, const double dx[6], const double velocities[6], float accumError[6], ros::Time *time, const float maxForce, const float maxTorque)
{
    // Calculate wrenches using PD control
    // Wrenches will be in base_link frame
    for (int i = 0; i < 3; i++)
    {
        wrench[i] = 0;
    }

    double rototatedAngVel[3] = {0.0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rototatedAngVel[i] += rot[i][j] * velocities[j + 3];
        }
    }
    // ROS_INFO("Ang Vel: %f, %f, %f", rototatedAngVel[0], rototatedAngVel[1], rototatedAngVel[2]);
    for (int i = 3; i < 6 ; i++)
    {
        wrench[i] = Kp[i] * dx[i] + Ki[i]*accumError[i] - Kd[i] * rototatedAngVel[i-3];
    }

    ros::Duration elapsed = ros::Time::now() - (*time);
    double dt = elapsed.toSec();

    for (int i = 0; i < 6; i++) {
        if (Ki[i] != 0) {
            accumError[i] += dx[i]*dt;
            if (i < 3) {
                accumError[i] = std::max(-1*maxForce/Ki[i], std::min(maxForce/Ki[i], accumError[i]));
            } else {
                accumError[i] = std::max(-1*maxTorque/Ki[i], std::min(maxTorque/Ki[i], accumError[i]));
            }
        }
    }
    
    float norms[2] = {0.0};
    calculateNorms(norms, wrench);
    if (norms[0] > maxForce)
    {
        for (int i = 0; i < 3; i++)
        {
            wrench[i] = maxForce * wrench[i] / norms[0];
        }
    }
    if (norms[1] > maxTorque)
    {
        for (int i = 3; i < 6; i++)
        {
            wrench[i] = maxTorque * wrench[i] / norms[1];
        }
    }
    (*time) = ros::Time::now();
}

void calculatePositionWrench(float wrench[6], const float Kp[6], const float Ki[6], const float Kd[6], const tf::Matrix3x3 rot, const double dx[6], const double velocities[6], float accumError[6], ros::Time *time, const float maxForce, const float maxTorque)
{
    // Calculate wrenches using PD control
    // Wrenches will be in base_link frame
    for (int i = 0; i < 3; i++)
    {
        wrench[i] = Kp[i] * dx[i] + Ki[i]*accumError[i] - Kd[i] * velocities[i];
    }

    for (int i = 3; i < 6 ; i++)
    {
        wrench[i] = 0;
    }

    ros::Duration elapsed = ros::Time::now() - (*time);
    double dt = elapsed.toSec();

    for (int i = 0; i < 6; i++) {
        if (Ki[i] != 0) {
            accumError[i] += dx[i]*dt;
            // if (i < 3) {
            //     accumError[i] = std::max(-3*maxForce/Ki[i], std::min(3*maxForce/Ki[i], accumError[i]));
            // } else {
            //     accumError[i] = std::max(-3*maxTorque/Ki[i], std::min(3*maxTorque/Ki[i], accumError[i]));
            // }
        }
    }
    
    float norms[2] = {0.0};
    calculateNorms(norms, wrench);
    if (norms[0] > maxForce)
    {
        for (int i = 0; i < 3; i++)
        {
            wrench[i] = maxForce * wrench[i] / norms[0];
        }
    }
    if (norms[1] > maxTorque)
    {
        for (int i = 3; i < 6; i++)
        {
            wrench[i] = maxTorque * wrench[i] / norms[1];
        }
    }
    (*time) = ros::Time::now();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrench_controller");

    ros::NodeHandle node;
    
    double controller_rate = 60.0; // Rate of ros node pubsub

    std_msgs::Bool planner_msg;
    bool startPlanner = false;
    bool planned = false;
    bool finished = false;
    bool dynamicComp = false;
    bool stopRecordingTraj = false;

    std_msgs::Int16 reamer_msg;
    reamer_msg.data = 0;
    int defaultSpeed = 300; // default speed of reamer when starting reaming (rpm)
    double reamerVel = 0.0; // variable to keep track of current reamer velocity (rpm)

    double xyzrpy[6] = {NAN};  
    std::array<double, 6> pelvis_xyzrpy = {NAN}; // pelvis_xyzrpy holds the pelvis frame pose wrt base_link frame
    std::array<double, 6> retract_xyzrpy = {NAN}; // retract_xyzrpy holds the retraction frame pose wrt base_link frame
    tf::Matrix3x3 currentRot = tf::Matrix3x3(); // holds the rotation matrix from base_link to ee frame
    tf::Quaternion qForTrajEval = tf::Quaternion(); 
    tf::Matrix3x3 pelvisRot = tf::Matrix3x3();
    std::vector<tf::Matrix3x3> desiredRots;
    // tf::Quaternion q = tf::Quaternion();
    double velocities[6] = {NAN};        // holds the lin and ang velocities of ee frame wrt base_link frame
    float accumError[6] = {0};
    ros::Time time = ros::Time::now();   // records the current time to be used when calculating dt for velocities
    ros::Time time2 = ros::Time::now();

    double forces[3] = {0}; // holds the forces measured from the force/torque sensor

    std::vector<std::array<double, 6>> traj; // holds the trajectory information
    int current_waypoint = -1;               // has the current waypoint
    int trajNum = -1;                        // variable to store whether new trajectory has come in

    // frame of wrench controller, check confluence for description
    int frame = 1;
    // duration of wrench command (ms)
    int duration = 1000;
    // mode of wrench controller (1 is restricted mode, only allows movement in direction of wrench); check confluence for details
    int mode = 1;

    // Kp and Kd constants for PD control

    const float defaultKp[6] = {500, 500, 500, 200, 200, 200};
    float Kp[6] = {0.0};
    const float defaultKi[6] = {75, 75, 75, 15, 15, 15};
    float Ki[6] = {0.0};
    // const float Ki[6] = {0, 0, 0, 0, 0, 0};
    const float Kd[6] = {650, 650, 650, 0, 0, 0};
    // Max F/T in N or Nm to apply
    const float maxForce = 15.0;
    const float maxTorque = 8.0;
    

    // sends wrench commands to kortex_driver service
    ros::ServiceClient wrench_commander = node.serviceClient<kortex_driver::SendWrenchCommand>("/my_gen3/base/send_wrench_command");
    // sends speeds to reamer
    ros::Publisher reamer_commander = node.advertise<std_msgs::Int16>("/reamer_speed", 1);
    // sub to get current reamer velocity
    ros::Subscriber reamer_sub = node.subscribe<std_msgs::Float64>("/reamer_velocity", 1, boost::bind(&reamerVelCallback, _1, &reamerVel));
    // sub to get ee pose wrt base_link
    ros::Subscriber transform_sub = node.subscribe<geometry_msgs::Transform>("/my_gen3/ee_tf", 1, boost::bind(&tfCallback, _1, xyzrpy, &retract_xyzrpy, &currentRot, velocities, &time, &dynamicComp, &finished, &qForTrajEval));
    // sub to get pelvis orientation wrt base_link
    ros::Subscriber pelvis_transform_sub = node.subscribe<geometry_msgs::PoseStamped>("/reaming_end_point", 1, boost::bind(&pelvisTFCallback, _1, &pelvis_xyzrpy, &pelvisRot));
    // sub to get trajectory information
    ros::Subscriber trajectory_sub = node.subscribe<arthur_planning::arthur_traj>("/my_gen3/arthur_traj", 1, boost::bind(&trajCallback, _1, &traj, &current_waypoint, &trajNum, &desiredRots, wrench_commander, reamer_commander, &planned, &reamerVel));
    // sub to see dynamic comp
    ros::Subscriber dynamic_compensation_listener = node.subscribe<std_msgs::Bool>("/pelvis_error", 1, boost::bind(&dynamicCompTrigger, _1, &dynamicComp, &startPlanner, &finished, &planned, wrench_commander, reamer_commander, &reamerVel));
    // sub to get forces at end-effector
    ros::Subscriber force_listener = node.subscribe<geometry_msgs::Vector3>("/sensor_force", 1, boost::bind(&forceSensorCallback, _1, forces));
    // pub to notify planner to start planning
    ros::Publisher notify_planner = node.advertise<std_msgs::Bool>("/start_planning", 1);
    // pub to send actual pose to trajectory evaluator
    ros::Publisher traj_eval_pub = node.advertise<geometry_msgs::Pose>("/actual_traj", 1);

    ros::Rate rate(controller_rate);
    while (ros::ok())
    {
        // ROS_INFO("ReamerVel Main Loop: %f", reamerVel);
        if (startPlanner) {
            planner_msg.data = startPlanner;
            notify_planner.publish(planner_msg);
            startPlanner = false;
        } else {
            planner_msg.data = startPlanner;
            notify_planner.publish(planner_msg);
        }

        for (int i = 0; i < 6; i++) {
            Kp[i] = defaultKp[i];
            Ki[i] = defaultKi[i];
        }
        
        // wrench to send; [force_x, force_y, force_z, torque_x, torque_y, torque_z]
        float wrench[6] = {0};
        // ROS_INFO("Measured Forces: %f, %f, %f", forces[0], forces[1], forces[2]);
        
        // Don't modify wrench if trajectory and ee frame aren't detected
        if (!isnan(xyzrpy[0]) && !isnan(velocities[0] && !isnan(pelvis_xyzrpy[0])))
        {
            // ROS_INFO("Traj Size: %li", traj.size());
            // ROS_INFO("Current Waypoint: %d", current_waypoint);
            // ROS_INFO("trajNum: %d", trajNum);
            // Controller Code
            // We align orientation to pelvis reaming axis, then we follow trajectory
            // If dynamic compensation kicks in, we stop reaming, and back up; then we reorient and restart
            if (!finished && !planned && !startPlanner && !dynamicComp)
            {
                
                // To align our ee, we use frame 1 (trans about base, rot about ee)
                frame = 1;

                // ROS_INFO("xyzrpy: %f, %f, %f, %f, %f, %f", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
                // ROS_INFO("traj_waypoint: %f, %f, %f, %f, %f, %f", traj[current_waypoint][0], traj[current_waypoint][1], traj[current_waypoint][2], traj[current_waypoint][3], traj[current_waypoint][4], traj[current_waypoint][5]);

                // Calculate the error in pose wrt base_link
                double dx[6] = {0.0};
                calculateDX(dx, pelvis_xyzrpy, xyzrpy, pelvisRot, currentRot);
                ROS_INFO("dx: %f, %f, %f, %f, %f, %f", dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]);

                // Calculate norm of translation error and norm of orientation error separately
                double norms[2] = {0.0};
                calculateNorms(norms, dx);

                // if the error in translation and orientation are both small, set next waypoint
                if (norms[0] < 7e-3) {
                    for (int i = 0; i < 3; i++) {
                        accumError[i] = 0;
                    }
                }
                if (norms[1] < 5e-2) {
                    for (int i = 3; i < 6; i++) {
                        accumError[i] = 0;
                    }
                }
                if (norms[0] < 7e-3 && norms[1] < 5e-2)
                {
                    startPlanner = true;
                    ROS_INFO("Starting Reaming");
                    
                } else {
                    ROS_INFO("Moving to start point!");
                }

                // Calculate the wrench vector (forces to translate in base_link frame, torques to orient in ee frame)
                if (!finished) {
                    calculateFullWrench(wrench, Kp, Ki, Kd, currentRot, dx, velocities, accumError, &time2, maxForce, maxTorque);
                } else {
                    calculatePositionWrench(wrench, Kp, Ki, Kd, currentRot, dx, velocities, accumError, &time2, maxForce, maxTorque);
                }
                
            } else if (!finished && planned && current_waypoint >= 0 && traj.size() > 0 && !dynamicComp) {
                ROS_INFO("Current Waypoint: %d out of %li", current_waypoint, traj.size()-1);
                Kp[0] = 2000;
                Kp[1] = 2000;
                Kp[2] = 2000;
                Kp[3] = 150;
                Kp[4] = 150;
                Kp[5] = 150;
                Ki[0] = 200;
                Ki[1] = 200;
                Ki[2] = 200;
                Ki[3] = 5;
                Ki[4] = 5;
                Ki[5] = 5;

                

                // To align our ee, we use frame 1 (trans about base, rot about ee)
                frame = 1;

                // ROS_INFO("xyzrpy: %f, %f, %f, %f, %f, %f", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
                // ROS_INFO("traj_waypoint: %f, %f, %f, %f, %f, %f", traj[current_waypoint][0], traj[current_waypoint][1], traj[current_waypoint][2], traj[current_waypoint][3], traj[current_waypoint][4], traj[current_waypoint][5]);

                // Calculate the error in pose wrt base_link
                double dx[6] = {0.0};
                calculateDX(dx, traj[current_waypoint], xyzrpy, desiredRots[current_waypoint], currentRot);
                ROS_INFO("dx: %f, %f, %f, %f, %f, %f", dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]);

                // Calculate norm of translation error and norm of orientation error separately
                double norms[2] = {0.0};
                calculateNorms(norms, dx);

                double accum = 0.0;
                for (int i = 0; i < 3; i++)
                {
                    accum += forces[i] * forces[i];
                }
                double normForces = sqrt(accum);
                ROS_INFO("Norm of Forces: %e", normForces);
                if (!finished && abs(reamerVel) <= 0.01 && normForces >= 3 && norms[0] < 3e-2) {
                    // TODO: Modify start reamer
                    reamer_msg.data = defaultSpeed;
                    reamer_commander.publish(reamer_msg);
                    ROS_INFO("Reamer On!");
                } else if (finished) {
                    reamer_msg.data = 0;
                    reamer_commander.publish(reamer_msg);
                }

                // if the error in translation and orientation are both small, set next waypoint
                if (norms[0] < 3.5e-3) {
                    if (!stopRecordingTraj) {
                        geometry_msgs::Pose eval_pose;
                        eval_pose.position.x = xyzrpy[0];
                        eval_pose.position.y = xyzrpy[1];
                        eval_pose.position.z = xyzrpy[2];
                        eval_pose.orientation.x = qForTrajEval.getX();
                        eval_pose.orientation.y = qForTrajEval.getY();
                        eval_pose.orientation.z = qForTrajEval.getZ();
                        eval_pose.orientation.w = qForTrajEval.getW();
                        traj_eval_pub.publish(eval_pose);
                    }
                    for (int i = 0; i < 6; i++) {
                        accumError[i] = 0;
                    }
                    if (current_waypoint >= traj.size()-1) {
                        finished = true;
                        // TODO: Add stop reamer code using pauseControls
                        while (!pauseControls(wrench_commander, reamer_commander, &reamerVel))
                        {
                            ROS_INFO("Trying to pause Reaming Operation!");
                            ros::spinOnce();
                        }
                    }
                    if (current_waypoint == traj.size()-1) {
                        stopRecordingTraj = true;
                    }
                    current_waypoint = std::max(0, std::min(current_waypoint + 1, (int)traj.size() - 1));
                }

                // Calculate the wrench vector (forces to translate in base_link frame, torques to orient in ee frame)
                calculatePositionWrench(wrench, Kp, Ki, Kd, currentRot, dx, velocities, accumError, &time2, maxForce, maxTorque);
            } else if (dynamicComp || finished) {
                if (dynamicComp) {
                    ROS_INFO("Dynamic Compensation; Retracting Arm");
                    reamer_msg.data = 0;
                    reamer_commander.publish(reamer_msg);
                } else {
                    ROS_INFO("FINISHED REAMING!!! YAY!!!");
                    reamer_msg.data = 0;
                    reamer_commander.publish(reamer_msg);
                }
                // To align our ee, we use frame 1 (trans about base, rot about ee)
                frame = 1;

                // ROS_INFO("xyzrpy: %f, %f, %f, %f, %f, %f", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
                // ROS_INFO("traj_waypoint: %f, %f, %f, %f, %f, %f", traj[current_waypoint][0], traj[current_waypoint][1], traj[current_waypoint][2], traj[current_waypoint][3], traj[current_waypoint][4], traj[current_waypoint][5]);

                // Calculate the error in pose wrt base_link
                double dx[6] = {0.0};
                calculateDX(dx, retract_xyzrpy, xyzrpy, pelvisRot, currentRot);
                ROS_INFO("dx: %f, %f, %f, %f, %f, %f", dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]);

                // Calculate norm of translation error and norm of orientation error separately
                double norms[2] = {0.0};
                calculateNorms(norms, dx);
                // ROS_INFO("Normal: %f", norms[0]);
                if (norms[0] < 3e-2) {
                    if (dynamicComp) {
                        dynamicComp = false;
                    }
                    for (int i = 0; i < 6; i++) {
                        accumError[i] = 0;
                    }
                }

                calculatePositionWrench(wrench, Kp, Ki, Kd, currentRot, dx, velocities, accumError, &time2, maxForce, maxTorque);
            }
        }
        
        // sends wrench to robot
        if (!isnan(wrench[0]) && !isnan(wrench[1]) && !isnan(wrench[2]) && !isnan(wrench[3]) && !isnan(wrench[4]) && !isnan(wrench[5])) {
            if (sendWrench(wrench_commander, frame, mode, duration, wrench))
            {
                ROS_INFO("Sent wrench command successfully: %.6f %.6f %.6f %.6f %.6f %.6f", wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
            }
            else
            {
                ROS_INFO("Failed to send wrench command");
            }
        } else {
            pauseControls(wrench_commander, reamer_commander, &reamerVel);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};

/**
 * @brief This is code for calculating dx and then wrenches
 * 
 */
// // To align our ee, we use frame 1 (trans about base, rot about ee)
// frame = 1;

// // ROS_INFO("xyzrpy: %f, %f, %f, %f, %f, %f", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
// // ROS_INFO("traj_waypoint: %f, %f, %f, %f, %f, %f", traj[current_waypoint][0], traj[current_waypoint][1], traj[current_waypoint][2], traj[current_waypoint][3], traj[current_waypoint][4], traj[current_waypoint][5]);

// // Calculate the error in pose wrt base_link
// double dx[6] = {0.0};
// calculateDX(dx, traj[current_waypoint], xyzrpy, desiredRots[current_waypoint], currentRot);
// ROS_INFO("dx: %f, %f, %f, %f, %f, %f", dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]);

// // Calculate norm of translation error and norm of orientation error separately
// double norms[2] = {0.0};
// calculateNorms(norms, dx);

// // if the error in translation and orientation are both small, set next waypoint
// if (norms[0] < 1e-3) {
//     for (int i = 0; i < 3; i++) {
//         accumError[i] = 0;
//     }
// }
// if (norms[1] < 1e-2) {
//     for (int i = 3; i < 6; i++) {
//         accumError[i] = 0;
//     }
// }
// if (norms[0] < 1e-3 && norms[1] < 1e-2)
// {
//     current_waypoint = std::max(0, std::min(current_waypoint + 1, (int)traj.size() - 1));
// }

// // Calculate the wrench vector (forces to translate in base_link frame, torques to orient in ee frame)
// calculateFullWrench(wrench, Kp, Ki, Kd, currentRot, dx, velocities, accumError, &time2, maxForce, maxTorque);

// void getSkewMat(double skewMat[3][3], const tf::Quaternion q) {
//     skewMat[0][1] = -1*q.getZ();
//     skewMat[1][0] = q.getZ();
//     skewMat[0][2] = q.getY();
//     skewMat[2][0] = -1*q.getY();
//     skewMat[1][2] = -1*q.getX();
//     skewMat[2][1] = q.getX();
// }

// void getL(double L[4][4], const tf::Quaternion q) {
//     double skewMat[3][3] = {0};
//     getSkewMat(skewMat, q);
//     double sI[3][3] = {0};
//     sI[0][0] = q.getW();
//     sI[1][1] = q.getW();
//     sI[2][2] = q.getW();
//     L[0][0] = q.getW();
//     L[0][1] = -1*q.getX();
//     L[0][2] = -1*q.getY();
//     L[0][3] = -1*q.getZ();
//     L[1][0] = q.getX();
//     L[2][0] = q.getY();
//     L[3][0] = q.getZ();
//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             L[i+1][j+1] = sI[i][j] - skewMat[i][j];
//         }
//     }
// }


/**
 * @brief This is code for quaternion math
 * 
 */
// void transpose(double transposed[4][4], const double M[4][4]) {
//     for (int i = 0; i < 4; i++) {
//         for (int j = 0; j < 4; j++) {
//             transposed[i][j] = M[j][i];
//         }
//     }
// }

// void getDq(tf::Quaternion *dq, const tf::Quaternion q1, const tf::Quaternion q2) {
//     double L[4][4] = {0};
//     getL(L, q1);
//     double LTransposed[4][4] = {0};
//     transpose(LTransposed, L);
//     double q2_wxyz[4] = {0};
//     q2_wxyz[0] = q2.getW();
//     q2_wxyz[1] = q2.getX();
//     q2_wxyz[2] = q2.getY();
//     q2_wxyz[3] = q2.getZ();

//     (*dq).setW(LTransposed[0][0]*q2_wxyz[0] + LTransposed[0][1]*q2_wxyz[1] + LTransposed[0][2]*q2_wxyz[2] + LTransposed[0][3]*q2_wxyz[3]);
//     (*dq).setX(LTransposed[1][0]*q2_wxyz[0] + LTransposed[1][1]*q2_wxyz[1] + LTransposed[1][2]*q2_wxyz[2] + LTransposed[1][3]*q2_wxyz[3]);
//     (*dq).setY(LTransposed[2][0]*q2_wxyz[0] + LTransposed[2][1]*q2_wxyz[1] + LTransposed[2][2]*q2_wxyz[2] + LTransposed[2][3]*q2_wxyz[3]);
//     (*dq).setZ(LTransposed[3][0]*q2_wxyz[0] + LTransposed[3][1]*q2_wxyz[1] + LTransposed[3][2]*q2_wxyz[2] + LTransposed[3][3]*q2_wxyz[3]);
// }