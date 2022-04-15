#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <arthur_planning/arthur_traj.h>
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
bool pauseControls(ros::ServiceClient wrench_commander)
{
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
    return wrench_commander.call(srv);
}

// Updates the ee pose wrt to the base_link
void tfCallback(const geometry_msgs::Transform::ConstPtr &msg, double xyzrpy[6], tf::Matrix3x3 *rot, double velocities[6], ros::Time *time)
{
    // extract pose from msg
    double pose[6];
    pose[0] = (*msg).translation.x;
    pose[1] = (*msg).translation.y;
    pose[2] = (*msg).translation.z;
    tf::Quaternion q = tf::Quaternion((*msg).rotation.x, (*msg).rotation.y, (*msg).rotation.z, (*msg).rotation.w);
    // save orientation as rotation matrix for rotating torques later
    (*rot) = tf::Matrix3x3(q);
    (*rot).getRPY(pose[4], pose[5], pose[6]);

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
    (*time) = ros::Time::now();
    // ROS_INFO("%f, %f, %f, %f, %f, %f", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
}

// Checks to see if there is a new trajectory. If yes, then update trajectory vector
void trajCallback(const arthur_planning::arthur_traj::ConstPtr &msg, std::vector<std::array<double, 6>> *traj, int *current_waypoint, int *trajNum, ros::ServiceClient wrench_commander)
{
    // if new trajectory, do the following
    if ((*msg).trajNum != (*trajNum))
    {
        // stop wrench commands, and pause robot at current spot
        while (!pauseControls(wrench_commander))
        {
            ROS_INFO("Trying to pause Reaming Operation!");
        }
        ROS_INFO("Pausing Reaming Operation for 3 seconds");

        // clear trajectory vector
        (*traj).clear();
        // for each pose in trajectory, store it in array and push into trajectory vector
        int len = (*msg).cartesian_states.poses.size();
        for (int i = 0; i < len; i++)
        {
            std::array<double, 6> a = {0};
            a[0] = (*msg).cartesian_states.poses[i].position.x;
            a[1] = (*msg).cartesian_states.poses[i].position.y;
            a[2] = (*msg).cartesian_states.poses[i].position.z;
            tf::Quaternion q = tf::Quaternion((*msg).cartesian_states.poses[i].orientation.x, (*msg).cartesian_states.poses[i].orientation.y, (*msg).cartesian_states.poses[i].orientation.z, (*msg).cartesian_states.poses[i].orientation.w);
            tf::Matrix3x3(q).getRPY(a[4], a[5], a[6]);
            (*traj).push_back(a);
        }
        (*current_waypoint) = 0;     // reset waypoint back to 0
        (*trajNum) = (*msg).trajNum; // save current trajectory number for detecting new trajectory

        // pause control node for 3 seconds
        ros::Duration(3).sleep();
    }
    // if ((*traj).size() > 0) {
    //     ROS_INFO("Last Waypoint: %f, %f, %f, %f, %f, %f", (*traj).back()[0], (*traj).back()[1], (*traj).back()[2], (*traj).back()[3], (*traj).back()[4], (*traj).back()[5]);
    // }
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

void calculateDX(double dx[6], const std::array<double, 6> x1, const double x2[6])
{
    for (int i = 0; i < 6; i++)
    {
        dx[i] = x1[i] - x2[i];
    }
}

void calculateFullWrench(float wrench[6], const double Kp[6], const double Kd[6], const tf::Matrix3x3 rot, const double dx[6], const double velocities[6], const float maxForce, const float maxTorque)
{
    // Calculate wrenches using PD control
    // Wrenches will be in base_link frame
    for (int i = 0; i < 6; i++)
    {
        wrench[i] = Kp[i] * dx[i] - Kd[i] * velocities[i];
    }
    ROS_INFO("PD Wrenches: %f %f %f %f %f %f", wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
    // Rotate torques to be in tool frame
    double rotTorques[3] = {0.0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotTorques[i] += rot[i][j] * wrench[j + 3];
        }
    }
    ROS_INFO("Rot Torques: %f %f %f", rotTorques[0], rotTorques[1], rotTorques[2]);
    for (int i = 0; i < 3; i++)
    {
        wrench[i + 3] = rotTorques[i];
    }
    ROS_INFO("PD Wrenches After Rotation: %f %f %f %f %f %f", wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
    // Calculate norms of forces and torques; if above maximums, scale wrench
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
}

void calculateZWrench(float wrench[6], const double Kp[6], const double Kd[6], const double dx[6], const double velocities[6], const float maxForce)
{
    // Calculate wrenches using PD control
    // Wrenches will be in ee frame
    float forces[3] = {0.0};
    for (int i = 0; i < 3; i++)
    {
        forces[i] = Kp[i] * dx[i] - Kd[i] * velocities[i];
    }

    float accum = 0.0;
    for (int i = 0; i < 3; i++)
    {
        accum += forces[i] * forces[i];
    }
    float normForce = sqrt(accum);

    wrench[0] = 0;
    wrench[1] = 0;
    wrench[2] = std::min(normForce, maxForce);
    wrench[3] = 0;
    wrench[4] = 0;
    wrench[5] = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrench_controller");

    ros::NodeHandle node;
    
    double controller_rate = 10.0; // Rate of ros node pubsub

    double xyzrpy[6] = {NAN};            // xyzrpy holds the ee frame pose wrt base_link frame
    tf::Matrix3x3 rot = tf::Matrix3x3(); // holds the rotation matrix from base_link to ee frame
    double velocities[6] = {NAN};        // holds the lin and ang velocities of ee frame wrt base_link frame
    ros::Time time = ros::Time::now();   // records the current time to be used when calculating dt for velocities

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
    const double Kp[6] = {500, 500, 500, 0, 0, 0};
    const double Kd[6] = {350, 350, 350, 0, 0, 0};
    // Max F/T in N or Nm to apply
    const float maxForce = 15.0;
    const float maxTorque = 5.0;

    

    // sends wrench commands to kortex_driver service
    ros::ServiceClient wrench_commander = node.serviceClient<kortex_driver::SendWrenchCommand>("/my_gen3/base/send_wrench_command");
    // sub to get ee pose wrt base_link
    ros::Subscriber transform_sub = node.subscribe<geometry_msgs::Transform>("/my_gen3/ee_tf", 1, boost::bind(&tfCallback, _1, xyzrpy, &rot, velocities, &time));
    // sub to get trajectory information
    ros::Subscriber trajectory_sub = node.subscribe<arthur_planning::arthur_traj>("/my_gen3/arthur_traj", 1, boost::bind(&trajCallback, _1, &traj, &current_waypoint, &trajNum, wrench_commander));

    ros::Rate rate(controller_rate);
    while (ros::ok())
    {
        // wrench to send; [force_x, force_y, force_z, torque_x, torque_y, torque_z]
        float wrench[6] = {0};
        current_waypoint = traj.size()-1;
        // ROS_INFO("Traj Length: %li", traj.size());
        ROS_INFO("Current Waypoint: %d", current_waypoint);
        // Don't modify wrench if trajectory and ee frame aren't detected
        if (traj.size() > 0 && !isnan(xyzrpy[0]) && !isnan(velocities[0]))
        {
            // Controller Code
            // We align orientation and position for first waypoint (waypoint = 0)
            // Then we just ream along tool-frame z-axis (waypoint > 0)
            // if (current_waypoint == 0)
            if (current_waypoint == traj.size()-1)
            {
                // To align our ee, we use frame 1 (trans about base, rot about ee)
                frame = 1;

                ROS_INFO("xyzrpy: %f, %f, %f, %f, %f, %f", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
                ROS_INFO("traj_waypoint: %f, %f, %f, %f, %f, %f", traj[current_waypoint][0], traj[current_waypoint][1], traj[current_waypoint][2], traj[current_waypoint][3], traj[current_waypoint][4], traj[current_waypoint][5]);

                // Calculate the error in pose wrt base_link
                double dx[6] = {0.0};
                calculateDX(dx, traj[current_waypoint], xyzrpy);
                ROS_INFO("dx: %f, %f, %f, %f, %f, %f", dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]);

                // Calculate norm of translation error and norm of orientation error separately
                double norms[2] = {0.0};
                calculateNorms(norms, dx);
                // ROS_INFO("dx_norms: %f, %f", norms[0], norms[1]);

                // if the error in translation and orientation are both small, set next waypoint
                if (norms[0] < 2e-3 && norms[1] < 2e-2)
                {
                    current_waypoint = std::min(current_waypoint + 1, (int)traj.size() - 1);
                }

                // Calculate the wrench vector (forces to translate in base_link frame, torques to orient in ee frame)
                calculateFullWrench(wrench, Kp, Kd, rot, dx, velocities, maxForce, maxTorque);
            }
            // else if (current_waypoint > 0)
            // {
            //     // Now that our orientation is good, we can just apply force along z-axis of ee frame, use frame = 0
            //     frame = 0;

            //     // Calculate the error in pose wrt base_link
            //     double dx[6] = {0.0};
            //     calculateDX(dx, traj[current_waypoint], xyzrpy);

            //     // Calculate norm of translation error and norm of orientation error separately
            //     double norms[2] = {0.0};
            //     calculateNorms(norms, dx);

            //     // if the error in translation is small, set next waypoint
            //     if (norms[0] < 2e-3)
            //     {
            //         current_waypoint = std::min(current_waypoint + 1, (int)traj.size() - 1);
            //     }

            //     // Calculate the wrench vector (should only have force in z since we are using frame = 0)
            //     calculateZWrench(wrench, Kp, Kd, dx, velocities, maxForce);
            // }
        }

        // sends wrench to robot
        if (sendWrench(wrench_commander, frame, mode, duration, wrench))
        {
            ROS_INFO("Sent wrench command successfully: %.6f %.6f %.6f %.6f %.6f %.6f", wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
        }
        else
        {
            ROS_INFO("Failed to send wrench command");
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};