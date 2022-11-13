#pragma once

#include "task.hpp"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <cmath>

namespace priority_control
{
    class PelvisAlignmentTask : public Task
    {
        public:
        static constexpr double MAX_TRANSLATIONAL_ERROR = 2; // mm
        static constexpr double MAX_ORIENTATION_ERROR = 1.5; // deg

        PelvisAlignmentTask(std::shared_ptr<ArthurRobotModel> robot, const std::string& task_frame);
        bool updateError(const KDL::JntArray& q_pos, const geometry_msgs::Transform& error, const Eigen::VectorXd& twist, ros::Publisher& error_metrics_pub);
        bool errorAboveThreshold();

        protected:
        bool error_above_thresh_;
        Eigen::VectorXd desired_twist_;
        Eigen::VectorXd pidController(const geometry_msgs::Transform& error, const Eigen::VectorXd& twist, ros::Publisher& error_metrics_pub);
    };
}
