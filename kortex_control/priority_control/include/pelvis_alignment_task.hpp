#pragma once

#include "task.hpp"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

namespace priority_control
{
    class PelvisAlignmentTask : public Task
    {
        public:
        PelvisAlignmentTask(std::shared_ptr<ArthurRobotModel> robot, const std::string& task_frame);
        bool updateError(const KDL::JntArray& q_pos, const geometry_msgs::Transform& error, const Eigen::VectorXd& twist);

        protected:
        Eigen::VectorXd desired_twist_;
        Eigen::VectorXd pidController(const geometry_msgs::Transform& error, const Eigen::VectorXd& twist);
    };
}
