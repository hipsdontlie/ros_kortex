#include "pelvis_alignment_task.hpp"

namespace priority_control
{
    PelvisAlignmentTask::PelvisAlignmentTask(std::shared_ptr<ArthurRobotModel> robot, const std::string& task_frame) :
    Task(robot, {true, true, true, true, true, false}, task_frame)
    {
        desired_twist_ = Eigen::VectorXd::Zero(num_task_dof_);
    }

    bool PelvisAlignmentTask::updateError(const KDL::JntArray& q_pos, const geometry_msgs::Transform& error, const Eigen::VectorXd& twist)
    {
        desired_twist_ = pidController(error, twist);
        return update_task(desired_twist_);
    }

    Eigen::VectorXd PelvisAlignmentTask::pidController(const geometry_msgs::Transform& error, const Eigen::VectorXd& twist)
    {
        std::array<double, 6> Kp = {1, 1, 1, 1, 1, 1};
        std::array<double, 6> Ki = {0, 0, 0, 0, 0, 0};
        std::array<double, 6> Kd = {0, 0, 0, 0, 0, 0};
        Eigen::VectorXd twist_command = Eigen::VectorXd::Zero(robot_->CARTESIAN_DOF);
        twist_command(0) = error.translation.x;
        twist_command(1) = error.translation.y;
        twist_command(2) = error.translation.z;
        tf::Matrix3x3(tf::Quaternion(error.rotation.x, error.rotation.y, error.rotation.z, error.rotation.w)).getRPY(twist_command(3), twist_command(4), twist_command(5));

        for (int i = 0; i < robot_->CARTESIAN_DOF; ++i)
        {
            twist_command(i) = Kp[i] * twist_command(i) - Kd[i]*twist(i);
        }
        Eigen::VectorXd task_twist_command = Eigen::VectorXd::Zero(num_task_dof_);
        
        for (int i = 0; i < num_task_dof_; ++i)
        {
            task_twist_command(i) = twist_command(i);
        }

        return task_twist_command;
    }
}