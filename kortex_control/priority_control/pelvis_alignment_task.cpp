#include "pelvis_alignment_task.hpp"

namespace priority_control
{
    constexpr double PelvisAlignmentTask::MAX_TRANSLATIONAL_ERROR; // mm
    constexpr double PelvisAlignmentTask::MAX_ORIENTATION_ERROR; // deg

    PelvisAlignmentTask::PelvisAlignmentTask(std::shared_ptr<ArthurRobotModel> robot, const std::string& task_frame) :
    Task(robot, {true, true, true, true, true, false}, task_frame)
    {
        desired_twist_ = Eigen::VectorXd::Zero(num_task_dof_);
        error_above_thresh_ = false;
    }

    bool PelvisAlignmentTask::updateError(const KDL::JntArray& q_pos, const geometry_msgs::Transform& error, const Eigen::VectorXd& twist, ros::Publisher& error_metrics_pub)
    {
        desired_twist_ = pidController(error, twist, error_metrics_pub);
        return update_task(desired_twist_);
    }

    bool PelvisAlignmentTask::errorAboveThreshold()
    {
        return error_above_thresh_;
    }

    Eigen::VectorXd PelvisAlignmentTask::pidController(const geometry_msgs::Transform& error, const Eigen::VectorXd& twist, ros::Publisher& error_metrics_pub)
    {
        std::array<double, 6> Kp = {1, 1, 1, 1, 1, 1};
        std::array<double, 6> Ki = {0, 0, 0, 0, 0, 0};
        std::array<double, 6> Kd = {0, 0, 0, 0, 0, 0};
        Eigen::VectorXd twist_command = Eigen::VectorXd::Zero(robot_->CARTESIAN_DOF);
        twist_command(0) = error.translation.x;
        twist_command(1) = error.translation.y;
        twist_command(2) = error.translation.z;
        tf::Matrix3x3 R = tf::Matrix3x3(tf::Quaternion(error.rotation.x, error.rotation.y, error.rotation.z, error.rotation.w));
        Eigen::Vector3d z0 = Eigen::Vector3d::Zero(3);
        z0(2) = 1;
        Eigen::Vector3d z1 = Eigen::Vector3d::Zero(3);
        z1(2) = 1;
        Eigen::Vector3d temp = Eigen::Vector3d::Zero(3);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                temp(i) = temp(i) + R[i][j] * z1(j);
            }
        }
        z1 = temp;
        double theta = acos(z0.dot(z1));
        Eigen::Vector3d axis = z0.cross(z1);
        tf::Quaternion quat = tf::Quaternion(0, 0, 0, 1);
        quat.setRotation(tf::Vector3(axis(0), axis(1), axis(2)), theta);
        if (theta > 0.0001 || theta < -0.0001)
        {
            tf::Matrix3x3(quat).getRPY(twist_command(3), twist_command(4), twist_command(5));
        }
        


        std_msgs::Float64MultiArray error_msg;
        error_msg.data.push_back(1000*sqrt(twist_command(0)*twist_command(0) + twist_command(1)*twist_command(1) + twist_command(2)*twist_command(2)));
        error_msg.data.push_back(180.0*sqrt(twist_command(3)*twist_command(3) + twist_command(4)*twist_command(4)) / M_PI);
        error_metrics_pub.publish(error_msg);

        error_above_thresh_ = error_msg.data[0] > PelvisAlignmentTask::MAX_TRANSLATIONAL_ERROR || error_msg.data[1] > PelvisAlignmentTask::MAX_ORIENTATION_ERROR;

        for (int i = 0; i < robot_->CARTESIAN_DOF; ++i)
        {
            twist_command(i) = Kp[i] * twist_command(i) - Kd[i]*twist(i);
        }
        Eigen::VectorXd task_twist_command = Eigen::VectorXd::Zero(num_task_dof_);
        
        int j = 0;
        for (int i = 0; i < robot_->CARTESIAN_DOF; ++i)
        {
            if (task_dof_[i])
            {
                task_twist_command(j) = twist_command(i);
                ++j;
            }
        }

        return task_twist_command;
    }
}