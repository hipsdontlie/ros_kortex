#pragma once

#include "arthur_robot_model.hpp"
#include <ros/console.h>
#include <math.h>

namespace priority_control 
{
    class Task
    {
        public:
        Task(std::shared_ptr<ArthurRobotModel> robot, std::array<bool, ArthurRobotModel::CARTESIAN_DOF> dof, const std::string& task_frame);

        KDL::Jacobian const& basic_jacobian();
        KDL::Jacobian const& rotated_jacobian();
        Eigen::MatrixXd const& cropped_jacobian();
        Eigen::MatrixXd const& task_jacobian();
        Eigen::MatrixXd const& pseudoinverse_jacobian();
        Eigen::MatrixXd const& identity_matrix();
        Eigen::VectorXd const& task_twist();
        bool update_task(Eigen::VectorXd& task_twist);
        bool compute_kinematic_matrices(const KDL::JntArray& q_pos, const Eigen::MatrixXd& joint_limit_avoidance_Wq, const Eigen::MatrixXd& null_space_projector);
        bool set_q_vel(Eigen::VectorXd q_vel);
        Eigen::VectorXd const& get_q_vel();
        std::array<bool, ArthurRobotModel::CARTESIAN_DOF> const& task_dof();
        size_t num_task_dof();
        int rank();

        protected:
        size_t num_task_dof_;
        std::array<bool, ArthurRobotModel::CARTESIAN_DOF> task_dof_;
        std::string task_frame_;
        std::shared_ptr<ArthurRobotModel> robot_;
        KDL::JntArray q_pos_;
        KDL::Jacobian basic_jacobian_;
        KDL::Jacobian rotated_jacobian_;
        Eigen::MatrixXd cropped_jacobian_;
        Eigen::MatrixXd task_jacobian_;
        Eigen::MatrixXd pseudoinverse_jacobian_;
        Eigen::VectorXd task_twist_;
        Eigen::MatrixXd I_;
        Eigen::VectorXd q_vel_;
        int task_jacobian_rank_;
        
        bool compute_jacobian(const Eigen::MatrixXd& joint_limit_avoidance_Wq, const Eigen::MatrixXd& null_space_projector);
        bool rotate_jacobian();
        bool crop_jacobian();
        bool compute_pseudoinverse_jacobian();
        double compute_lambda_scaled(double sigma);
    };
}
