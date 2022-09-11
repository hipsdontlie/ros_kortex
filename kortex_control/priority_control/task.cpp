#include "task.hpp"

namespace priority_control
{

    Task::Task(std::shared_ptr<ArthurRobotModel> robot, std::array<bool, ArthurRobotModel::CARTESIAN_DOF> dof, const std::string& task_frame)
    {
        // TODO: add error checking for inputs
        robot_ = robot;
        task_dof_ = dof;
        num_task_dof_ = 0;
        for (size_t i = 0; i < ArthurRobotModel::CARTESIAN_DOF; ++i)
        {
            if (task_dof_[i])
                ++num_task_dof_;
        }
        task_frame_ = task_frame;
        q_pos_ = KDL::JntArray(robot_->nj());
        basic_jacobian_ = KDL::Jacobian(robot_->nj());
        rotated_jacobian_ = KDL::Jacobian(robot_->nj());
        cropped_jacobian_ = Eigen::MatrixXd::Zero(num_task_dof_, robot_->nj());
        task_jacobian_ = Eigen::MatrixXd::Zero(num_task_dof_, robot_->nj());
        pseudoinverse_jacobian_ = Eigen::MatrixXd::Zero(robot_->nj(), num_task_dof_);
        task_twist_ = Eigen::VectorXd::Zero(num_task_dof_);
        I_ = Eigen::MatrixXd::Identity(num_task_dof_, num_task_dof_);
    }

    
    Eigen::MatrixXd const& Task::basic_jacobian()
    {
        return basic_jacobian_.data;
    }
    Eigen::MatrixXd const& Task::rotated_jacobian()
    {
        return rotated_jacobian_.data;
    }
    Eigen::MatrixXd const& Task::cropped_jacobian()
    {
        return cropped_jacobian_;
    }
    Eigen::MatrixXd const& Task::task_jacobian()
    {
        return task_jacobian_;
    }
    Eigen::MatrixXd const& Task::pseudoinverse_jacobian()
    {
        return pseudoinverse_jacobian_;
    }
    Eigen::MatrixXd const& Task::identity_matrix()
    {
        return I_;
    }
    Eigen::VectorXd const& Task::task_twist()
    {
        return task_twist_;
    }

    bool Task::update_task(const KDL::JntArray& q_pos, const Eigen::VectorXd& task_twist)
    {
        q_pos_ = q_pos;
        task_twist_ = task_twist;
        // TODO: error check
        return true;
    }

    bool Task::compute_kinematic_matrices(const Eigen::MatrixXd& joint_limit_avoidance_Wq)
    {
        if (!compute_jacobian(joint_limit_avoidance_Wq))
        {
            // TODO: error!
            return false;
        }
        if (!compute_pseudoinverse_jacobian())
        {
            // TODO: error!
            return false;
        }
        return true;
    }

    bool Task::compute_jacobian(const Eigen::MatrixXd& joint_limit_avoidance_Wq)
    {
        // TODO: Implement segnr for jacobian calculation
        if (robot_->jac_solver_->JntToJac(q_pos_, basic_jacobian_) < 0)
        {
            ROS_ERROR("Could not compute jacobian!");
            // TODO: Stop Controller or Send Message to Watchdog
            return false;
        }
        // TODO: Rotate jacobian?
        if (!rotate_jacobian())
        {
            // TODO: error!
        }
        // TODO: Crop Jacobian?
        if (!crop_jacobian())
        {
            // TODO: error!
        }
        // TODO: Multiple Jacobian by weighting matrices?
        task_jacobian_ = cropped_jacobian_ * joint_limit_avoidance_Wq;
        return true;
    }

    bool Task::rotate_jacobian()
    {
        // TODO: Implement SegNr for fk calculation
        KDL::Frame task_frame = KDL::Frame();
        if (robot_->fk_pos_solver_->JntToCart(q_pos_, task_frame) < 0)
        {
            // TODO: error!
            return false;
        }
        if (!KDL::changeBase(basic_jacobian_, task_frame.M.Inverse(), rotated_jacobian_))
        {
            // TODO: error!
            return false;
        }
        return true;
    }

    bool Task::crop_jacobian()
    {
        int row_index = 0;
        for (size_t i = 0; i < ArthurRobotModel::CARTESIAN_DOF; ++i)
        {
            if (task_dof_[i])
            {
                cropped_jacobian_.row(row_index) = rotated_jacobian_.data.row(i);
                ++row_index;
            }
        }
        // TODO: error check
        return true;
    }

    bool Task::compute_pseudoinverse_jacobian()
    {
        // TODO: Check singularities
        robot_->svd_full_solver_->compute(task_jacobian_);
        // std::cout << "-----------------------" << std::endl;
        // std::cout << robot_->svd_full_solver_->singularValues() << std::endl;
        std::cout << "==========================" << std::endl;
        std::cout << task_jacobian_ << std::endl;
        pseudoinverse_jacobian_ = Eigen::MatrixXd::Zero(robot_->nj(), num_task_dof_);
        for (size_t i = 0; i < ArthurRobotModel::CARTESIAN_DOF; ++i)
        {
            pseudoinverse_jacobian_ = pseudoinverse_jacobian_ +
            ((1.0 / robot_->svd_full_solver_->singularValues()(i)) *
            robot_->svd_full_solver_->matrixV().col(i) *
            robot_->svd_full_solver_->matrixU().col(i).transpose());
        }
        // TODO: check errors
        return true;
    }
}