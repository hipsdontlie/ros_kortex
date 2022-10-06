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

    
    KDL::Jacobian const& Task::basic_jacobian()
    {
        return basic_jacobian_;
    }
    KDL::Jacobian const& Task::rotated_jacobian()
    {
        return rotated_jacobian_;
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

    bool Task::set_q_vel(Eigen::VectorXd q_vel)
    {
        if ((size_t) q_vel.rows() != robot_->nj())
        {
            // TODO: error
            return false;
        }
        q_vel_ = q_vel;
        return true;
    }

    Eigen::VectorXd const& Task::get_q_vel()
    {
        return q_vel_;
    }

    std::array<bool, ArthurRobotModel::CARTESIAN_DOF> const& Task::task_dof()
    {
        return task_dof_;
    }

    bool Task::update_task(Eigen::VectorXd& task_twist)
    {
        int num_linear_dof = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (task_dof()[i])
                ++num_linear_dof;
        }
        int num_angular_dof = 0;
        for (int i = 3; i < robot_->CARTESIAN_DOF; ++i)
        {
            if (task_dof()[i])
                ++num_angular_dof;
        }
        double linearVel = 0;
        for (int i = 0; i < num_linear_dof; ++i)
        {
            linearVel += task_twist(i)*task_twist(i);
        }
        linearVel = sqrt(linearVel);
        double angularVel = 0;
        for (int i = num_linear_dof; i < num_linear_dof+num_angular_dof; ++i)
        {
            angularVel += task_twist(i)*task_twist(i);
        }
        angularVel = sqrt(angularVel);
        if (linearVel > robot_->MAX_LINEAR_VELOCITY)
        {
            for (int i = 0; i < num_linear_dof; ++i)
            {
                task_twist(i) = robot_->MAX_LINEAR_VELOCITY * task_twist(i) / linearVel;
            }
        }
        if (angularVel > robot_->MAX_ANGULAR_VELOCITY)
        {
            for (int i = num_linear_dof; i < num_linear_dof+num_angular_dof; ++i)
            {
                task_twist(i) = robot_->MAX_ANGULAR_VELOCITY * task_twist(i) / angularVel;
            }
        }
        task_twist_ = task_twist;
        // TODO: error check
        return true;
    }

    bool Task::compute_kinematic_matrices(const KDL::JntArray& q_pos, const Eigen::MatrixXd& joint_limit_avoidance_Wq, const Eigen::MatrixXd& null_space_projector)
    {
        q_pos_ = q_pos;
        // std::cout << "Check 2.311" << std::endl;
        if (!compute_jacobian(joint_limit_avoidance_Wq, null_space_projector))
        {
            // TODO: error!
            return false;
        }
        // std::cout << "Check 2.312" << std::endl;
        if (!compute_pseudoinverse_jacobian())
        {
            // TODO: error!
            return false;
        }
        // std::cout << "Check 2.313" << std::endl;
        return true;
    }

    bool Task::compute_jacobian(const Eigen::MatrixXd& joint_limit_avoidance_Wq, const Eigen::MatrixXd& null_space_projector)
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
        task_jacobian_ = cropped_jacobian_ * joint_limit_avoidance_Wq * null_space_projector;
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
        // std::cout << "==========================" << std::endl;
        // std::cout << task_jacobian_ << std::endl;
        pseudoinverse_jacobian_ = Eigen::MatrixXd::Zero(robot_->nj(), num_task_dof_);
        int max_rank = std::min(task_jacobian_.rows(), task_jacobian_.cols());
        for (int i = 0; i < max_rank; ++i)
        {
            if (robot_->svd_full_solver_->singularValues()(i) < ArthurRobotModel::DEFAULT_EPSILON)
            {
                double lambda_scaled = compute_lambda_scaled(robot_->svd_full_solver_->singularValues()(i));
                pseudoinverse_jacobian_ = pseudoinverse_jacobian_ +
                ((robot_->svd_full_solver_->singularValues()(i) / 
                ((robot_->svd_full_solver_->singularValues()(i) * robot_->svd_full_solver_->singularValues()(i)) + 
                (lambda_scaled * lambda_scaled))) *
                robot_->svd_full_solver_->matrixV().col(i) *
                robot_->svd_full_solver_->matrixU().col(i).transpose());
            }
            else
            {
                pseudoinverse_jacobian_ = pseudoinverse_jacobian_ +
                ((1.0 / robot_->svd_full_solver_->singularValues()(i)) *
                robot_->svd_full_solver_->matrixV().col(i) *
                robot_->svd_full_solver_->matrixU().col(i).transpose());
            }
            
        }
        // TODO: check errors
        return true;
    }

    double Task::compute_lambda_scaled(double sigma)
    {
        return ArthurRobotModel::DEFAULT_LAMBDA * sqrt(1 - ((sigma * sigma) / (ArthurRobotModel::DEFAULT_EPSILON * ArthurRobotModel::DEFAULT_EPSILON)));
    }
}