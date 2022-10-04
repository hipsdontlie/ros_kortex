#include "priority_controller.hpp"

namespace priority_control
{
    PriorityController::PriorityController(std::shared_ptr<ArthurRobotModel> robot, double dt)
    {
        robot_ = robot;
        dt_ = dt;
        q_vel_cmd_ = Eigen::VectorXd::Zero(robot_->nj());
        q_vel_sum_ = Eigen::VectorXd::Zero(robot_->nj());
        q_pos_ = KDL::JntArray(robot_->nj());
        null_space_projector_ = Eigen::MatrixXd::Identity(robot_->nj(), robot_->nj());
        joint_lim_avoidance_Wq_ = Eigen::MatrixXd::Identity(robot_->nj(), robot_->nj());
        joint_lim_avoidance_F_ = Eigen::VectorXd::Zero(robot_->nj());
        singularity_avoidance_F_ = Eigen::VectorXd::Zero(robot_->nj());
    }

    bool PriorityController::addTask(std::shared_ptr<Task> task, size_t priority_num)
    {
        if (!tasks_.insert({priority_num, task}).second)
        {
            // TODO: error!
            return false;
        }
        return true;
    }

    bool PriorityController::removeTask(size_t priority_num)
    {
        if (tasks_.erase(priority_num) == 0)
        {
            // TODO: error!
            return false;
        }
        return true;
    }

    bool PriorityController::computeJointVelocityCommand(const KDL::JntArray& q_pos)
    {
        q_pos_ = q_pos;
        computeJointLimitAvoidance(joint_lim_avoidance_Wq_, joint_lim_avoidance_F_, q_pos_);
        computeSingularityAvoidance(singularity_avoidance_F_);
        computeTaskJointVelocities();
        q_vel_cmd_ = q_vel_sum_ + null_space_projector_ * (robot_->identity_matrix() - joint_lim_avoidance_Wq_) * joint_lim_avoidance_F_; 
        q_vel_cmd_ = q_vel_cmd_ + null_space_projector_ * singularity_avoidance_F_;
        auto task = tasks_.rbegin();
        size_t jointAtLimit = atJointVelLimit(q_vel_cmd_);
        while (jointAtLimit < robot_->nj())
        {
            q_vel_cmd_ =  robot_->joint_vel_limit()[jointAtLimit] * q_vel_cmd_ / q_vel_cmd_(jointAtLimit);
            jointAtLimit = atJointVelLimit(q_vel_cmd_);
        }
        while (!validJointVel(q_vel_cmd_) && task != tasks_.rend())
        {
            q_vel_cmd_ = q_vel_cmd_ - task->second->get_q_vel();
            ++task;
        }
        if (!validJointVel(q_vel_cmd_))
        {
            q_vel_cmd_ = Eigen::VectorXd::Zero(robot_->nj());
            // TODO: error
            return false;
        }
        return true;
    }

    Eigen::VectorXd const& PriorityController::getJointVelocityCommand()
    {
        return q_vel_cmd_;
    }

    bool PriorityController::validJointVel(const Eigen::VectorXd& q_vel)
    {
        KDL::JntArray q_pos_next = KDL::JntArray(robot_->nj());
        for (size_t i = 0; i < robot_->nj(); ++i)
        {
            q_pos_next(i) = q_pos_(i) + q_vel(i)*dt_;
            if (q_pos_next(i) > robot_->upper_joint_limit()[i] || q_pos_next(i) < robot_->lower_joint_limit()[i])
            {
                ROS_WARN("At Hard Joint Position Limit for Joint: %ld", i+1);
                return false;
            }
        }

        KDL::Jacobian jac_next = KDL::Jacobian(robot_->nj());
        if (robot_->jac_solver_->JntToJac(q_pos_next, jac_next) < 0)
        {
            ROS_ERROR("Could not compute jacobian!");
            // TODO: Stop Controller or Send Message to Watchdog
            return false;
        }

        robot_->svd_fast_solver_->compute(jac_next.data);
        int basic_rank = robot_->svd_fast_solver_->rank();
        Eigen::MatrixXd Wq_temp = Eigen::MatrixXd::Identity(robot_->nj(), robot_->nj());
        Eigen::VectorXd F_temp = Eigen::VectorXd::Zero(robot_->nj());
        computeJointLimitAvoidance(Wq_temp, F_temp, q_pos_next);
        robot_->svd_fast_solver_->compute(jac_next.data*Wq_temp);
        int joint_lim_rank = robot_->svd_fast_solver_->rank();
        if ((basic_rank >= 6 && joint_lim_rank < 6) || joint_lim_rank < basic_rank)
        {
            return false;
        }
        return true;
    }

    size_t PriorityController::atJointVelLimit(const Eigen::VectorXd& q_vel)
    {
        KDL::JntArray q_pos_next = KDL::JntArray(robot_->nj());
        for (size_t i = 0; i < robot_->nj(); ++i)
        {
            if (abs(q_vel(i)) > robot_->joint_vel_limit()[i])
            {
                ROS_WARN("At Hard Joint Velocity Limit for Joint: %ld", i+1);
                return i;
            }
        }
        
        return robot_->nj();
    }

    void PriorityController::computeJointLimitAvoidance(Eigen::MatrixXd& Wq, Eigen::VectorXd& F, const KDL::JntArray& q_pos)
    {
        for (size_t i = 0; i < robot_->nj(); ++i)
        {
            if (q_pos(i) >= robot_->lower_joint_limit()[i] && q_pos(i) < robot_->lower_damping_threshold()[i])
            {
                Wq(i,i) = jointLimitDampingFunction((robot_->lower_damping_threshold()[i] - q_pos(i)) / (robot_->lower_damping_threshold()[i] - robot_->lower_joint_limit()[i]));
                F(i) = robot_->joint_limit_force_max()[i] * (robot_->lower_damping_threshold()[i] - q_pos(i)) / (robot_->lower_damping_threshold()[i] - robot_->lower_joint_limit()[i]);
            }
            else if (q_pos(i) > robot_->upper_damping_threshold()[i] && q_pos(i) <= robot_->upper_joint_limit()[i])
            {
                Wq(i,i) = jointLimitDampingFunction((q_pos(i) - robot_->upper_damping_threshold()[i]) / (robot_->upper_joint_limit()[i] - robot_->upper_damping_threshold()[i]));
                F(i) = robot_->joint_limit_force_max()[i] * (robot_->upper_damping_threshold()[i] - q_pos(i)) / (robot_->upper_joint_limit()[i] - robot_->upper_damping_threshold()[i]);
            }
            else if (q_pos(i) >= robot_->lower_damping_threshold()[i] && q_pos(i) <= robot_->upper_damping_threshold()[i])
            {
                Wq(i,i) = 1;
                F(i) = 0;
            }
            else
            {
                Wq(i,i) = 0;
                F(i) = 0;
            }
        }
    }

    double PriorityController::jointLimitDampingFunction(double x)
    {
        return 0.5 - 0.5*tanh((1 / (1-x)) - (1/x));
    }

    void PriorityController::computeSingularityAvoidance(Eigen::VectorXd& F)
    {
        manipulabilityGradient(q_pos_, F);
        F = robot_->DEFAULT_SINGULARITY_FORCE_COEFFICIENT * F;
    }

    bool PriorityController::manipulabilityGradient(const KDL::JntArray& q_pos, Eigen::VectorXd& g)
    {
        KDL::JntArray dq = KDL::JntArray(robot_->nj());
        KDL::JntArray sum = KDL::JntArray(robot_->nj());
        for (size_t i = 0; i < robot_->nj(); ++i)
        {
            dq(i) = 0.001 * dt_;
            KDL::Add(q_pos, dq, sum);
            double sum_manipulability = manipulability(sum);
            double current_manipulability = manipulability(q_pos);
            if (sum_manipulability < 0 || current_manipulability < 0)
            {
                // TODO: error!
                return false;
            }
            g(i) = (sum_manipulability - current_manipulability) / dq(i);
            dq(i) = 0;
        }
        g = g / g.norm();
        return true;
    }

    double PriorityController::manipulability(const KDL::JntArray& q_pos)
    {
        KDL::Jacobian jac = KDL::Jacobian(robot_->nj());
        if (robot_->jac_solver_->JntToJac(q_pos, jac) < 0)
        {
            ROS_ERROR("Could not compute jacobian!");
            // TODO: Stop Controller or Send Message to Watchdog
            return -1;
        }
        robot_->svd_fast_solver_->compute(jac.data);
        return robot_->svd_fast_solver_->singularValues()(0) / robot_->svd_fast_solver_->singularValues()(robot_->CARTESIAN_DOF - 1);
    }

    void PriorityController::computeTaskJointVelocities()
    {
        null_space_projector_ = Eigen::MatrixXd::Identity(robot_->nj(), robot_->nj());
        q_vel_sum_ = Eigen::VectorXd::Zero(robot_->nj());
        for (auto& task : tasks_)
        {
            task.second->compute_kinematic_matrices(q_pos_, joint_lim_avoidance_Wq_, null_space_projector_);
            task.second->set_q_vel(
                task.second->pseudoinverse_jacobian() * 
                (task.second->task_twist() - task.second->task_jacobian()*q_vel_sum_));
            null_space_projector_ = null_space_projector_ * 
                (robot_->identity_matrix() - task.second->pseudoinverse_jacobian() * task.second->task_jacobian());
            q_vel_sum_ = q_vel_sum_ + task.second->get_q_vel();
        }
    }

}