#pragma once

#include <map>
#include <eigen3/Eigen/Dense>
#include "task.hpp"
#include "arthur_robot_model.hpp"
#include "kdl/jntarray.hpp"

namespace priority_control
{
    class PriorityController
    {
        public:
        PriorityController(std::shared_ptr<ArthurRobotModel> robot, double dt);
        bool addTask(std::shared_ptr<Task> task, size_t priority_num);
        bool removeTask(size_t priority_num);
        bool computeJointVelocityCommand(const KDL::JntArray& q_pos);
        Eigen::VectorXd const& getJointVelocityCommand();
        double manipulability(const KDL::JntArray& q_pos);
        bool hit_joint_limit();
        void reset_fault();

        protected:
        std::shared_ptr<ArthurRobotModel> robot_;
        std::map<size_t, std::shared_ptr<Task>> tasks_;
        Eigen::VectorXd q_vel_cmd_;
        Eigen::VectorXd q_vel_sum_;
        double dt_;
        KDL::JntArray q_pos_;
        Eigen::MatrixXd null_space_projector_;
        bool joint_lim_avoidance_active_;
        bool hit_joint_lim_;
        Eigen::MatrixXd joint_lim_avoidance_Wq_;
        Eigen::VectorXd joint_lim_avoidance_F_;
        Eigen::VectorXd singularity_avoidance_F_;
        bool fault_;

        bool validJointVel(const Eigen::VectorXd& q_vel);
        size_t atJointVelLimit(const Eigen::VectorXd& q_vel);
        void computeJointLimitAvoidance(Eigen::MatrixXd& Wq, Eigen::VectorXd& F, const KDL::JntArray& q_pos);
        double jointLimitDampingFunction(double x);
        void computeSingularityAvoidance(Eigen::VectorXd& F);
        void computeTaskJointVelocities();
        bool manipulabilityGradient(const KDL::JntArray& q_pos, Eigen::VectorXd& g);
    };
}