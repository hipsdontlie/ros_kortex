#pragma once

#include "arthur_robot_model.hpp"
#include <ros/console.h>

namespace priority_control 
{
    class Task
    {
        public:
        

        protected:
        KDL::Jacobian jacobian_ = KDL::Jacobian(robot_->nj());
        KDL::JntArray q_pos_ = KDL::JntArray(robot_->nj());
        Eigen::VectorXd q_vel_ = Eigen::VectorXd::Zero(robot_->nj());
    };
}
