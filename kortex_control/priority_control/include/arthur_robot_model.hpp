#pragma once

#include <urdf_model/model.h>
#include <urdf/model.h>
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <ros/console.h>

namespace priority_control 
{
    class ArthurRobotModel
    {
        public:
        static constexpr double DEFAULT_JOINT_LIMIT_AVOIDANCE_BANDWIDTH = 0.1;
        static constexpr size_t CARTESIAN_DOF = 6;
        static constexpr double DEFAULT_LAMBDA = 0.2;
        static constexpr double DEFAULT_EPSILON = 0.05;
        static constexpr double DEFAULT_JOINT_LIMIT_FORCE_COEFFICIENT = 3;

        std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        std::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
        std::shared_ptr<Eigen::JacobiSVD<Eigen::MatrixXd>> svd_full_solver_;
        std::shared_ptr<Eigen::JacobiSVD<Eigen::MatrixXd>> svd_fast_solver_;

        ArthurRobotModel(const std::string& robot_description, const std::string& base_frame, const std::string& tip_frame);
        std::vector<double> const& upper_joint_limit();
        std::vector<double> const& lower_joint_limit();
        std::vector<double> const& upper_damping_threshold();
        std::vector<double> const& lower_damping_threshold();
        std::vector<double> const& joint_rom();
        std::vector<double> const& joint_vel_limit();
        std::vector<double> const& joint_limit_force_max();
        size_t nj();
        Eigen::MatrixXd& identity_matrix();

        private:
        urdf::Model urdf_model_;
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;
        std::string robot_description_;
        size_t number_joints_;
        double joint_limit_avoidance_bandwidth_;
        std::vector<double> upper_joint_limit_;
        std::vector<double> lower_joint_limit_;
        std::vector<double> upper_damping_threshold_;
        std::vector<double> lower_damping_threshold_;
        std::vector<double> joint_rom_;
        std::vector<double> joint_vel_limit_;
        std::vector<double> joint_limit_force_max_;
        Eigen::MatrixXd I_;

        bool compute_joint_limits(const std::string& base_frame, const std::string& tip_frame);
        void compute_joint_limit_avoidance_thresholds();
    };
}
