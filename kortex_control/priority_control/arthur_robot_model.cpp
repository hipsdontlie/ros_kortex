#include "arthur_robot_model.hpp"

namespace priority_control
{
    constexpr size_t ArthurRobotModel::CARTESIAN_DOF;
    constexpr double ArthurRobotModel::DEFAULT_LAMBDA;
    constexpr double ArthurRobotModel::DEFAULT_EPSILON;
    constexpr double ArthurRobotModel::DEFAULT_JOINT_LIMIT_FORCE_COEFFICIENT;
    constexpr double ArthurRobotModel::DEFAULT_SINGULARITY_FORCE_COEFFICIENT;
    constexpr double ArthurRobotModel::MAX_LINEAR_VELOCITY;
    constexpr double ArthurRobotModel::MAX_ANGULAR_VELOCITY;
    constexpr double ArthurRobotModel::VEL_SCALEDOWN_RATE;

    ArthurRobotModel::ArthurRobotModel(const std::string& robot_description, const std::string& base_frame, const std::string& tip_frame)
    {
        // TODO: Add error checking for inputs
        joint_limit_avoidance_bandwidth_ = DEFAULT_JOINT_LIMIT_AVOIDANCE_BANDWIDTH;

        robot_description_ = robot_description;
        
        if (!urdf_model_.initString(robot_description_)){
            ROS_ERROR("Could not initialize tree object");
        }
        if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)){
            ROS_ERROR("Could not initialize tree object");
        }
        if (!kdl_tree_.getChain(base_frame, tip_frame, kdl_chain_))
        {
            ROS_ERROR("Could not initialize chain object");
        }
        number_joints_ = kdl_chain_.getNrOfJoints();
        upper_joint_limit_.resize(number_joints_, 0.0);
        lower_joint_limit_.resize(number_joints_, 0.0);
        upper_damping_threshold_.resize(number_joints_, 0.0);
        lower_damping_threshold_.resize(number_joints_, 0.0);
        joint_rom_.resize(number_joints_, 0.0);
        joint_vel_limit_.resize(number_joints_, 0.0);
        if(!compute_joint_limits(base_frame, tip_frame))
        {
            ROS_ERROR("Could not initialize joint limits");
        }

        joint_limit_force_max_.resize(number_joints_, 0.0);
        for (size_t i = 0; i < number_joints_; ++i)
        {
            joint_limit_force_max_[i] = DEFAULT_JOINT_LIMIT_FORCE_COEFFICIENT * joint_rom_[i];
        }
        
        jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
        fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
        svd_full_solver_ = std::make_shared<Eigen::JacobiSVD<Eigen::MatrixXd>>(CARTESIAN_DOF, number_joints_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        svd_full_solver_->setThreshold(DEFAULT_EPSILON);
        svd_fast_solver_ = std::make_shared<Eigen::JacobiSVD<Eigen::MatrixXd>>(CARTESIAN_DOF, number_joints_);
        svd_fast_solver_->setThreshold(DEFAULT_EPSILON);
        I_ = Eigen::MatrixXd::Identity(number_joints_, number_joints_);
    }

    std::vector<double> const& ArthurRobotModel::upper_joint_limit()
    {
        return upper_joint_limit_;
    }
    std::vector<double> const& ArthurRobotModel::lower_joint_limit()
    {
        return lower_joint_limit_;
    }
    std::vector<double> const& ArthurRobotModel::upper_damping_threshold()
    {
        return upper_damping_threshold_;
    }
    std::vector<double> const& ArthurRobotModel::lower_damping_threshold()
    {
        return lower_damping_threshold_;
    }
    std::vector<double> const& ArthurRobotModel::joint_rom()
    {
        return joint_rom_;
    }
    std::vector<double> const& ArthurRobotModel::joint_vel_limit()
    {
        return joint_vel_limit_;
    }
    std::vector<double> const& ArthurRobotModel::joint_limit_force_max()
    {
        return joint_limit_force_max_;
    }
    size_t ArthurRobotModel::nj()
    {
        return number_joints_;
    }
    Eigen::MatrixXd& ArthurRobotModel::identity_matrix()
    {
        return I_;
    }

    bool ArthurRobotModel::compute_joint_limits(const std::string& base_frame, const std::string& tip_frame)
    {
        // get joint maxs and mins
        std::shared_ptr<const urdf::Link> link = urdf_model_.getLink(tip_frame);
        std::shared_ptr<const urdf::Joint> joint;
        int i = number_joints_ - 1;
        while (link && link->name != base_frame) 
        {
            // std::cout << link->name << std::endl;
            if (i < 0)
            {
                ROS_ERROR("compute_joint_limits::Joint mismatch!");
                return false;
            }
            joint = urdf_model_.getJoint(link->parent_joint->name);
            if (!joint) 
            {
                ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
                return false;
            }
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
            {
                if ( joint->type != urdf::Joint::CONTINUOUS ) 
                {
                    lower_joint_limit_[i] = joint->limits->lower;
                    upper_joint_limit_[i] = joint->limits->upper;
                } 
                else 
                {
                    lower_joint_limit_[i] = -3*M_PI;
                    upper_joint_limit_[i] = 3*M_PI;
                }
                joint_rom_[i] = upper_joint_limit_[i] - lower_joint_limit_[i];
                joint_vel_limit_[i] = joint->limits->velocity;
                --i;
            }
            link = urdf_model_.getLink(link->getParent()->name);
        }
        compute_joint_limit_avoidance_thresholds();
        return true;
    }

    void ArthurRobotModel::compute_joint_limit_avoidance_thresholds()
    {
        for (size_t i = 0; i < number_joints_; ++i)
        {
            lower_damping_threshold_[i] = (1.0 - joint_limit_avoidance_bandwidth_) * lower_joint_limit_[i] + joint_limit_avoidance_bandwidth_ * upper_joint_limit_[i];
            upper_damping_threshold_[i] = (1.0 - joint_limit_avoidance_bandwidth_) * upper_joint_limit_[i] + joint_limit_avoidance_bandwidth_ * lower_joint_limit_[i];
        }
    }

    
}