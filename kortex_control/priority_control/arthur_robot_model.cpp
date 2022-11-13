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

    ArthurRobotModel::ArthurRobotModel(const std::string& robot_description, const std::string& base_frame, const std::string& tip_frame, std::shared_ptr<planning_scene::PlanningScene>& planning_scene)
    {
        // TODO: Add error checking for inputs
        joint_limit_avoidance_bandwidth_ = DEFAULT_JOINT_LIMIT_AVOIDANCE_BANDWIDTH;

        robot_description_ = robot_description;
        planning_scene_ = planning_scene;
        
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

    std::string const& ArthurRobotModel::segnr2name(unsigned int i)
    {
        return kdl_chain_.getSegment(i).getName();
    }

    unsigned int ArthurRobotModel::name2segnr(const std::string& seg_name)
    {
        for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
        {
            if (kdl_chain_.getSegment(i).getName() == seg_name)
                return i;
        }
        return kdl_chain_.getNrOfSegments();
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
                    joint_rom_[i] = upper_joint_limit_[i] - lower_joint_limit_[i];
                } 
                else 
                {
                    lower_joint_limit_[i] = -std::numeric_limits<double>::infinity();
                    upper_joint_limit_[i] = std::numeric_limits<double>::infinity();
                    joint_rom_[i] = 2.0*M_PI;
                }
                
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

    bool ArthurRobotModel::is_colliding(KDL::JntArray q_pos)
    {
        moveit::core::RobotState current_state = planning_scene_->getCurrentState();
        collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
        std::vector<double> joint_positions;
        for (size_t i = 0; i < number_joints_; ++i)
        {
            joint_positions.push_back(q_pos(i));
        }
        current_state.setVariablePositions(joint_positions);

        // collision_detection::CollisionResult::ContactMap::const_iterator it2;
        // // for (it2 = collision_result_.contacts.begin(); it2 != collision_result_.contacts.end(); ++it2)
        // // {
        // //     // acm.setEntry(it2->first.first, it2->first.second, true);
        // //     std::cout << it2->first.first << std::endl;
        // //     std::cout << it2->first.second << std::endl;
        // //     std::cout << "-----------" << std::endl;
        // // }
        acm.setEntry("table", "base_link", true);
        acm.setEntry("base_link", "table", true);
        acm.setEntry("table", "wall", true);
        acm.setEntry("wall", "table", true);
        acm.setEntry("base_link", "shoulder_link", true);
        acm.setEntry("shoulder_link", "base_link", true);
        acm.setEntry("tool_frame", "reamer_head", true);
        acm.setEntry("reamer_head", "tool_frame", true);
        acm.setEntry("reamer_head", "tool_frame", true);
        acm.setEntry("ee_adapter_link", "reamer_head", true);
        acm.setEntry("reamer_head", "ee_adapter_link", true);
        collision_request_.contacts = true;
        collision_request_.max_contacts = 1000;

        //

        collision_result_.clear();
        planning_scene_->checkSelfCollision(collision_request_, collision_result_, current_state, acm);
        // ROS_INFO_STREAM("Test 5: Current state is " << (collision_result_.collision ? "in" : "not in") << " self collision");
        // collision_detection::CollisionResult::ContactMap::const_iterator it;
        // for (it = collision_result_.contacts.begin(); it != collision_result_.contacts.end(); ++it)
        // {
        //     ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
        // }

        // collision_result_.clear();
        // planning_scene_->checkSelfCollision(collision_request_, collision_result_, current_state, acm);
        // std::cout << current_state.getVariableCount() << std::endl;
        return collision_result_.collision;
    }
}