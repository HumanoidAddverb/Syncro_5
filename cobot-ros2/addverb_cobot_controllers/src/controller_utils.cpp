#include "addverb_cobot_controllers/controller_utils.h"

namespace controller_utils
{
    TrajectoryTransferUtil::TrajectoryTransferUtil(
        std::vector<hardware_interface::LoanedCommandInterface> &cmds,
        std::vector<hardware_interface::LoanedStateInterface> &states)
        : command_interfaces_(cmds), state_interfaces_(states)
    {
    }
    
    void TrajectoryTransferUtil::setTrajectory(const trajectory &traj)
    {
        trajectory_ = traj;
    }
    
    void TrajectoryTransferUtil::setTrajectory(
        const std::vector<std::string> &joint_names,
        const std::vector<std::vector<double>> &trajectory_points,
        const std::vector<double> &time_seq)
    {
        trajectory_ = convertToTrajectory(joint_names, trajectory_points, time_seq);
    }
    
    TrajectoryTransferUtil::trajectory TrajectoryTransferUtil::convertToTrajectory(const std::vector<std::string> &joint_names,
                                                const std::vector<std::vector<double>> &trajectory_points,
                                                const std::vector<double> &time_seq)
    {
        trajectory traj_msg;
        traj_msg.joint_names = joint_names;
    
        for (size_t i = 0; i < trajectory_points.size(); ++i)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = trajectory_points[i];
            point.time_from_start = rclcpp::Duration::from_seconds(time_seq[i]);
            traj_msg.points.push_back(point);
        }
    
        return traj_msg;
    }
    
    void TrajectoryTransferUtil::sendTrajectory()
    {
        if (static_cast<size_t>(cur_index_) < trajectory_.points.size())
        {
            const auto &point = trajectory_.points[cur_index_];
    
            for (size_t i = 0; i < point.positions.size(); ++i)
            {
                command_interfaces_[i].set_value(point.positions[i]);
            }
    
            command_interfaces_[6].set_value(point.time_from_start.sec +
                                             point.time_from_start.nanosec / 1e9);
    
            command_interfaces_[7].set_value(
                static_cast<int>(addverb_cobot::TransferCommand::eTransferring));
    
            cur_index_++;
        }
        else
        {
            command_interfaces_[7].set_value(
                static_cast<int>(addverb_cobot::TransferCommand::ePublish));
            cur_index_ = 0;
        }
    }
    
    void TrajectoryTransferUtil::updateControllerStatus()
    {
        command_interfaces_[7].set_value(
            static_cast<int>(addverb_cobot::TransferCommand::eRcdNewTraj));
    }
    
    void TrajectoryTransferUtil::execute()
    {
        command_interfaces_[7].set_value(
            static_cast<int>(addverb_cobot::TransferCommand::eExecute));
    }
    
    //// KDL UTILS 
    /** @brief Set joint velocity
     *   @param 
     *   @return true
     *  @return false
    */
    bool KdlUtil::init()
    {
        /// create tree from urdf 
        if (!kdl_parser::treeFromFile(urdf_path_, kdl_tree_)){
            RCLCPP_ERROR(logger_,
            "KDL: file not found ...");

            return false;
        }
            
        RCLCPP_INFO(logger_, "KDL tree created.");
        
        /// create KDL chain 
        if (!kdl_tree_.getChain(base_link_, tip_link_, kdl_chain_))
        { 
            RCLCPP_INFO(logger_,
            "Failed to create KDL chain.");
            
            return false;
        }

        RCLCPP_INFO(logger_, "KDL chain created.");
        
        // number of joint present in KDL chain 
        num_joints_ = kdl_chain_.getNrOfJoints();
        

        // check if number of joints matches
        if (num_joints_ != addverb_cobot::n_dof)
        {
            RCLCPP_ERROR(logger_,
            "Mismatch in number of joints in KDL chain and the actual number of joints");

            return false;
        }
        
        /// initialise kdl variables
        initKDLVar_();
        
        /// initialise kdl solvers
        initDynSolver_();

        RCLCPP_INFO(logger_, "KDL initialised .... ");
    
        return true;

    }
    
    /** @brief Set joint velocity
     * @param jpos
     * @return true
     * @return false
    */ 
    bool KdlUtil::setJpos(std::vector<double>& jpos)
    {
        if (jpos.size() != addverb_cobot::n_dof)
        {
            return false;
        } 

        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            q_(i) = jpos[i];
        }

        return true;
    }

    /** @brief Set joint velocity
    *   @param jvel
    *   @return true
    *   @return false
    */
    bool KdlUtil::setJvel(std::vector<double>& jvel)
    {
        if (jvel.size()!= addverb_cobot::n_dof)
        {
            return false;
        }

        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            q_dot_(i) = jvel[i];
        }

        return true;
    }
    
    /** @brief Compute gravity compenstation torque
     * @param jpos 
     * @return std::vector<double>
     */ 
    void KdlUtil::computeGravityComp(std::vector<double>& effort)
    {
        dyn_solver_->JntToGravity(q_, gravity_torque_);

        effort.resize(addverb_cobot::n_dof, 0.0);

        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            effort[i] = gravity_torque_(i);
        }
    }

    /** @brief Compute gravity coriolis torque 
     * @param effort 
     * @return
     */
    void KdlUtil::computeCoriolisTorque(std::vector<double>& effort)
    {
        dyn_solver_->JntToCoriolis(q_, q_dot_, coriolis_torque_);

        effort.resize(addverb_cobot::n_dof, 0.0);

        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            effort[i] = coriolis_torque_(i);
        }
    }

    /**
     * @brief Create KDL variables 
     * for  jpos, jvel, gravity torques, & coriolis_torque
     * 
     * @return
     */ 
    void KdlUtil::initKDLVar_()
    {
        q_ = KDL::JntArray(num_joints_);
        q_dot_ = KDL::JntArray(num_joints_);
        gravity_torque_ = KDL::JntArray(num_joints_);
        coriolis_torque_ = KDL::JntArray(num_joints_);
    }

    /**
     * @brief Create KDL dynamics solver  
     * for gravity torques, & coriolis_torque
     * 
     * @return
     */ 
    void KdlUtil::initDynSolver_()
    {
        dyn_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);
    }
}
