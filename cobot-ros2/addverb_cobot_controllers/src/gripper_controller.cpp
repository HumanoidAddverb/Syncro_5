#include "addverb_cobot_controllers/gripper_controller.h"

namespace addverb_cobot_controllers
{
    /**
     * @brief load state interfaces and command interfaces required for this controller
     *
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn GripperController::on_init()
    {
        try
        {
            data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
            commands_ = get_node()->get_parameter("commands").as_string_array();
        }
        catch (...)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Exception raised in initialising joint_trajectory_controller");
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief configure the controller - setup the subscription
     *
     */
    controller_interface::CallbackReturn GripperController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief claim the command interfaces required
     *
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration GripperController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = commands_;

        return config;
    }

    /**
     * @brief claim the state interfaces required
     *
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration GripperController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;

        return config;
    }

    /**
     * @brief update the next command for the hardware based on user request
     *
     */
    controller_interface::return_type GripperController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
       gripper_mutex_.lock();
       if (gripper_cmd_.transfer_cmd == static_cast<double>(addverb_cobot::GripperTransferCommand::eHasCommand))
       {
            command_interfaces_[0].set_value(gripper_cmd_.transfer_cmd);
            command_interfaces_[1].set_value(gripper_cmd_.position);
            command_interfaces_[2].set_value(gripper_cmd_.grasp_force);

            gripper_cmd_.transfer_cmd = static_cast<double>(addverb_cobot::GripperTransferCommand::eNone);
       }
       else
       {
            command_interfaces_[0].set_value(gripper_cmd_.transfer_cmd);
       }
       gripper_mutex_.unlock();
       
       return controller_interface::return_type::OK;
    }

    /**
     * @brief reset the buffer values, to allow non-junk data to be transmitted to the robot
     *
     */
    controller_interface::CallbackReturn GripperController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            gripper_srv_ = get_node()->create_service<GripperSrv>(
                std::string(get_node()->get_name()) + "/command",
                std::bind(&GripperController::gripperServiceCallback_, this, std::placeholders::_1, std::placeholders::_2));
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief de-allocate resources
     *
     */
    controller_interface::CallbackReturn GripperController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            // reset subscriber
            gripper_srv_.reset();
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void GripperController::gripperServiceCallback_(const GripperSrv::Request::SharedPtr req, GripperSrv::Response::SharedPtr res)
    {
        addverb_cobot::hw_interface_defs::GripperCmd validation_cmd;

        validation_cmd.position = req->position;
        validation_cmd.grasp_force = req->grasp_force;

        addverb_cobot::error_codes validation_result = data_validator_->validateRequest(validation_cmd);

        if (validation_result != addverb_cobot::error_codes::NoError)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Gripper command validation failed for gripper controller. Error code: %d",
                static_cast<int>(validation_result));
                res->success = true;
                res->message = "Invalid gripper command";

                return;
            }

        if (gripper_state_ == static_cast<addverb_cobot::GripperState>(req->position + 1))
        {
            res->success = true;
            res->message = "gripper is already in requested state";

            gripper_state_ = static_cast<addverb_cobot::GripperState>(req->position + 1);

            return;
        }

        gripper_mutex_.lock();
        /// @todo: check if gripper is already open / close
        gripper_cmd_.transfer_cmd = static_cast<double>(addverb_cobot::GripperTransferCommand::eHasCommand);
        gripper_cmd_.position = req->position;
        gripper_cmd_.grasp_force = req->grasp_force;
        gripper_mutex_.unlock();

        rclcpp::sleep_for(std::chrono::seconds(2));

        gripper_state_ = static_cast<addverb_cobot::GripperState>(req->position + 1);

        res->success = true;
        res->message = "success";
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot_controllers::GripperController, controller_interface::ControllerInterface)
