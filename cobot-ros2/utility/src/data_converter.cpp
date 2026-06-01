#include "utility/data_converter.h"

namespace addverb_cobot
{
    /**
     * @brief execute the given request on the given data
     *
     * @param request : one/more of validation, conversion and communication
     * @param container : the data on which the operations are to be performed
     * @return int : 0 - no error, any other number is error code
     */
    error_codes DataConverter::handleForward(const DataProcessorRequest &request, DataContainer &container)
    {
        error_codes ret = error_codes::DataNotPresent;

        if (request.convert != DataConverterRequest::eNone)
        {
            if (!container.validation_data.has_value())
            {
                RCLCPP_INFO(logger_, "DataConverter::DataNotPresent");

                return error_codes::DataNotPresent;
            }
        }

        switch (request.convert)
        {
        case DataConverterRequest::eNone:
            ret = error_codes::NoError;
            break;

        case DataConverterRequest::eSafety:
            if (container.validation_data->safety.has_value())
            {
                ret = convert(*(container.validation_data->safety), container);
            }
            break;

        case DataConverterRequest::eGripperConfig:
            if (container.validation_data->gripper.has_value())
            {
                ret = convert(*(container.validation_data->gripper), container);
            }
            break;

        case DataConverterRequest::eFTSensor:
            if (container.validation_data->ft.has_value())
            {
                ret = convert(*(container.validation_data->ft), container);
            }
            break;

        case DataConverterRequest::ePayload:
            if (container.validation_data->payload.has_value())
            {
                ret = convert(*(container.validation_data->payload), container);
            }
            break;

        case DataConverterRequest::eVelocity:
            // RCLCPP_INFO(logger_, "conversion in velocity");
            if (container.validation_data->velocity.has_value())
            {
                ret = convert(*(container.validation_data->velocity), container);
            }
            // else
            // {
            // RCLCPP_INFO(logger_, "empty velocity");
            // }
            break;

        case DataConverterRequest::eEffort:
            // RCLCPP_INFO(logger_, "conversion in effort");
            if (container.validation_data->effort.has_value())
            {
                ret = convert(*(container.validation_data->effort), container);
            }
            // else
            // {
            // RCLCPP_INFO(logger_, "empty effort");
            // }
            break;

        case DataConverterRequest::eJointJogging:
            if (container.validation_data->joint_jogging.has_value())
            {
                ret = convert(*(container.validation_data->joint_jogging), container);
            }
            break;

        case DataConverterRequest::eCartesianJogging:
            if (container.validation_data->cartesian_jogging.has_value())
            {
                ret = convert(*(container.validation_data->cartesian_jogging), container);
            }
            break;

        case DataConverterRequest::ePoint:
            if (container.validation_data->point.has_value())
            {
                ret = convert(*(container.validation_data->point), container);
            }
            break;

        case DataConverterRequest::eMultiPoint:
            if (container.validation_data->multi_point.has_value())
            {
                ret = convert(*(container.validation_data->multi_point), container);
            }
            else
            {
                RCLCPP_INFO(logger_, "empty multi pt");
            }
            break;

        case DataConverterRequest::eJointImpedance:
            if (container.validation_data->joint_impedance.has_value())
            {
                ret = convert(*(container.validation_data->joint_impedance), container);
            }
            break;

        case DataConverterRequest::eController:
            if (container.validation_data->controller.has_value())
            {
                ret = convert(*(container.validation_data->controller), container);
            }
            break;

        case DataConverterRequest::eReplay:
            if (container.validation_data->replay_config.has_value())
            {
                ret = convert(*(container.validation_data->replay_config), container);
            }
            else
            {
                RCLCPP_INFO(logger_, "empty replay data");
            }
            break;

        case DataConverterRequest::eTcpMultipoint:
            if (container.validation_data->tcp_multi_point.has_value())
            {
                ret = convert(*(container.validation_data->tcp_multi_point), container);
            }
            break;
            // add other conversions

        case DataConverterRequest::eFlexPoint:
            if (container.validation_data->flex_point.has_value())
            {
                ret = convert(*(container.validation_data->flex_point), container);
            }
            break;

        case DataConverterRequest::eCartesianImpedance:
            if (container.validation_data->cartesian_impedance.has_value())
            {
                ret = convert(*(container.validation_data->cartesian_impedance), container);
            }
            break;

        case DataConverterRequest::eGripperCmd:
            if (container.validation_data->gripper_cmd.has_value())
            {
                ret = convert(*(container.validation_data->gripper_cmd), container);
            }
            break;

        default:
            RCLCPP_INFO(logger_, "DataConverter::InvalidRequest");
            ret = error_codes::InvalidRequest;
            break;
        }

        if (ret == error_codes::NoError)
        {
            return DataProcessor::handleForward(request, container);
        }

        return ret;
    }

    /**
     * @brief handle backward
     *
     * @param request
     * @param container
     * @return error_codes
     */
    error_codes DataConverter::handleBackward(const DataProcessorRequest &request, DataContainer &container)
    {
        error_codes ret = error_codes::DataNotPresent;

        if (request.convert != DataConverterRequest::eNone)
        {
            if (!container.communication_data.has_value())
            {
                return error_codes::DataNotPresent;
            }
        }

        switch (request.convert)
        {
        case DataConverterRequest::eReadFeedback:
            if (container.communication_data->robot_feedback.has_value())
            {
                ret = convert(*(container.communication_data->robot_feedback), container);
            }
            break;

        case DataConverterRequest::eReadState:
            if (container.communication_data->robot_state.has_value())
            {
                ret = convert(*(container.communication_data->robot_state), container);
            }
            break;

        default:
            ret = error_codes::InvalidRequest;
            break;
        }

        if (ret == error_codes::NoError)
        {
            return DataProcessor::handleBackward(request, container);
        }

        return ret;
    }

    /**
     * @brief allow returning shared pointer to this
     *
     * @return std::shared_ptr<DataProcessorInterface>
     */
    std::shared_ptr<DataProcessor> DataConverter::shared_from_this()
    {
        return std::enable_shared_from_this<DataConverter>::shared_from_this();
    }

    /**
     * @brief convert to apt safety type
     *
     * @param safety
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::SafetyMode &safety, DataContainer &container)
    {
        DataConverterContainer conversion;
        conversion.safety_type = safety.safety_type;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt gripper config
     *
     * @param safety
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::GripperConfig &gripper, DataContainer &container)
    {
        DataConverterContainer conversion;
        GripperConfig grip;
        grip.gripper_type = gripper.gripper_type;
        conversion.gripper = grip;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt controller
     *
     * @param safety
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::ControlMode &mode, DataContainer &container)
    {
        DataConverterContainer conversion;
        conversion.controller_type = static_cast<int>(mode.controller);


        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt payload config
     *
     * @param safety
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::Payload &config, DataContainer &container)
    {
        PayloadConfig payload_config;
        payload_config.mass = config.mass;
        payload_config.com = config.com;
        payload_config.moi = config.moi;

        DataConverterContainer conversion;
        conversion.payload = payload_config;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt velocity commands
     *
     * @param velocity
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::Velocity &velocity, DataContainer &container)
    {
        ControlInterrupt interrupt;
        interrupt.ext_ctrl = velocity.cmd;

        DataConverterContainer conversion;
        conversion.interrupt = interrupt;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to api effort commands
     *
     * @param effort
     * @param container
     * @return int
     */

    error_codes DataConverter::convert(const hw_interface_defs::Effort &effort, DataContainer &container)
    {
        ControlInterrupt interrupt;
        interrupt.ext_ctrl = effort.cmd;

        DataConverterContainer conversion;
        conversion.interrupt = interrupt;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt velocity commands
     * @brief convert to joint jogging commands
     *
     * @param velocity
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::JointJog &joint_jog, DataContainer &container)
    {
        ControlInterrupt interrupt;
        interrupt.joint_jog = joint_jog.jog_cmd.cmd;

        DataConverterContainer conversion;
        conversion.interrupt = interrupt;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to cartesian jogging commands
     *
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::CartesianJog &cartesian_jog, DataContainer &container)
    {
        ControlInterrupt interrupt;
        interrupt.cart_jog = cartesian_jog.jog_cmd.cmd;

        DataConverterContainer conversion;
        conversion.interrupt = interrupt;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to joint impedance commands
     *
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::JointImpedance &joint_impedance, DataContainer &container)
    {
        ControlInterrupt interrupt;
        interrupt.stiffness_matrix = joint_impedance.stiffness.stiffness;
        interrupt.damping_matrix = joint_impedance.damping.damping;

        DataConverterContainer conversion;
        conversion.interrupt = interrupt;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to cartesian impedance commands
     *
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::CartesianImpedance &cartesian_impedance, DataContainer &container)
    {
        ControlInterrupt interrupt;
        interrupt.stiffness_matrix = cartesian_impedance.stiffness.stiffness;
        interrupt.damping_matrix = cartesian_impedance.damping.damping;
        interrupt.mass_matrix = cartesian_impedance.mass_matrix.mass_matrix;
        interrupt.ft_force = cartesian_impedance.ft_force.force;
        interrupt.target_force = cartesian_impedance.target_force.force;

        DataConverterContainer conversion;
        conversion.interrupt = interrupt;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt point commands
     *
     * @param point
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::Point &point, DataContainer &container)
    {
        ControllerConfig config;
        config.target_pos = point.jpos;
        config.target_time = point.delta_t;

        DataConverterContainer conversion;
        conversion.controller = config;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt MultiPoint commands
     *
     * @param MultiPoint
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::MultiPoint &multi_point, DataContainer &container)
    {
        ControllerConfig config;

        config.target_pos_seq.resize(multi_point.getSize());
        config.target_time_seq.resize(multi_point.getSize());

        for (int i = 0; i < multi_point.getSize(); i++)
        {
            config.target_pos_seq[i] = multi_point.getPoint(i).jpos;
            config.target_time_seq[i] = multi_point.getPoint(i).delta_t;
        }

        DataConverterContainer conversion;
        conversion.controller = config;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to apt FT config
     *
     * @param safety
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::FTConfig &config, DataContainer &container)
    {
        FTConfig ft;
        ft.ft_type = config.ft_type;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                ft.ft_rotation_matrix.push_back(config.rot[i][j]);
            }
        }

        DataConverterContainer conversion;
        conversion.ft = ft;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to advanced controller config
     *
     * @param safety
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const hw_interface_defs::ReplayConfig &config, DataContainer &container)
    {
        AdvancedControllerConfig advanced_config;
        ControllerConfig controller_config;
        ScriptConfig script_config;
        RecordedDataSet set;

        controller_config.controller = static_cast<int>(API::ePlayRecAPI);

        std::string label = "replay_trajectory";

        hw_interface_defs::MultiPoint multi_point = config.points;

        script_config.label = label;
        script_config.iterations = config.iterations;
        script_config.mode = static_cast<int>(RecordingMode::eNone);

        controller_config.script_config = script_config;

        RecordedDataEntry record_entry;

        for (int i = 0; i < multi_point.getSize(); i++)
        {
            record_entry.desired_pos = multi_point.getPoint(i).jpos;
            record_entry.desired_time = multi_point.getPoint(i).delta_t;
            set.dataset.push_back(record_entry);
        }

        advanced_config.dataset = set;

        DataConverterContainer conversion;
        conversion.advanced_config = advanced_config;
        conversion.controller = controller_config;

        // reset data of container
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to hw interface robot feedback from comm wrapper
     *
     * @param robot_feedback
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const RobotFeedback &robot_feedback, DataContainer &container)
    {
        DataConverterContainer conversion;

        hw_interface_defs::RobotFeedback hw_robot_feedback;

        hw_robot_feedback.jpos = robot_feedback.jpos;
        hw_robot_feedback.jvel = robot_feedback.jvel;
        hw_robot_feedback.jtor = robot_feedback.jtor;
        hw_robot_feedback.ft_data = robot_feedback.force_val;
        hw_robot_feedback.ee_pos_data = robot_feedback.ee_pos;

        conversion.hw_robot_feedback = hw_robot_feedback;

        container.convert_data = conversion;

        container.communication_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to api flex point cmd
     *
     * @param velocity
     * @param container
     * @return int
     */

    error_codes DataConverter::convert(const hw_interface_defs::FlexPoint &flex_point, DataContainer &container)
    {
        ControllerConfig config;
        config.target_pos = flex_point.point.jpos;
        config.target_time = flex_point.point.delta_t;
        config.flex_factor = flex_point.flex_factor;

        DataConverterContainer conversion;
        conversion.controller = config;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to hw interface robot state from comm wrapper
     *
     * @param robot_state
     * @param container
     * @return int
     */
    error_codes DataConverter::convert(const RobotState &robot_state, DataContainer &container)
    {
        DataConverterContainer conversion;

        conversion.robot_state = robot_state;

        container.convert_data = conversion;

        container.communication_data.reset();

        return error_codes::NoError;
    }

    /**
     * @brief convert to hw interface tcp multi point cmd
     *
     * @param tcp_multi_point
     * @param container
     */
    error_codes DataConverter::convert(const hw_interface_defs::TcpMultipoint &tcp_multi_point, DataContainer &container)
    {
        ControllerConfig config;

        config.target_pos_seq.resize(tcp_multi_point.getSize());
        config.target_time_seq.resize(tcp_multi_point.getSize());

        for (int i = 0; i < tcp_multi_point.getSize(); i++)
        {
            config.target_pos_seq[i] = tcp_multi_point.getPoint(i).pose;
            config.target_time_seq[i] = tcp_multi_point.getPoint(i).delta_t;
        }

        DataConverterContainer conversion;
        conversion.controller = config;
        container.convert_data = conversion;
        container.validation_data.reset();

        return error_codes::NoError;
    }

    error_codes DataConverter::convert(const hw_interface_defs::GripperCmd &gripper_cmd, DataContainer &container)
    {
        EventConfig event;

        if (static_cast<int>(gripper_cmd.position) == 0)
        {
            event.event = static_cast<int>(Event::eCloseGripper);
            event.clamp_force = gripper_cmd.grasp_force;
        }
        else
        {
            event.event = static_cast<int>(Event::eOpenGripper);
        }

        DataConverterContainer conversion;
        conversion.event_config = event;
        container.convert_data = conversion;
        container.validation_data.reset();
        
        return error_codes::NoError;
    }
}
