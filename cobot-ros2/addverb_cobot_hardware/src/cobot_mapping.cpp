#include "addverb_cobot_hardware/cobot_hw_interface.h"
namespace addverb_cobot
{
    void CobotHWInterface::initializeMapping()
    {
        createArmGroupMapping();
        createArmIndexMap();
        createHardwareSafetyMapping();
        createPayloadStatusMapping();
        createGripperTypeMapping();
        createFTMapping();
        createFTRotationMatrixMapping();
        createPayloadMassMapping();
        createPayloadCOGMapping();
        createPayloadInertiaMapping();
        createRobotControlDataMapping();
        createRobotStateDataMapping();
        createRobotMotionMapping();
        createRobotStateMapping();
        createGPIORegistry();
        createControlModeMapping();
        createEffortControllerMapping();
        createControlLoopMap();
        createMultiPointMap();
    }

    void CobotHWInterface::createArmGroupMapping()
    {
        
        // Store arm joint indices relative to the arm command/state vectors rather than
        // the original info_.joints indices. Gripper joints are skipped when populating
        // position_cmd_, velocity_cmd_, and effort_cmd_, so using info_.joints indices
        // would cause mismatches (e.g. arm_2 joints starting at index 7 instead of 6)
        // and lead to out-of-range access when indexing the command/state vectors.
        const size_t joints_per_robot = n_dof + 1;

        int gripper_counter = 0;
        size_t arm_idx = 0;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            const std::string& name = info_.joints[i].name;

            if ((i + 1) % joints_per_robot == 0)
            {
                gripper_groups_[extractRobotName(info_.joints[i].name)].push_back(gripper_counter);
                gripper_counter++;
                continue;
            }

            
            arm_groups_[extractRobotName(name)].push_back(arm_idx);
            arm_idx++;
        }
    }

    void CobotHWInterface::createArmIndexMap()
    {
        int i = 0;
        for(const auto& [name, _]: arm_groups_)
        {
            arm_index_[name] = i;
            arm_names_.push_back(name);
            i++;
        }

        for(const auto& [name, index]: arm_index_)
        {
            arm_index_reverse_[index] = name;
        }
    }

    void CobotHWInterface::createPayloadCOGMapping()
    {
        const std::string param_suffix = "payload_com";
        const std::array<std::string,3> directions = {"x","y","z"};
        std::vector<double> cog;

        for (const auto& [name, robot_index] : arm_index_)
        {
            cog.resize(3);

            for (size_t i = 0; i < directions.size(); ++i)
            {
                std::string temp_suffix = param_suffix + directions[i];

                const std::string key =
                    name.empty() ? temp_suffix : name + "." + temp_suffix;

                auto it = info_.hardware_parameters.find(key);
                if (it == info_.hardware_parameters.end())
                {
                    throw std::runtime_error("Missing hardware parameter: " + key);
                }

                cog[i] = std::stod(it->second);
            }

            payload_cog_map_[robot_index] = cog;
            cog.clear();
        }
    }
    
    void CobotHWInterface::createControlLoopMap()
    {
        for(const auto& [name, robot_index] : arm_index_)
        {
            control_loop_map_[robot_index] = std::bind(&CobotHWInterface::extVelocity_, this, robot_index);
        }
    }
    
    void CobotHWInterface::createMultiPointMap()
    {
        for(const auto& [name, robot_index] : arm_index_)
        {
            multi_point_map_[robot_index] = hw_interface_defs::MultiPoint();
            tcp_multi_point_map_[robot_index] = hw_interface_defs::TcpMultipoint();
        }
    }

    void CobotHWInterface::createPayloadInertiaMapping()
    {
        const std::string param_suffix = "payload";
        const std::array<std::string,6> directions = {"ixx","iyy","izz", "ixy", "ixz", "iyz"};
        std::vector<double> inertia;

        for (const auto& [name, robot_index] : arm_index_)
        {
            inertia.resize(6);

            for (size_t i = 0; i < directions.size(); ++i)
            {
                std::string temp_suffix = param_suffix + "_" + directions[i];

                const std::string key =
                    name.empty() ? temp_suffix : name + "." + temp_suffix;

                auto it = info_.hardware_parameters.find(key);
                if (it == info_.hardware_parameters.end())
                {
                    throw std::runtime_error("Missing hardware parameter: " + key);
                }

                inertia[i] = std::stod(it->second);
            }

            payload_intertia_map_[robot_index] = inertia;
            inertia.clear();
        }
    }

    void CobotHWInterface::createControlModeMapping()
    {
        for(const auto&[name, index] : arm_index_)
        {
            hw_interface_defs::ControlMode control_mode;
            control_mode.controller = API::eNone; //No controller active at the start
            ros_controller_map_[index] = control_mode;
        }
    }

    void CobotHWInterface::createHardwareSafetyMapping()
    {
        createMapping<uint16_t>("safety_type", safety_map_);
    }

    void CobotHWInterface::createPayloadStatusMapping()
    {
        createMapping<int>("payload_status", payload_status_map_);
    }

    void CobotHWInterface::createGripperTypeMapping()
    {
        createMapping<int>("gripper_type", gripper_type_map_);
    }

    void CobotHWInterface::createFTMapping()
    {
        createMapping<int>("ft_type", ft_type_map_);
    }

    void CobotHWInterface::createPayloadMassMapping()
    {
        createMapping<double>("payload_mass", payload_mass_map_);
    }

    void CobotHWInterface::createFTRotationMatrixMapping()
    {
        for (const auto& [name, robot_index] : arm_index_)
        {
            std::string param_suffix = "ft_rot_r";

            std::vector<double> temp;

            for (int i = 1; i <= 3; i++)
            {
                for (int j = 1; j <= 3; j++)
                {
                    std::string key = name.empty() ? param_suffix : name + "." + param_suffix;
                    
                    std::string entry = key + std::to_string(i) +
                                        std::to_string(j);

                    auto it = info_.hardware_parameters.find(entry);
                    if (it == info_.hardware_parameters.end())
                    {
                        throw std::runtime_error("Missing hardware parameter: " + entry);
                    }
                    temp.push_back(stod(it->second));
                }
                ft_rotation_matrix_map_[robot_index].push_back(temp);
                temp.clear();

            }
        }
    }
    

    template <typename T>
    void CobotHWInterface::createMapping(const std::string &param_suffix,
                                         std::unordered_map<int, T> &target_map)
    {
        for (const auto& [name, robot_index] : arm_index_)
        {
            const std::string key =
                name.empty() ? param_suffix : name + "." + param_suffix;

            auto it = info_.hardware_parameters.find(key);
            if (it == info_.hardware_parameters.end())
            {
                throw std::runtime_error("Missing hardware parameter: " + key);
            }
            target_map[robot_index] = convertFromString<T>(it->second);
        }
    }

    std::string CobotHWInterface::extractRobotName(const std::string& input)
    {
        //!Restricting usage of arm_1 etc. For multi robot setup this prefix has to be present
        
        // Must start with "arm_"
        if (input.rfind("arm_", 0) != 0)
        {
            return "";
        }

        // Find first underscore (after "arm")
        size_t first = input.find('_');  

        // Find second underscore (after number)
        size_t second = input.find('_', first + 1);

        // If no second underscore → invalid pattern
        if (second == std::string::npos)
        {
            return "";
        }

        // Extract "arm_X"
        return input.substr(0, second);
        
    }

    void CobotHWInterface::createRobotMotionMapping()
    {
        for(const auto&[robot_name, robot_index] : arm_index_)
        {
            hw_interface_defs::RobotMotionStruct motion_command(n_dof);
            robot_motion_command_data_map_[robot_index] = motion_command;
        }
    }


    void CobotHWInterface::createRobotStateMapping()
    {
        for(const auto&[robot_name, robot_index] : arm_index_)
        {
            hw_interface_defs::RobotMotionStruct motion_state(n_dof);
            robot_motion_state_data_map_[robot_index] = motion_state;
        }
    }

    template<typename T>
    T CobotHWInterface::convertFromString(const std::string& value)
    {
        if constexpr (std::is_same_v<T, int> || std::is_same_v<T,uint16_t>)
        {
            return std::stoi(value);
        }
        else if constexpr (std::is_same_v<T, double>) 
        {
            return std::stod(value);
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
            return value == "true" || value == "1";
        }
        else
        {
            static_assert(!sizeof(T), "Unsupported type for convertFromString");
        }
    }

    void CobotHWInterface::createEffortControllerMapping()
    {
        for(const auto& [name, index] : arm_index_)
        {
            robot_effort_controller_status_map_[index] = addverb_cobot::effortControllerStatus::eInactive;
            robot_effort_controller_event_map_[index] = addverb_cobot::effortControllerEvents::eNone;
        }
    }

     

    void CobotHWInterface::createGPIORegistry()
    {
        std::vector<std::pair<std::string, hw_interface_defs::GpioType>> sorted_keys(
            hw_interface_defs::GpioEntry::gpio_map.begin(),
            hw_interface_defs::GpioEntry::gpio_map.end()
        );

        std::sort(sorted_keys.begin(), sorted_keys.end(),  //Sort the keys to get longer keys first
          [](const auto& a, const auto& b)
          {
              return a.first.size() > b.first.size();
          });
        
        for (size_t i = 0; i < info_.gpios.size(); ++i)
        {
            const std::string &name = info_.gpios[i].name;
           
            std::string robot_name = extractRobotName(name);

            hw_interface_defs::GpioEntry entry;
            entry.robot_id = arm_index_.at(robot_name);

            
            entry.gpio_index = i;

            bool found = false;

            for (const auto& [key, type] : sorted_keys)
            {
                if (name.size() >= key.size() &&
                    name.compare(name.size() - key.size(), key.size(), key) == 0) //Check the name and key size as well as for multi arm check substring
                    
                    //Behaviour
                    // | Name              | Matched Key  |
                    // | ----------------- | ------------ |
                    // | arm1_time_cmd     | time_cmd     |
                    // | arm2_time_cmd     | time_cmd     |
                    // | arm1_tcp_time_cmd | tcp_time_cmd |
                    // | arm2_tcp_time_cmd | tcp_time_cmd |
                    // | time_cmd          | time_cmd     |
                    // | tcp_time_cmd      | tcp_time_cmd |

                {
                    entry.type = type;
                    found = true;
                    auto it = arm_index_.find(robot_name);

                    if (it == arm_index_.end()) {
                        throw std::runtime_error("Invalid robot_name: " + robot_name);
                    }

                    entry.robot_id = it->second;
                    gpio_map_[it->second].push_back(entry);
                    // gpio_map_[arm_index_[robot_name]].push_back(entry);
                    break;
                }
            }

            if (!found)
            {
                throw std::runtime_error("Wrong gpio present. Check URDF and list");
            }
                        
        }
    }

    void CobotHWInterface::createRobotControlDataMapping()
    {
        robot_control_data_map_.reserve(arm_index_.size());
        for(const auto&[name, index] : arm_index_)
        {
            hw_interface_defs::RobotControlData control_data(n_dof, n_controllers);

            robot_control_data_map_[index] = control_data;
        }
    }

    void CobotHWInterface::createRobotStateDataMapping()
    {
        for(const auto&[name, index] : arm_index_)
        {
            hw_interface_defs::RobotStateData state_data;

            robot_state_data_map_[index] = state_data;
        }
    }

    void CobotHWInterface::createGPIOStateInterfaces(std::vector<hardware_interface::StateInterface>& state_interfaces)
    {
        for(const auto&[robot_index, gpio_vector]: gpio_map_)
        {
            for(const auto& gpio: gpio_vector)
            {
                switch(gpio.type)
                {
                    case hw_interface_defs::GpioType::PtpTransfer:
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).state_interfaces[0].name,
                            &robot_state_data_map_[robot_index].ptp_state_.transfer_state
                        ));
                        break;
                    }

                    case hw_interface_defs::GpioType::PtpTcpTransfer:
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).state_interfaces[0].name,
                            &robot_state_data_map_[robot_index].tcp_ptp_state_.transfer_state
                        ));
                        break;
                    
                    case hw_interface_defs::GpioType::Tcp:
                        for (int i = 0; i < 6; i++)
                        {
                            state_interfaces.emplace_back(hardware_interface::StateInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).state_interfaces[i].name,
                                &robot_state_data_map_[robot_index].tcp_state_[i]));
                        }
                        break;

                    case hw_interface_defs::GpioType::RobotState:
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).state_interfaces[0].name,
                            &robot_state_data_map_[robot_index].robot_status_
                        ));
                        break;
                }
            }
            
        }
    }

    void CobotHWInterface::createGPIOCommandInterfaces(std::vector<hardware_interface::CommandInterface>& command_interfaces)
    {
        for(const auto&[robot_index, gpio_vector]: gpio_map_)
        {
            for(const auto& gpio: gpio_vector)
            {
                switch(gpio.type)
                {
                    case hw_interface_defs::GpioType::TimeCmd:
                    {
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[0].name,
                            &robot_control_data_map_[robot_index].ptp_cmd_.target.delta_t
                        ));
                        break;
                    }

                    case hw_interface_defs::GpioType::PtpTransfer:
                    {
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[0].name,
                            &robot_control_data_map_.at(robot_index).ptp_cmd_.transfer_cmd
                        ));
                        break;
                    }
                    case hw_interface_defs::GpioType::RecordCmd:
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[0].name,
                            &robot_control_data_map_[robot_index].replay_iterations_cmd_
                        ));
                        break;

                    case hw_interface_defs::GpioType::TcpTimeCmd:
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[0].name,
                            &robot_control_data_map_[robot_index].tcp_ptp_cmd_.target.delta_t
                        ));
                        break;
                    case hw_interface_defs::GpioType::PtpTcpTransfer:
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[0].name,
                            &robot_control_data_map_[robot_index].tcp_ptp_cmd_.transfer_cmd
                        ));
                        break;

                    case hw_interface_defs::GpioType::Tcp:
                        for(auto i = 0; i < 6; i++)
                        {
                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i].name,
                                &robot_control_data_map_[robot_index].tcp_command_[i]
                            ));
                        }
                        break;
                        
                    case hw_interface_defs::GpioType::JointJogging:
                        for(auto i = 0; i < n_dof; i++)
                        {
                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i].name,
                                &robot_control_data_map_[robot_index].joint_jogging_cmd_.jog_cmd.cmd[i]
                            ));
                        }
                        break;

                    case hw_interface_defs::GpioType::CartesianJogging:
                        for(auto i = 0; i < 6; i++)
                        {
                             command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i].name,
                                &robot_control_data_map_[robot_index].cartesian_jogging_cmd_.jog_cmd.cmd[i]
                            ));
                        }
                        break;
                    
                        case hw_interface_defs::GpioType::JointImpedance:
                            for(auto i = 0; i < robot_control_data_map_[robot_index].joint_impedance_cmd_.stiffness.stiffness.size(); i++)
                            {
                                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                    info_.gpios.at(gpio.gpio_index).name,
                                    info_.gpios.at(gpio.gpio_index).command_interfaces[i].name,
                                    &robot_control_data_map_[robot_index].joint_impedance_cmd_.stiffness.stiffness.at(i).at(i)
                                ));

                                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                    info_.gpios.at(gpio.gpio_index).name,
                                    info_.gpios.at(gpio.gpio_index).command_interfaces[i + 6].name,
                                    &robot_control_data_map_[robot_index].joint_impedance_cmd_.damping.damping.at(i).at(i)
                                ));
                            }
                            break;

                    case hw_interface_defs::GpioType::CartesianImpedance:
                        for(auto i = 0; i < robot_control_data_map_[robot_index].cartesian_impedance_cmd_.stiffness.stiffness.size(); i++)
                        {
                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i].name,
                                &robot_control_data_map_[robot_index].cartesian_impedance_cmd_.stiffness.stiffness[i][i]
                            ));

                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i + 6].name,
                                &robot_control_data_map_[robot_index].cartesian_impedance_cmd_.damping.damping[i][i]
                            ));


                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i + 12].name,
                                &robot_control_data_map_[robot_index].cartesian_impedance_cmd_.mass_matrix.mass_matrix[i][i]
                            ));

                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i + 18].name,
                                &robot_control_data_map_[robot_index].cartesian_impedance_cmd_.ft_force.force[i]
                            ));

                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i + 24].name,
                                &robot_control_data_map_[robot_index].cartesian_impedance_cmd_.target_force.force[i]
                            ));
                        
                        }
                        break;

                    case hw_interface_defs::GpioType::Gripper:
                        
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[0].name,
                            &robot_control_data_map_[robot_index].gripper_cmd_.transfer_cmd
                        ));

                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[1].name,
                            &robot_control_data_map_[robot_index].gripper_cmd_.position
                        ));

                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.gpios.at(gpio.gpio_index).name,
                            info_.gpios.at(gpio.gpio_index).command_interfaces[2].name,
                            &robot_control_data_map_[robot_index].gripper_cmd_.grasp_force
                        ));
                    
                        break;

                    case hw_interface_defs::GpioType::ControllerName:
                        for (int i = 0; i < n_controllers; i++)
                        {
                            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                                info_.gpios.at(gpio.gpio_index).name,
                                info_.gpios.at(gpio.gpio_index).command_interfaces[i].name,
                                &robot_control_data_map_[robot_index].controller_name_cmd_[i]
                            ));
                        }
                        break;

                }
            }
            
        }
    }

    void CobotHWInterface::createCommunicationMapping()
    {
        std::vector<std::string> sorted_arm_names = arm_names_;
        std::sort(sorted_arm_names.begin(), sorted_arm_names.end());

        //!Sorted so that arm_1 is always index 1
        for(int i = 0; i < sorted_arm_names.size(); i++)
        {
            std::shared_ptr<DataProcessor> data_processor_ = std::make_shared<DataConverter>(rclcpp::get_logger("CobotHWInterface"));

            data_processor_map_[arm_index_[sorted_arm_names[i]]] = data_processor_;

            std::shared_ptr<DataProcessor> communicator_  = std::make_shared<DataCommunicator>(rclcpp::get_logger("CobotHWInterface"), i); /// Assumption is left is index 0 and right is index 1

            std::cout << "Arm index for " << sorted_arm_names[i] << ": " << arm_index_[sorted_arm_names[i]] << " i: " << i << std::endl;
            data_communicator_map_[arm_index_[sorted_arm_names[i]]] = communicator_;

            gripper_map_[sorted_arm_names[i]] = i;

        }

    }
    
    std::string CobotHWInterface::getStringFromAPI(API api) 
    {
        for (const auto& [key, value] : control_mode_map_) 
        {
            if (value == api) {
                return key;
            }
        }
        return ""; 
    }
    
}