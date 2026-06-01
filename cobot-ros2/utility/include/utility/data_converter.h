/**
 * @file data_converter.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Convert from the data format used by the hardware interface to the format
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

 #ifndef DATA_CONVERTER_H_
 #define DATA_CONVERTER_H_
 
 #include "data_processor.h"
 namespace addverb_cobot
 {
     class DataConverter : public DataProcessor, public std::enable_shared_from_this<DataConverter>
     {
     public:
         explicit DataConverter(const rclcpp::Logger &log) : logger_(log) {};
 
         ~DataConverter() {};
 
         const std::string getName() override
         {
             return "DataConverter";
         }
 
         /// @brief execute the given request on the given data
         /// @param request : one/more of validation, conversion and communication
         /// @param container : the data on which the operations are to be performed
         /// @return 0 - no error, any other number is error code
         error_codes handleForward(const DataProcessorRequest &request, DataContainer &container) override;
 
         /// @brief execute the given request on the given structure in reverse order
         /// @param request : one/more of validation, conversion and communication
         /// @param container : the data on which the operations are to be performed
         /// @return 0 - no error, any other number is error code
         error_codes handleBackward(const DataProcessorRequest &request, DataContainer &container) override;
 
         /// @brief allow returning shared pointer to this
         /// @return
         std::shared_ptr<DataProcessor> shared_from_this() override;
 
     private:
         rclcpp::Logger logger_;
 
         /// @brief convert to the safety mode
         error_codes convert(const hw_interface_defs::SafetyMode &, DataContainer &);
 
         /// @brief convert to gripper config
         error_codes convert(const hw_interface_defs::GripperConfig &, DataContainer &);
 
         /// @brief convert to ft config
         error_codes convert(const hw_interface_defs::FTConfig &, DataContainer &);
 
         /// @brief convert to payload
         error_codes convert(const hw_interface_defs::Payload &, DataContainer &);
 
         /// @brief convert to velocity
         error_codes convert(const hw_interface_defs::Velocity &, DataContainer &);
 
         /// @brief convert to effort 
         error_codes convert(const hw_interface_defs::Effort &, DataContainer &);
 
         /// @brief convert to controller
         error_codes convert(const hw_interface_defs::ControlMode &, DataContainer &);
 
         /// @brief convert to single point controller
         error_codes convert(const hw_interface_defs::Point &, DataContainer &);
 
         /// @brief convert to robot feedback
         error_codes convert(const RobotFeedback &, DataContainer &);
 
         /// @brief convert to robot state
         error_codes convert(const RobotState &, DataContainer &);
 
         /// @brief convert to multi point controller format
         error_codes convert(const hw_interface_defs::MultiPoint &, DataContainer &);
 
         /// @brief convert to tcp multi point controller format
         error_codes convert(const hw_interface_defs::TcpMultipoint &, DataContainer &);
 
         /// @brief convert to replay config
         error_codes convert(const hw_interface_defs::ReplayConfig &, DataContainer &);
 
         /// @brief convert to flex point config
         error_codes convert(const hw_interface_defs::FlexPoint &, DataContainer &);
 
         /// @brief convert to joint jogging commands
         error_codes convert(const hw_interface_defs::JointJog &joint_jog, DataContainer &container);
 
         /// @brief convert to cartesian jogging commands
         error_codes convert(const hw_interface_defs::CartesianJog &cartesian_jog, DataContainer &container);
 
         /// @brief convert to joint impedance format
         error_codes convert(const hw_interface_defs::JointImpedance &, DataContainer &);
 
         /// @brief convert to cartesian impedance format
         error_codes convert(const hw_interface_defs::CartesianImpedance &, DataContainer &);

         /// @brief convert gripper commands
         error_codes convert(const hw_interface_defs::GripperCmd &, DataContainer &);
 
         // /// @brief convert to ptp
         // int convert(const hw_interface_defs::PtP &, DataContainer &);
 
         // /// @brief convert to multi pt
         // int convert(const hw_interface_defs::MultiPt &, DataContainer &);
     };
 
 };
 
 #endif
 