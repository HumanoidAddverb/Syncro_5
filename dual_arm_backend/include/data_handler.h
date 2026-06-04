/**
 * @file data_handler.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data Handler
 * implements a two-way adapter to exchange data with the robot
 *
 * @version 0.1
 * @date 2024-06-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DATA_HANDLER_H_
#define DATA_HANDLER_H_

#include "network_config.h"
#include "socket_client_comm_wrapper.h"
#include "network_manager.h"
#include "robot_fsm_enums.h"
#include "error_codes.h"
#include "ui_data_types.h"
#include "data_adapter.h"

/// @brief data types sent from UI to robot
enum class UIDataType
{
    eNone,
    eConfigData,
    eControlData,
    eEventData
};

class DataHandler
{
public:
    /// @brief ctor
    explicit DataHandler(const int &idx) : net_(idx) {};

    // Allow the compiler to generate the move constructor
    DataHandler(DataHandler &&) = default;
    DataHandler &operator=(DataHandler &&) = default;

    // If you don't want copies, you can delete them
    DataHandler(const DataHandler &) = delete;

    /// @brief dtor
    ~DataHandler() {};

    /// @brief setup the data handler
    /// @return
    bool setup();

    /// @brief returns true if the peer is connected
    /// @return
    bool isConnected() const;

    /// @brief connect with the peer
    /// @return
    bool connect();

    /// @brief disconnect from the peer
    /// @return
    bool disconnect();

    /// @brief clear resources in case of abrupt closing
    void clearResources();

    /// @brief shutdown the network pipeline
    /// @return
    bool shutdown();

    /// @brief read from remote
    /// @return
    bool readData();

    /// @brief write to remote
    /// @return
    bool writeData();

    /// @brief set action for robot
    /// @param
    /// @return
    void setAction(const RobotAction &);

    /// @brief append the recorded dataset to the configuration
    /// @param
    void appendConfig(const RecordedDataSet &);

    /// @brief append script config to existing config
    /// @param
    void appendConfig(const ScriptConfig &);

    /// @brief set the UserConfig
    // void setUserConfig(UserConfig);

    /// @brief set error code
    /// @param
    /// @return
    void setError(const HealErrors::Code &);

    /// set the user config
    // void setUserConfig(RequestPayload);

    /// control data as given by user at run-time
    void setUserInput(ControlInterrupt);

    /// @brief set the vent configuration
    /// @param
    void setEventConfig(const EventConfig &);

    /// @brief get the state of the robot as recd from the remote
    /// @param
    /// @return
    void getState(RobotState &);

    /// get the error as recd
    void getError(HealErrors::Code &);

    /// @brief get execution status of the user provided event
    /// @param
    void getEventStatus(EventStatus &);

    /// @brief get the feedback from the robot
    /// @param
    /// @return
    bool getRobotFeedback(RobotFeedback &);

    /// @brief clear data of given type - only use this for slow moving data
    void clearData(UIDataType);

    /// @brief append payload to the configuration that will be sent to the robot
    void appendConfig(const PayloadConfig &);

    /// @brief append controller config to the configuration that will be sent to the robot
    void appendConfig(const ControllerConfig &);

    /// @brief append the type of gripper to the configuration that will be sent to the robot
    void appendGripperType(const int &);

    /// @brief append advanced controller ocnfig to the current configuration
    void appendConfig(const AdvancedControllerConfig &);

    /// @brief update the user config in the data packet that is to be sent to the robot
    void setUserConfig();

private:
    /// @brief network manager with the communication type as given by UIComm
    NetworkManager<SocketClientCommWrapper, std::string, std::string, BACKEND_COMM_TYPE> net_;

    /// @brief configuration specified by the user - this instance is to be used when multiple requests add parts of the config, but not the whole config
    UserConfig config_;

    /// @brief send data to the robot
    SocketTx send_struct_;

    /// @brief data to be recd from the robot
    SocketRx recv_struct_;

    /// @brief data to be sent
    std::string send_data_;

    /// @brief data to receive
    std::string recv_data_;

    /// @brief data adapter
    DataAdapter adapter_;

    /// @brief recorded data set
    RecordedDataSet dataset_;

    /// @brief script config
    ScriptConfig script_config_;

    /// @brief set the initial data to be sent across
    void setInitSendData_();

    /// @brief clear the data to be sent
    void clearSendData_();

    /// @brief clear the receving data structure
    void clearRecvData_();

    /// @brief fill the stream that goes to the remote
    /// @return
    bool fillStream_();

    void print_(const std::string &msg)
    {
        std::cout << msg << std::endl;
    }
};

#endif