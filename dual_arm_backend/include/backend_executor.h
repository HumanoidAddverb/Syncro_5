/**
 * @file backend_executor.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Executor for carrying out tasks for the HTTPServer
 * including
 * - data feed and retrieval
 * - handling connection with the robot
 *
 * @version 0.1
 * @date 2024-06-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef BACKEND_EXECUTOR_H_
#define BACKEND_EXECUTOR_H_

#define convertToRad(degree) (((double)(degree) / 180) * 3.14)
#define convertToDeg(radian) (((double)(radian) / 3.14) * 180)

#define CX 0
#define CY 0
#define R_INNER 0.30
#define R_OUTER 0.45
#define TUBE_LL 0.46
#define TUBE_UL 0.58
#define CENTRE_X 0.00
#define CENTRE_Y 0.00
#define TUBE_TH 0.04

#define ROBOT_IDX 0

#include <memory>
#include <algorithm>
#include <map>
#include <cmath>
#include "data_store.h"
#include "data_handler.h"
#include "jogging_planes.h"

#define RECORDING_FREQUENCY 20

/// @brief holds the data needed for longer run for the backend
struct DataHolder
{
    /// @brief mode of recording
    RecordingMode mode;

    /// @brief label in which recording has to be saved
    std::string script_label;

    /// @brief number of times the recording is to be played
    int iterations;

    /// @brief api selected
    UIVisAPI api;

    /// @brief default ctor
    DataHolder()
    {
        reset();
    }

    /// @brief reset the structure to default values
    void reset()
    {
        mode = RecordingMode::eNone;
        script_label = std::string();
        iterations = 0;
        api = UIVisAPI::eNone;
    }
};

/**
 * @brief format of converted joint position
 *
 */
enum class ConvertedFormat
{
    eRadians,
    eDegrees
};

class BackendExecutor
{
public:
    /// @brief ctor
    BackendExecutor() : handler_(ROBOT_IDX)
    {
        expect_connection_ = false;
        fetch_task_status_ = false;
    };

    /// @brief dtor
    ~BackendExecutor() {};

    /// @brief setup the executor
    /// @return
    bool setup(std::shared_ptr<ErrorManagerInterface> &);

    /// @brief task to be executed
    bool execute(const ExecTaskStruct &, ExecDataStruct &);

    /// @brief terminate the system
    /// @return
    bool terminate();

private:
    /// @brief falg to indicate if connection should be expected with the robot
    bool expect_connection_;

    /// @brief when there is long running task
    bool fetch_task_status_;

    /// @brief adapter to convert from std::string to structure and vice-versa
    DataAdapter adapter_;

    /// @brief handles interaction with the remote
    DataHandler handler_;

    /// @brief data store
    DataStore store_;

    /// @brief response for the client
    ResponseJSON response_;

    /// @brief robot state
    RobotState robot_state_;

    /// @brief error manager
    std::shared_ptr<ErrorManagerInterface> error_man_;

    /// @brief requested action from the user
    UIAction req_action_;

    /// @brief controller map
    std::map<UIVisAPI, API> controller_map_;

    /// @brief data to be retained across multiple requests to the backend
    DataHolder data_holder_;

    /// @brief recording controllers
    std::vector<UIVisAPI> recording_ctrl_ = std::vector<UIVisAPI>{UIVisAPI::eRecJogAxis, UIVisAPI::eRecJogRPY, UIVisAPI::eRecJogJoint, UIVisAPI::eRecFlexJogAxis, UIVisAPI::eRecFlexJogRPY, UIVisAPI::eRecFlexJogJoint, UIVisAPI::eRecFreeDrive, UIVisAPI::eRecMultiPoint};

    /// @brief jogging controllers in recording mode
    std::vector<UIVisAPI> jogging_ctrl_ = std::vector<UIVisAPI>{UIVisAPI::eRecJogAxis, UIVisAPI::eRecJogRPY, UIVisAPI::eRecJogJoint, UIVisAPI::eRecFlexJogAxis, UIVisAPI::eRecFlexJogRPY, UIVisAPI::eRecFlexJogJoint};

    /// @brief handle the request from the client
    /// @param
    /// @return
    bool handleRequest_(const std::string &);

    /// @brief write to the robot
    /// @return
    bool handleWrite_();

    /// @brief parse the request stream
    /// @param
    /// @return
    bool parseStream_(const std::string &, RequestJSON &);

    /// @brief parse the request structure to decide what happens on requested action
    /// @param
    /// @return
    bool executeAction_(const RequestJSON &);

    /// @brief handle the request to connect with the robot
    /// @return
    bool handleConnectReq_();

    /// @brief set the control input if recd explicitly from the user
    /// @return
    bool setControlInput_(const RequestPayload &);

    /// @brief set the response to be sent
    /// @param
    /// @return
    bool setResponse_(ExecDataStruct &);

    /// @brief set the configuration for the robot
    /// @return
    bool setConfig_(const RequestPayload &);

    /// @brief clear the response field
    void clearResponse_();

    /// @brief start the robot
    /// @return
    bool startRobot_();

    /// @brief clear the error
    /// @return
    bool clearError_();

    /// @brief record point
    /// @return
    bool recordPoint_(const RequestPayload &);

    /// @brief update the response to be sent to the client
    /// @return
    bool updateResponse_();

    /// @brief update the response from the robot end
    /// @return
    void updateRobotResponse_();

    /// @brief perform additional computations on the feedback recd from the robot
    void performAddComp_();

    /// @brief reset configuration for running the robot
    /// @return
    bool resetConfig_();

    /// @brief stop the robot - in case of emergency
    /// @return
    bool stopRobot_();

    /// @brief setup the map between controller from UI and controller on robot end
    void setupControllerMap_();

    /// @brief update the status of the task
    void updateTaskStatus_(const RobotState &);

    /// @brief update robot feedback in the response data
    void updateRobotFeedback_();

    /// @brief stop the robot abruptly
    /// @return
    bool abruptStopRobot_();

    /// @brief get the API from the data provided by the UI
    /// @param
    /// @param
    /// @return
    bool getAPI_(const ControllerConfig &, int &);

    /// @brief start recording in the label provided during config setup
    /// @return
    bool startRecording_();

    /// @brief stop recording
    /// @return
    bool stopRecording_();

    /// @brief additional configuration tasks
    /// @return
    bool addConfigTasks_(ControllerConfig &);

    /// @brief handle recording of muti point data entries
    /// @param
    /// @return
    bool recMultiPoint_(const ControllerConfig &);

    /// @brief check if the controller config provided is correct
    /// @param
    /// @return
    bool isValidControllerConfig_(const RequestPayload &);

    /// @brief validate the config recd based on the controller recd
    /// @param
    /// @return
    bool isValidConfig_(const ControllerConfig &);

    /// @brief validate the recorded data set
    /// @param
    /// @return
    bool isValidConfig_(const RecordedDataSet &);

    /// @brief validity checks for Linear velocity controller
    /// @param
    /// @param
    /// @return
    bool isValidConfig_(const std::vector<double> &, const double &);

    /// @brief validate multi point control checks
    /// @param
    /// @param
    /// @return
    bool isValidConfig_(const std::vector<std::vector<double>> &, const std::vector<double> &);

    /// @brief validate flex linear velocity checks
    /// @param
    /// @param
    /// @param
    /// @return
    bool isValidConfig_(const std::vector<double> &, const double &, const double &);

    /// @brief validate play controller
    /// @param
    /// @return
    bool isValidConfig_(const ScriptConfig &);

    /// @brief save the configuration for use within backend executor till the life of the controller
    /// @param
    /// @return false if the config is invalid
    bool saveConfig_(const RequestPayload &);

    /// @brief save the script config
    /// @param
    /// @return
    bool saveConfig_(const ScriptConfig &);

    /// @brief common validity checks for all controllers that supoort recording feature
    /// @param
    /// @return
    bool isValidRecConfig_(const ScriptConfig &, const bool & = true);

    /// @brief record point in continous mode
    /// @return
    bool recordPoint_();

    /// @brief record point in intermediate mdoe
    /// @param
    /// @return
    bool recordPoint_(const AdditionalInput &);

    /// @brief additional control
    /// @param
    /// @return
    bool addControl_(const RequestPayload &);

    /// @brief convert to the format req by robot
    /// @param
    void convert_(std::vector<double> &, const ConvertedFormat & = ConvertedFormat::eRadians);

    /// @brief convert to the format req by robot
    void convert_(std::vector<std::vector<double>> &);

    /// @brief convert to the format req by robot
    void convert_(RecordedDataSet &);

    /// @brief convert the feedback position from radians to degrees
    /// @param
    void convert_(RobotFeedback &);

    /// @brief convert target joint position to radians as required by the robot
    /// @param
    void convert_(ControllerConfig &);

    /// @brief update the payload values to avoid truncation errors
    /// @param
    void modifyPayload_(PayloadConfig &);

    /// @brief add epsilon multiplied by the multiplier
    void addEps_(double &, const int & = 1000000);
};

#endif