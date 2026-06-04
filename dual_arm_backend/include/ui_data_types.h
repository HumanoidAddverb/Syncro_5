/**
 * @file ui_data_types.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief File contains all data structures to be used for communication with the frontend and the robot
 * @version 0.1
 * @date 2024-06-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef UI_DATA_TYPES_H_
#define UI_DATA_TYPES_H_

#include <vector>
#include <string>
#include "robot_config.h"
#include "struct_mapping.h"
#include "api_types.h"
#include "error_codes.h"
#include "event_types.h"

/****************************  Types (Enumerations) ********************************/

/// @brief action from the UI
enum class UIAction
{
    eNone = 0,            // no action
    eConnectToRobot = 1,  // connect button
    eSetConfig = 2,       // the submit button of the screen wherein payload and controller details have been added
    eSetControlInput = 3, // action to add jogging inputs
    eStartRobot = 4,      // start the robot button
    eStopRobot = 5,       // stop the robot button
    eResetConfig = 6,     // reset the controller config button
    eStartRecording = 7,  // start recording button
    eStopRecording = 8,   // stop recording button
    eRecordPoint = 9,     // record current point
    eFetchStatus = 10,    // fetch status of the hardware
    eClearError = 11,     // clear the error
    eFetchConfig = 12,    // fetch the configuration from the backend/robot for display decisions on the UI
    eMaxRegime = 13
};

/// @brief status of the hardware
enum class HWStatus
{
    eNone = 0,         // a dummy status - indicates no state
    eDisconnected = 1, // disconnected - till robot is not connected or when "stop" action has been executed
    eConnected = 2,    // connect - only connection has been established or when "reset" has been done by the user
    eConfigured = 3,   // config updated on the backend
    eMoving = 4,       // only when the robot succesfully starts
    eWaiting = 5,      // waiting for the task execution to be completed
    eError = 6,        // any failure
    eRecording = 7     // recording the data
};

/// @brief tasks to be executed by the executor
enum class ExecTask
{
    eHandleRequest,
    eGetResponse
};

/// @brief recording mode
enum class RecordingMode
{
    eNone = 0,
    eContinous = 1,
    eIntermediate = 2
};

/// @brief APIs visible on the UI
enum class UIVisAPI
{
    eNone = 0,
    eRecJogAxis = 1,
    eRecJogRPY = 2,
    eRecJogJoint = 3,
    eRecFlexJogAxis = 4,
    eRecFlexJogRPY = 5,
    eRecFlexJogJoint = 6,
    eRecFreeDrive = 7,
    eRecMultiPoint = 8,
    eCustomAPI1 = 9,
    eCustomAPI2 = 10,
    eRecordedScripts = 11,
    eJogAxis = 12,
    eJogRPY = 13,
    eJogJoint = 14,
    eFlexJogAxis = 15,
    eFlexJogRPY = 16,
    eFlexJogJoint = 17,
    eLinearVelocity = 18,
    eFlexLinearVelocity = 19,
    eFreeDrive = 20,
    eMultiPoint = 21
};

/************************ Communication Structures ********************************/

/// @brief Meta of the RequestJSON
// basically refers to that part of the RequestJSON which is mandatory in each exchange of data between front-end and back-end
// for the RequestJSON, Meta = the action requested by the user/front-end
struct RequestMeta
{
    /// @brief action requested by user
    int action = []
    {
        struct_mapping::reg(&RequestMeta::action, "action");
        return static_cast<int>(UIAction::eNone);
    }();

    /// @brief print method
    void print()
    {
        std::cout << "request meta \n";
        std::cout << "action : " << action << "\n";
    };
};

/// @brief Meta for the ResponseJSON
// basically refers to that part of the RequestJSON which is mandatory in each exchange of data between front-end and back-end
// for the ResponseJSON, Meta = current status of the robot. This status might not always necessarily be in response some task given by the user, but shall be representing status of the robot
struct ResponseMeta
{
    /// @brief status (in response to the requested action)
    int status = []
    {
        struct_mapping::reg(&ResponseMeta::status, "status");
        return static_cast<int>(HWStatus::eNone);
    }();

    /// @brief print method
    void print()
    {
        std::cout << "response meta \n";
        std::cout << "status : " << status << "\n";
    };
};


/// @brief Configuration parameters for the setup of FT sensor
struct FTConfig
{
    /// @brief FT Sensor type at the end-effector
    int ft_type = []
    {
        struct_mapping::reg(&FTConfig::ft_type, "ft_type");
        return 0;
    }();

    /// @brief moi of the payload at the end-effector
    std::vector<double> ft_rotation_matrix = []
    {
        struct_mapping::reg(&FTConfig::ft_rotation_matrix, "ft_rotation_matrix");
        return std::vector<double>{};
    }();

    /// @brief print method
    void print()
    {
        std::cout << "ft type : " << ft_type << std::endl;
        std::cout << "ft rotation matrix : ";
        for (int i = 0; i < 9; i++)
        {
            std::cout << ft_rotation_matrix[i] << "\t";
        }
        std::cout << std::endl;
    }
};

/// @brief configuration to setup gripper
struct GripperConfig
{
    /// @brief gripper type at the end-effector
    int gripper_type = []
    {
        struct_mapping::reg(&GripperConfig::gripper_type, "gripper_type");
        return 0;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "gripper type : " << gripper_type << std::endl;
    }
};

/// @brief container to be used in case of error
// gives a code corresponding to the error and a short human-fathomable description of the same
struct ErrorContainer
{
    /// @brief error code
    int code = []
    {
        struct_mapping::reg(&ErrorContainer::code, "code");
        return static_cast<int>(HealErrors::Code::eNone);
    }();

    /// @brief error description string
    std::string desc = []
    {
        struct_mapping::reg(&ErrorContainer::desc, "description");
        return std::string();
    }();

    /// @brief print method
    void print()
    {
        std::cout << "error container \n";
        std::cout << "code : " << code << "\ndescription message : " << desc << std::endl;
    }
};

/// @brief payload configuration for robot
// contains mass, centre of mass and moment of inertia values based on the attachment at the end of the robot
/// also contains information about the type of gripper and FT sensor, or any such attachment at the end of the robot
struct PayloadConfig
{

    /// @brief mass at the end-effector
    double mass = []
    {
        struct_mapping::reg(&PayloadConfig::mass, "mass");
        return 0;
    }();

    /// @brief com of the payload at the end-effector
    std::vector<double> com = []
    {
        struct_mapping::reg(&PayloadConfig::com, "com");
        return std::vector<double>{};
    }();

    /// @brief moi of the payload at the end-effector
    std::vector<double> moi = []
    {
        struct_mapping::reg(&PayloadConfig::moi, "moi");
        return std::vector<double>{};
    }();

    /// @brief configuration for the attachment
    std::optional<FTConfig> ft_config = []
    {
        struct_mapping::reg(&PayloadConfig::ft_config, "ft_config");
        return std::nullopt;
    }();

    /// @brief configuration for the attachment
    std::optional<GripperConfig> gripper_config = []
    {
        struct_mapping::reg(&PayloadConfig::gripper_config, "gripper_config");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "payload details \n";
        std::cout << "mass : " << mass << std::endl
                  << "com : ";
        for (int i = 0; i < 3; i++)
        {
            std::cout << com[i] << "\t";
        }
        std::cout << std::endl
                  << "moi : ";
        for (int i = 0; i < 6; i++)
        {
            std::cout << moi[i] << "\t";
        }
        std::cout << std::endl;

        if (ft_config)
        {
            ft_config->print();
        }

        if (gripper_config)
        {
            gripper_config->print();
        }
    };
};

/// @brief details of the script (recorded data set)
// Use-case:
// Before recording : specifies the name by which the script is to be saved and the mode of data recording for this script
// After recording : number of times the selected script has to be played on the robot
struct ScriptConfig
{
    /// @brief label for the script/motion sequence
    std::string label = []
    {
        struct_mapping::reg(&ScriptConfig::label, "label");
        return std::string();
    }();

    /// @brief iterations of the script
    int iterations = []
    {
        struct_mapping::reg(&ScriptConfig::iterations, "iterations");
        return 0;
    }();

    /// @brief mode of recording : continous or intermediate
    int mode = []
    {
        struct_mapping::reg(&ScriptConfig::mode, "mode");
        return static_cast<int>(RecordingMode::eNone);
    }();

    /// @brief print
    void print()
    {
        std::cout << "ScriptConfig : \n";
        std::cout << "label : " << label << std::endl;
        std::cout << "iterations : " << iterations << std::endl;
        std::cout << "mode : " << mode << std::endl;
    }
};

/// @brief Single Data Point to be saved when recording the data while teaching
// holds the target position to be achieved and the time in which to achieve the position
struct RecordedDataEntry
{
    /// @brief target position
    std::vector<double> desired_pos = []
    {
        struct_mapping::reg(&RecordedDataEntry::desired_pos, "desired_pos");
        return std::vector<double>{};
    }();

    /// @brief time to reach the target
    double desired_time = []
    {
        struct_mapping::reg(&RecordedDataEntry::desired_time, "desired_time");
        return 0;
    }();

    /// print method
    void print(int i = -1)
    {
        if (i == -1)
        {
            std::cout << "target position :\t";
        }
        else
        {
            std::cout << "target position (entry number : " << i << " ) :\t";
        }
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << desired_pos[i] << "\t";
        }
        std::cout << "in time : " << desired_time << std::endl;
    }
};

/// @brief DataSet made up of multiple consecutive Single Data Points that were recorded while teaching the robot
// will be used to replay the taught motions
// this is basically the content of any of the script used for recording
struct RecordedDataSet
{
    /// @brief dataset : vector of individual data points
    std::vector<RecordedDataEntry> dataset = []
    {
        struct_mapping::reg(&RecordedDataSet::dataset, "dataset");
        return std::vector<RecordedDataEntry>{};
    }();

    /// print method
    void print()
    {
        std::cout << "recorded data set\n";
        for (int i = 0; i < dataset.size(); i++)
        {
            dataset[i].print(i);
        }
    }

    /// @brief method to append new entry to the existing data set
    /// @param entry
    void addEntry(RecordedDataEntry entry)
    {
        dataset.push_back(entry);
    }

    /// @brief clear the current data set
    void clear()
    {
        dataset.clear();
    }

    /// @brief check if current dataset is empty
    /// @return true if the dataset is empty
    bool isEmpty()
    {
        return (dataset.empty());
    }

    /// @brief get the size of the current dataset
    /// @return
    int getSize() const
    {
        return (dataset.size());
    }
};

/// @brief Configuration details for the controller
//  contains details like
// - the controller id to run the robot with
// - the goal position
// - time to achieve the goal position
// - safety mode
// - flexibility factor
struct ControllerConfig
{
    /// @brief controller ID
    int controller = []
    {
        struct_mapping::reg(&ControllerConfig::controller, "controller");
        return static_cast<int>(API::eNone);
    }();

    /// @brief goal position
    std::vector<double> target_pos = []
    {
        struct_mapping::reg(&ControllerConfig::target_pos, "target_position");
        return std::vector<double>{};
    }();

    /// @brief time to acheive the goal pos
    double target_time = []
    {
        struct_mapping::reg(&ControllerConfig::target_time, "target_time");
        return 0;
    }();

    /// @brief factor of flexibility
    double flex_factor = []
    {
        struct_mapping::reg(&ControllerConfig::flex_factor, "flexibility_factor");
        return 0;
    }();

    /// @brief mode of safety - defaults to baseline mode
    int safety_mode = []
    {
        struct_mapping::reg(&ControllerConfig::safety_mode, "safety_mode");
        return 0;
    }();

    /// @brief sequence of target poses - to be used in case multiple target points are to be specified
    std::vector<std::vector<double>> target_pos_seq = []
    {
        struct_mapping::reg(&ControllerConfig::target_pos_seq, "target_position_seq");
        return std::vector<std::vector<double>>{};
    }();

    /// @brief sequence of target time values - to be used in case multiple target points are to be specified
    std::vector<double> target_time_seq = []
    {
        struct_mapping::reg(&ControllerConfig::target_time_seq, "target_time_seq");
        return std::vector<double>{};
    }();

    /// @brief details of the recorded motion sequence - including label, number of iterations to play the recorded motion
    std::optional<ScriptConfig> script_config = []
    {
        struct_mapping::reg(&ControllerConfig::script_config, "script_config");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "controller : " << controller << "\ndes_time : " << target_time << std::endl;
        std::cout << "flexibility : " << flex_factor << "safety_mode : " << safety_mode << std::endl;

        if (target_pos.size() != 0)
        {
            std::cout << "goal pos vector size : " << target_pos.size() << "\ngoal pos : \n";
            for (int i = 0; i < N_DOF; i++)
            {
                std::cout << target_pos[i] << "\t";
            }
            std::cout << std::endl;
        }

        if (target_pos_seq.size() != 0)
        {
            std::cout << "number of target positions : " << target_pos_seq.size() << "\nvalues : \n";
            for (int j = 0; j < target_pos_seq.size(); j++)
            {
                for (int i = 0; i < N_DOF; i++)
                {
                    std::cout << target_pos_seq[j][i] << "\t";
                }
                std::cout << std::endl;
            }
        }

        if (target_time_seq.size() != 0)
        {
            std::cout << "target time specified for : " << target_time_seq.size() << " points\nvalues : \n";
            for (int i = 0; i < target_time_seq.size(); i++)
            {
                std::cout << target_time_seq[i] << "\n";
            }
            std::cout << std::endl;
        }

        if (script_config)
        {
            script_config->print();
        }
    };
};

/// @brief Controller configuration that are not required to be filled in for using basic controllers, but will be needed for advanced control techniques
//  like playing a recorded (taught) motion sequence on the robot
struct AdvancedControllerConfig
{
    /// @brief recorded motion sequence
    std::optional<RecordedDataSet> dataset = []
    {
        struct_mapping::reg(&AdvancedControllerConfig::dataset, "dataset");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "Adavnced Controller config print()\n";
        if (dataset)
        {

            dataset->print();
        }
    }
};

/// @brief Control Inputs to be provided to the robot when running it in Jogging mode/External control mode
// it is the set of control inputs that would be needed at each computation step
struct ControlInterrupt
{
    /// @brief cartesian jogging input
    std::vector<double> cart_jog = []
    {
        struct_mapping::reg(&ControlInterrupt::cart_jog, "jog_cartesian");
        return std::vector<double>{};
    }();

    /// @brief joint jogging input
    std::vector<double> joint_jog = []
    {
        struct_mapping::reg(&ControlInterrupt::joint_jog, "jog_joint");
        return std::vector<double>{};
    }();

    /// @brief external control input
    std::vector<double> ext_ctrl = []
    {
        struct_mapping::reg(&ControlInterrupt::ext_ctrl, "external_control");
        return std::vector<double>{};
    }();

    /// @brief stiffness values for the joints
    std::vector<std::vector<double>> stiffness_matrix = []
    {
        struct_mapping::reg(&ControlInterrupt::stiffness_matrix, "stiffness_matrix");
        return std::vector<std::vector<double>>{};
    }();

    /// @brief damping values for the joints
    std::vector<std::vector<double>> damping_matrix = []
    {
        struct_mapping::reg(&ControlInterrupt::damping_matrix, "damping_matrix");
        return std::vector<std::vector<double>>{};
    }();

    /// @brief mass_matrix values for the joints
    std::vector<std::vector<double>> mass_matrix = []
    {
        struct_mapping::reg(&ControlInterrupt::mass_matrix, "mass_matrix");
        return std::vector<std::vector<double>>{};
    }();

    /// @brief ft force values for the joints
    std::vector<double> ft_force = []
    {
        struct_mapping::reg(&ControlInterrupt::ft_force, "ft_force");
        return std::vector<double>{};
    }();

    /// @brief target force values for the joints
    std::vector<double> target_force = []
    {
        struct_mapping::reg(&ControlInterrupt::target_force, "target_force");
        return std::vector<double>{};
    }();

    /// @brief print
    void print()
    {
        if (cart_jog.size() != 0)
        {
            std::cout << "jog_cartesian\n";
            for (int i = 0; i < 6; i++)
            {
                std::cout << cart_jog[i] << "\t";
            }
            std::cout << std::endl;
        }

        if (joint_jog.size() != 0)
        {
            std::cout << "jog_joint\n";
            for (int i = 0; i < N_DOF; i++)
            {
                std::cout << joint_jog[i] << "\t";
            }
            std::cout << std::endl;
        }

        if (ext_ctrl.size() != 0)
        {
            std::cout << "external_control\n";
            for (int i = 0; i < N_DOF; i++)
            {
                std::cout << ext_ctrl[i] << "\t";
            }
            std::cout << std::endl;
        }
    };
};

/// @brief Control Input additional to the one obtained via jogging/external control mode to control the robot, which is provided at run time by the user
//  for instance which data points to record while teaching the robot
struct AdditionalInput
{
    /// @brief data entry point and time input to allow for saving points while running the robot
    std::optional<RecordedDataEntry> data_entry = []
    {
        struct_mapping::reg(&AdditionalInput::data_entry, "data_entry");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "AdditionalInput print()\n";
        if (data_entry)
        {
            data_entry->print();
        }
    }
};

/// @brief Feedback received from the robot at all instants of time
// contains summary of the state of the robot with data like
// - Joint Position
// - Joint Velocity
// - Joint Acceleration
// - Joint Torque
// - Joint wise error code
// - Joint wise status code
// - end-effector pose
// - magnitude of force at the end-effector
// - direction of force at the end-effector
// - time stamp at which the data was fetched from the hardware
struct RobotFeedback
{
    /// @brief joint position
    std::vector<double> jpos = []
    {
        struct_mapping::reg(&RobotFeedback::jpos, "joint_position");
        return std::vector<double>{};
    }();

    /// @brief joint velocity
    std::vector<double> jvel = []
    {
        struct_mapping::reg(&RobotFeedback::jvel, "joint_velocity");
        return std::vector<double>{};
    }();

    /// @brief joint acceleration
    std::vector<double> jacc = []
    {
        struct_mapping::reg(&RobotFeedback::jacc, "joint_acceleration");
        return std::vector<double>{};
    }();

    /// @brief joint torque
    std::vector<double> jtor = []
    {
        struct_mapping::reg(&RobotFeedback::jtor, "joint_torque");
        return std::vector<double>{};
    }();

    /// @brief actuator error code(as recd from lower level)
    std::vector<double> jerr = []
    {
        struct_mapping::reg(&RobotFeedback::jerr, "joint_error_code");
        return std::vector<double>{};
    }();

    /// @brief actuator status(as recd from lower level)
    std::vector<double> jstatus = []
    {
        struct_mapping::reg(&RobotFeedback::jstatus, "joint_status");
        return std::vector<double>{};
    }();

    /// @brief pose of the end-effector(x,y,z,phi,theta,psi)
    std::vector<double> ee_pos = []
    {
        struct_mapping::reg(&RobotFeedback::ee_pos, "end_effector_position");
        return std::vector<double>{};
    }();

    /// @brief force at the end effector
    std::vector<double> force_val = []
    {
        struct_mapping::reg(&RobotFeedback::force_val, "force_magnitude");
        return std::vector<double>{};
    }();

    /// @brief force direction vector
    std::vector<double> force_dir = []
    {
        struct_mapping::reg(&RobotFeedback::force_dir, "force_direction");
        return std::vector<double>{};
    }();

    /// @brief time stamp
    double time_stamp = []
    {
        struct_mapping::reg(&RobotFeedback::time_stamp, "time_stamp");
        return 0;
    }();

    /// @brief print data members
    void print()
    {
        std::cout << "robot_feedback :\n";
        // std::cout<<"I am here\n";
        std::cout << "time stamp : " << time_stamp << std::endl;

        /// joint position
        std::cout << "joint position : ";
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << jpos[i] << "\t";
        }
        std::cout << "\n";

        // joint velocity
        std::cout << "joint velocity : ";
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << jvel[i] << "\t";
        }
        std::cout << "\n";

        // joint acceleration
        std::cout << "joint acceleration : ";
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << jacc[i] << "\t";
        }
        std::cout << "\n";

        // joint torque
        std::cout << "joint torque : ";
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << jtor[i] << "\t";
        }
        std::cout << "\n";

        // joint error code
        std::cout << "joint error code : ";
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << jerr[i] << "\t";
        }
        std::cout << "\n";

        // joint status
        std::cout << "joint status : ";
        for (int i = 0; i < N_DOF; i++)
        {
            std::cout << jstatus[i] << "\t";
        }
        std::cout << "\n";

        // end-effector position
        std::cout << "end-effector position : ";
        for (int i = 0; i < 6; i++)
        {
            std::cout << ee_pos[i] << "\t";
        }
        std::cout << "\n";

        // force magnitude
        std::cout << "force magnitude : ";
        for (int i = 0; i < 6; i++)
        {
            std::cout << force_val[i] << "\t";
        }
        std::cout << "\n";

        // force direction
        std::cout << "force direction : ";
        for (int i = 0; i < 6; i++)
        {
            std::cout << force_dir[i] << "\t";
        }
        std::cout << "\n";
    };

    /// @brief method to initialise the data members through default initialisation
    void init()
    {
        jpos = std::vector<double>(N_DOF, 0);
        jvel = std::vector<double>(N_DOF, 0);
        jacc = std::vector<double>(N_DOF, 0);
        jtor = std::vector<double>(N_DOF, 0);
        jerr = std::vector<double>(N_DOF, 0);
        jstatus = std::vector<double>(N_DOF, 0);

        ee_pos = std::vector<double>(6, 0);
        force_val = std::vector<double>(6, 0);
        force_dir = std::vector<double>(6, 0);
    }
};

/// @brief Cartesian Jogging limits to be used for display of planes in the UI
struct CartJoggingLimits
{
    // depicts which of the four planes to highlight
    int plane_id = []
    {
        struct_mapping::reg(&CartJoggingLimits::plane_id, "plane_id");
        return 0;
    }();

    // a factor represeting how close the current TCP position is wrt the plane
    // can have a value in the range of 0 to 1
    // where 1 means on the safety_plane and 0 means TCP has come within a threshold of the safety_plane
    float proximity_scale = []
    {
        struct_mapping::reg(&CartJoggingLimits::proximity_scale, "proximity_scale");
        return 0;
    }();

    /// Radius of the inner cylinder
    float inner_radius = []
    {
        struct_mapping::reg(&CartJoggingLimits::inner_radius, "inner_radius");
        return 0;
    }();

    /// Radius of the outer cylinder
    float outer_radius = []
    {
        struct_mapping::reg(&CartJoggingLimits::outer_radius, "outer_radius");
        return 0;
    }();

    /// Height of the lower plane
    float lower_plane = []
    {
        struct_mapping::reg(&CartJoggingLimits::lower_plane, "lower_plane");
        return 0;
    }();

    /// Height of the upper plane
    float upper_plane = []
    {
        struct_mapping::reg(&CartJoggingLimits::upper_plane, "upper_plane");
        return 0;
    }();

    /// @brief x-coordinate of the centre of the circle
    float centre_x = []
    {
        struct_mapping::reg(&CartJoggingLimits::centre_x, "centre_x");
        return 0;
    }();

    /// @brief y-coordinate of the centre of the circle
    float centre_y = []
    {
        struct_mapping::reg(&CartJoggingLimits::centre_y, "centre_y");
        return 0;
    }();

    /// @brief print methodology
    void print()
    {
        std::cout << "CartJoggingLimits\n";
        std::cout << "plane id : " << plane_id << std::endl;
        std::cout << "proximity scale : " << proximity_scale << std::endl;
    }
};

/// @brief optional details - which will be a part of the RequestJSON only when user makes a request to run the robot with some controller
struct RequestPayload
{
    /// @brief payload (attachment details) for the end effector
    std::optional<PayloadConfig> payload = []
    {
        struct_mapping::reg(&RequestPayload::payload, "payload");
        return std::nullopt;
    }();

    /// @brief configuration parameters for the controller
    std::optional<ControllerConfig> control_config = []
    {
        struct_mapping::reg(&RequestPayload::control_config, "controller_config");
        return std::nullopt;
    }();

    /// @brief interrupt for controlling robot in jogging/external control mode
    std::optional<ControlInterrupt> control_int = []
    {
        struct_mapping::reg(&RequestPayload::control_int, "control_input");
        return std::nullopt;
    }();

    /// @brief additional inputs while running the robot, but inputs that are not needed to directly control the robot
    std::optional<AdditionalInput> additional_input = []
    {
        struct_mapping::reg(&RequestPayload::additional_input, "additional_input");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "RequestPayload print()\n";
        if (payload)
        {
            payload->print();
        }
        if (control_config)
        {
            control_config->print();
        }
        if (control_int)
        {
            control_int->print();
        }
        if (additional_input)
        {
            additional_input->print();
        }
    }
};

/// @brief JSON containing the request from the user
// has fields
// - meta - mandatory; contains the action to be performed
// - error - an optional field; to be filled in, if there is any error
// - data - an optional field; contains data associated with the controller to run the controller
struct RequestJSON
{
    /// @brief mandatory part of the request - contains action for the server
    RequestMeta meta = []
    {
        struct_mapping::reg(&RequestJSON::meta, "meta");
        return RequestMeta();
    }();

    /// @brief optional error container, to be filled when system is in error
    std::optional<ErrorContainer> error = []
    {
        struct_mapping::reg(&RequestJSON::error, "error");
        return std::nullopt;
    }();

    /// @brief optional data field, to be populated with controller configuration
    std::optional<RequestPayload> data = []
    {
        struct_mapping::reg(&RequestJSON::data, "data");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        meta.print();
        if (error)
        {
            error->print();
        }
        if (data.has_value())
        {
            data->print();
        }
    }
};

/// @brief configuration details requested by the front-end for dispaly purposes
// currently , this contains the list of labels of recorded (taught) motion sequences
struct Configuration
{
    /// @brief list of recorded script labels
    std::vector<std::string> scripts = []
    {
        struct_mapping::reg(&Configuration::scripts, "scripts");
        return std::vector<std::string>{};
    }();

    /// @brief print method
    void print()
    {
        std::cout << "configuration print()\n";

        if (scripts.size() > 0)
        {
            for (int i = 0; i < scripts.size(); i++)
            {
                std::cout << scripts[i] << std::endl;
            }
        }
    }
};

/// @brief optional field of the ResponseJSON
// contains the RobotFeedback and Configuration data
struct ResponsePayload
{
    /// @brief configuration of the robot and/or backend server, required for display on the front-end
    std::optional<Configuration> config = []
    {
        struct_mapping::reg(&ResponsePayload::config, "configuration");
        return std::nullopt;
    }();

    /// @brief feedback giving the current robot state
    std::optional<RobotFeedback> feedback = []
    {
        struct_mapping::reg(&ResponsePayload::feedback, "feedback");
        return std::nullopt;
    }();

    /// @brief optional field to give cartesian limits hitting status for robot when jogging in cartesian space
    std::optional<CartJoggingLimits> cart_jogging_limits = []
    {
        struct_mapping::reg(&ResponsePayload::cart_jogging_limits, "cart_jogging_limits");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        if (config)
        {
            config->print();
        }

        if (feedback)
        {
            feedback->print();
        }

        if (cart_jogging_limits)
        {
            cart_jogging_limits->print();
        }
    }
};

/// @brief JSON structure for the response to the front-end
// contains
// meta - mandatory field, contains status of the robot
// error - optional field, contains error code and message
// data - optional field, contains the RobotFeedback
struct ResponseJSON
{
    /// @brief mandatory field, contains the status of the robot
    ResponseMeta meta = []
    {
        struct_mapping::reg(&ResponseJSON::meta, "meta");
        return ResponseMeta();
    }();

    /// @brief optional error container, to be filled when system is in error
    std::optional<ErrorContainer> error = []
    {
        struct_mapping::reg(&ResponseJSON::error, "error");
        return std::nullopt;
    }();

    /// @brief optional data field, to be populated with feedback from the robot
    std::optional<ResponsePayload> data = []
    {
        struct_mapping::reg(&ResponseJSON::data, "data");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        meta.print();
        if (error)
        {
            error->print();
        }
        if (data)
        {
            data->print();
        }
    };

    /// @brief append configuration to existing data field
    /// @param cont
    void append(Configuration cont)
    {
        if (data)
        {
            data->config = cont;
        }
        else
        {
            ResponsePayload res;
            res.config = cont;
            data = res;
        }
    }

    /// @brief append feedback to existing data field
    /// @param feedback
    void append(RobotFeedback feedback)
    {
        if (data)
        {
            data->feedback = feedback;
        }
        else
        {
            ResponsePayload res;
            res.feedback = feedback;
            data = res;
        }
    }
};
/// @brief configuration provided by user to execute the requested action
struct UserConfig
{
    /// @brief payload(attachment) details
    std::optional<PayloadConfig> payload = []
    {
        struct_mapping::reg(&UserConfig::payload, "payload");
        return std::nullopt;
    }();

    /// @brief configurations to run basic controllers
    std::optional<ControllerConfig> controller_config = []
    {
        struct_mapping::reg(&UserConfig::controller_config, "controller_config");
        return std::nullopt;
    }();

    /// @brief configurations to run advanced controllers
    std::optional<AdvancedControllerConfig> advanced_controller_config = []
    {
        struct_mapping::reg(&UserConfig::advanced_controller_config, "advanced_controller_config");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "User config print()" << std::endl;
        if (payload)
        {
            payload->print();
        }
        if (controller_config)
        {
            controller_config->print();
        }
        if (advanced_controller_config)
        {
            advanced_controller_config->print();
        }
    }

    /// @brief clears the structure of any allocated memory
    void clear()
    {
        payload.reset();
        controller_config.reset();
        advanced_controller_config.reset();
    }

    /// @brief append payload to the current config
    void append(const PayloadConfig &payload_in)
    {
        payload = payload_in;
    }

    /// @brief append controller config to the current config
    /// @param controller_config
    void append(const ControllerConfig &config_in)
    {
        controller_config = config_in;
    }

    /// @brief append advanced controller config to the current config
    /// @param config_in
    void append(const AdvancedControllerConfig &config_in)
    {
        advanced_controller_config = config_in;
    }
};

/// @brief status of the event as requested by the user
struct EventStatus
{
    /// @brief event executed/in execution by the robot
    int event = []
    {
        struct_mapping::reg(&EventStatus::event, "event");
        return 0;
    }();

    /// @brief current status of the event
    int status = []
    {
        struct_mapping::reg(&EventStatus::status, "status");
        return 0;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "event status summary:- \n";
        std::cout << "\nevent : " << event << "\nstatus : " << status << std::endl;
    }

    /// @brief reset to default values
    void reset()
    {
        event = 0;
        status = 0;
    }
};

/// @brief configuration for any event, if given by the user
struct EventConfig
{
    /// @brief event to be executed by the robot
    int event = []
    {
        struct_mapping::reg(&EventConfig::event, "event");
        return 0;
    }();

    /// @brief force to clamp the object with
    double clamp_force = []
    {
        struct_mapping::reg(&EventConfig::clamp_force, "clamp_force");
        return 0;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "event config : \n";
        std::cout << "\nevent : " << event << "\nclamp_force : " << clamp_force << std::endl;
    }
};

/*****************************  Executor Structures  *************************************/

/// @brief structure containing the task to be executed and the details of the task
struct ExecTaskStruct
{
    /// @brief task to be executed
    ExecTask task;

    /// @brief task details
    std::string request;
};

/// @brief structure containing output obtained as a result of execution of the last requested task
struct ExecDataStruct
{
    /// @brief details in response to the last requested task
    std::string response;
};

/// @brief configuration to operate gripper
struct GripperCmd
{
    /// @brief force to close the gripper
    double clamp_force;

    /// @brief clear all values
    void reset()
    {
        clamp_force = 0;
    }

    /// @brief ctor
    GripperCmd()
    {
        reset();
    }
};

/**************************    Robot Interaction Data Structures      ********************************/

/// @brief Interrupt to be sent from the Backend to the Robot
// consisting of the action for the robot and error, if any
struct UIInterrupt
{
    /// @brief action requested by user
    int action = []
    {
        struct_mapping::reg(&UIInterrupt::action, "action");
        return 0;
    }();

    /// @brief error , if any
    int error = []
    {
        struct_mapping::reg(&UIInterrupt::error, "error");
        return 0;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "ui_interrupt : \n";
        std::cout << "action : " << action << "\nerror : " << error << std::endl;
    };
};

/// @brief Interrupt from the robot to the Backend
// updated from the robot at a given frequency
// contains
// - Action for the UI from the robot, if any
// - Current state of the robot
// - error if any
struct RobotInterrupt
{
    /// @brief action for UI
    int action = []
    {
        struct_mapping::reg(&RobotInterrupt::action, "action");
        return 0;
    }();

    /// @brief current state of the robot
    int state = []
    {
        struct_mapping::reg(&RobotInterrupt::state, "state");
        return 0;
    }();

    /// @brief error, if any
    int error = []
    {
        struct_mapping::reg(&RobotInterrupt::error, "error");
        return 0;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "robot_interrupt print() called\n";
        std::cout << "action : " << action << "\n"
                  << "state : " << state << "\n"
                  << "error : " << error << std::endl;
    };
};

/// @brief Data format to be used in exchange of data between robot and Backend
// this is the data that backend sends to the robot
// includes the action for the robot, along with accompanying data to execute the action
struct SocketTx
{
    /// @brief interrupt giving action desired by user
    UIInterrupt interrupt = []
    {
        struct_mapping::reg(&SocketTx::interrupt, "interrupt");
        return UIInterrupt();
    }();

    /// @brief configuration details provided by the user to execute the event
    std::optional<EventConfig> event_config = []
    {
        struct_mapping::reg(&SocketTx::event_config, "event_config");
        return std::nullopt;
    }();

    /// @brief configuration details provided by the user to execute the action
    std::optional<UserConfig> config = []
    {
        struct_mapping::reg(&SocketTx::config, "config");
        return std::nullopt;
    }();

    /// @brief run-time control inputs provided by the user to execute the action
    std::optional<ControlInterrupt> control = []
    {
        struct_mapping::reg(&SocketTx::control, "control_int");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "sockettx print()\n";
        interrupt.print();

        if (event_config)
        {
            event_config->print();
        }
        if (config)
        {
            config->print();
        }
        if (control)
        {
            control->print();
        }
    };

    /// @brief clear the received structure
    void clear()
    {
        interrupt = UIInterrupt();
        event_config.reset();
        config.reset();
        control.reset();
    };
};

/// @brief Data format to be used in exchange of data between robot and Backend
// this is the data that robot sends to the backend
// includes the current state of the robot, error details if any, along with detailed feedback of the robot state
struct SocketRx
{
    /// @brief interrupt - contains state of the robot
    RobotInterrupt interrupt = []
    {
        struct_mapping::reg(&SocketRx::interrupt, "interrupt");
        return RobotInterrupt();
    }();

    /// @brief status update on the last event as requested by the user
    std::optional<EventStatus> event_status = []
    {
        struct_mapping::reg(&SocketRx::event_status, "event_status");
        return std::nullopt;
    }();

    /// @brief detailed joint-wise feedback of the robot
    std::optional<RobotFeedback> feedback = []
    {
        struct_mapping::reg(&SocketRx::feedback, "feedback");
        return std::nullopt;
    }();

    /// @brief print method
    void print()
    {
        std::cout << "socketRx print() \n";
        interrupt.print();
        if (event_status)
        {
            event_status->print();
        }
        if (feedback)
        {
            feedback->print();
        }
    };

    /// @brief clear the received data structure
    void clear()
    {
        interrupt = RobotInterrupt();
        feedback.reset();
        event_status.reset();
    };
};

#endif
