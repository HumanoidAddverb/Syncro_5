#include "utility/hardware_interface_defs.h"

std::string addverb_cobot::hw_interface_defs::gpioTypeToString(GpioType type)
{
    switch (type)
    {
        case GpioType::TimeCmd:
            return "TimeCmd";
        case GpioType::PtpTransfer:
            return "PtpTransfer";
        case GpioType::RecordCmd:
            return "RecordCmd";
        case GpioType::TcpTimeCmd:
            return "TcpTimeCmd";
        case GpioType::PtpTcpTransfer:
            return "PtpTcpTransfer";
        case GpioType::Tcp:
            return "Tcp";
        case GpioType::JointJogging:
            return "JointJogging";
        case GpioType::CartesianJogging:
            return "CartesianJogging";
        case GpioType::JointImpedance:
            return "JointImpedance";
        case GpioType::CartesianImpedance:
            return "CartesianImpedance";
        case GpioType::ControllerName:
            return "ControllerName";
        case GpioType::RobotState:
            return "RobotState";
        case GpioType::Gripper:
            return "Gripper";
        default:
            return "Unknown";
    }
}

const std::vector<std::pair<std::string, addverb_cobot::hw_interface_defs::GpioType>>
addverb_cobot::hw_interface_defs::GpioEntry::gpio_map =
{
    {"time_cmd", GpioType::TimeCmd},
    {"ptp_transfer", GpioType::PtpTransfer},
    {"record_cmd", GpioType::RecordCmd},
    {"tcp_time_cmd", GpioType::TcpTimeCmd},
    {"ptp_tcp_transfer", GpioType::PtpTcpTransfer},
    {"joint_jogging", GpioType::JointJogging},
    {"cartesian_jogging", GpioType::CartesianJogging},
    {"joint_impedance", GpioType::JointImpedance},
    {"cartesian_impedance", GpioType::CartesianImpedance},
    {"controller_name", GpioType::ControllerName},
    {"robot_state", GpioType::RobotState},
    {"gripper", GpioType::Gripper},
    {"tcp", GpioType::Tcp} // keep generic ones last

};
std::ostream& addverb_cobot::hw_interface_defs::operator<<(std::ostream& os, GpioType type)
{
    os << gpioTypeToString(type);
    return os;
}

std::ostream& addverb_cobot::hw_interface_defs::operator<<(std::ostream& os, GpioEntry entry)
{
    os << "Gpio Struct for robot id " << entry.robot_id << " GPIO " << entry.type;
    return os;
}