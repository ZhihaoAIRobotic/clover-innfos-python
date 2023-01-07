
#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include "actuatordefine.h"
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_unknown_unknown(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// Actuator::ConnectStatus file: line:10
	pybind11::enum_<Actuator::ConnectStatus>(M("Actuator"), "ConnectStatus", pybind11::arithmetic(), "连接状态，用于执行器和CAN的连接状态判断：")
		.value("NO_CONNECT", Actuator::NO_CONNECT)
		.value("CAN_CONNECTED", Actuator::CAN_CONNECTED)
		.value("ACTUATOR_CONNECTED", Actuator::ACTUATOR_CONNECTED)
		.export_values();

;

	// Actuator::Channel_ID file: line:21
	pybind11::enum_<Actuator::Channel_ID>(M("Actuator"), "Channel_ID", pybind11::arithmetic(), "通道ID,用于标识执行器图表数据的通道索引：")
		.value("channel_1", Actuator::channel_1)
		.value("channel_2", Actuator::channel_2)
		.value("channel_3", Actuator::channel_3)
		.value("channel_4", Actuator::channel_4)
		.value("channel_cnt", Actuator::channel_cnt)
		.export_values();

;

	// Actuator::ErrorsDefine file: line:31
	pybind11::enum_<Actuator::ErrorsDefine>(M("Actuator"), "ErrorsDefine", pybind11::arithmetic(), "错误类型定义，定义了执行器内部和连接等错误代码：")
		.value("ERR_NONE", Actuator::ERR_NONE)
		.value("ERR_ACTUATOR_OVERVOLTAGE", Actuator::ERR_ACTUATOR_OVERVOLTAGE)
		.value("ERR_ACTUATOR_UNDERVOLTAGE", Actuator::ERR_ACTUATOR_UNDERVOLTAGE)
		.value("ERR_ACTUATOR_LOCKED_ROTOR", Actuator::ERR_ACTUATOR_LOCKED_ROTOR)
		.value("ERR_ACTUATOR_OVERHEATING", Actuator::ERR_ACTUATOR_OVERHEATING)
		.value("ERR_ACTUATOR_READ_OR_WRITE", Actuator::ERR_ACTUATOR_READ_OR_WRITE)
		.value("ERR_ACTUATOR_MULTI_TURN", Actuator::ERR_ACTUATOR_MULTI_TURN)
		.value("ERR_INVERTOR_TEMPERATURE_SENSOR", Actuator::ERR_INVERTOR_TEMPERATURE_SENSOR)
		.value("ERR_CAN_COMMUNICATION", Actuator::ERR_CAN_COMMUNICATION)
		.value("ERR_ACTUATOR_TEMPERATURE_SENSOR", Actuator::ERR_ACTUATOR_TEMPERATURE_SENSOR)
		.value("ERR_STEP_OVER", Actuator::ERR_STEP_OVER)
		.value("ERR_DRV_PROTECTION", Actuator::ERR_DRV_PROTECTION)
		.value("ERR_CODER_DISABLED", Actuator::ERR_CODER_DISABLED)
		.value("ERR_ACTUATOR_DISCONNECTION", Actuator::ERR_ACTUATOR_DISCONNECTION)
		.value("ERR_CAN_DISCONNECTION", Actuator::ERR_CAN_DISCONNECTION)
		.value("ERR_IP_ADDRESS_NOT_FOUND", Actuator::ERR_IP_ADDRESS_NOT_FOUND)
		.value("ERR_ABNORMAL_SHUTDOWN", Actuator::ERR_ABNORMAL_SHUTDOWN)
		.value("ERR_SHUTDOWN_SAVING", Actuator::ERR_SHUTDOWN_SAVING)
		.value("ERR_IP_HAS_BIND", Actuator::ERR_IP_HAS_BIND)
		.value("ERR_IP_CONFLICT", Actuator::ERR_IP_CONFLICT)
		.value("ERR_ID_UNUNIQUE", Actuator::ERR_ID_UNUNIQUE)
		.value("ERR_UNKOWN", Actuator::ERR_UNKOWN)
		.export_values();

;

	// Actuator::OnlineStatus file: line:78
	pybind11::enum_<Actuator::OnlineStatus>(M("Actuator"), "OnlineStatus", pybind11::arithmetic(), "在线状态，用于标识执行器是否处于连接状态：")
		.value("Status_Online", Actuator::Status_Online)
		.value("Status_Offline", Actuator::Status_Offline)
		.export_values();

;

	// Actuator::SwitchStatus file: line:87
	pybind11::enum_<Actuator::SwitchStatus>(M("Actuator"), "SwitchStatus", pybind11::arithmetic(), "开关状态，标识执行器的开关机状态：")
		.value("ACTUATOR_SWITCH_OFF", Actuator::ACTUATOR_SWITCH_OFF)
		.value("ACTUATOR_SWITCH_ON", Actuator::ACTUATOR_SWITCH_ON)
		.export_values();

;

	// Actuator::ChartSwitchStatus file: line:96
	pybind11::enum_<Actuator::ChartSwitchStatus>(M("Actuator"), "ChartSwitchStatus", pybind11::arithmetic(), "图表开关，用于标识执行器图表功能的开启或关闭：")
		.value("CHART_SWITCH_OFF", Actuator::CHART_SWITCH_OFF)
		.value("CHART_SWITCH_ON", Actuator::CHART_SWITCH_ON)
		.export_values();

;

	// Actuator::CurrnetChart file: line:105
	pybind11::enum_<Actuator::CurrnetChart>(M("Actuator"), "CurrnetChart", pybind11::arithmetic(), "电流环图表索引，用于标识电流图表是IQ值还是ID值")
		.value("IQ_CHART", Actuator::IQ_CHART)
		.value("ID_CHART", Actuator::ID_CHART)
		.export_values();

;

	// Actuator::HomingOperationMode file: line:111
	pybind11::enum_<Actuator::HomingOperationMode>(M("Actuator"), "HomingOperationMode", pybind11::arithmetic(), "归零模式，分为手动和自动两种")
		.value("Homing_Auto", Actuator::Homing_Auto)
		.value("Homing_Manual", Actuator::Homing_Manual)
		.export_values();

;

	// Actuator::CommunicationType file: line:119
	pybind11::enum_<Actuator::CommunicationType>(M("Actuator"), "CommunicationType", pybind11::arithmetic(), "通信方式，可通过以太网或者串口两种方式与执行器通信，初始化执行器控制器时候要指定方式，默认为以太网通信：")
		.value("Via_Ethernet", Actuator::Via_Ethernet)
		.value("Via_Serialport", Actuator::Via_Serialport)
		.export_values();

;

	// Actuator::Initlize_state file: line:128
	pybind11::enum_<Actuator::Initlize_state>(M("Actuator"), "Initlize_state", pybind11::arithmetic(), "")
		.value("Uninitialized", Actuator::Uninitialized)
		.value("Initialized", Actuator::Initialized)
		.export_values();

;

	// Actuator::OperationFlags file: line:135
	pybind11::enum_<Actuator::OperationFlags>(M("Actuator"), "OperationFlags", pybind11::arithmetic(), "操作标识，标识操作完成，可用于判断执行器控制器的指令执行状态：")
		.value("Recognize_Finished", Actuator::Recognize_Finished)
		.value("Launch_Finished", Actuator::Launch_Finished)
		.value("Close_Finished", Actuator::Close_Finished)
		.value("Save_Params_Finished", Actuator::Save_Params_Finished)
		.value("Save_Params_Failed", Actuator::Save_Params_Failed)
		.value("Attribute_Change_Finished", Actuator::Attribute_Change_Finished)
		.export_values();

;

	// Actuator::ActuatorMode file: line:152
	pybind11::enum_<Actuator::ActuatorMode>(M("Actuator"), "ActuatorMode", pybind11::arithmetic(), "执行器模式，标识当前执行器的模式：")
		.value("Mode_None", Actuator::Mode_None)
		.value("Mode_Cur", Actuator::Mode_Cur)
		.value("Mode_Vel", Actuator::Mode_Vel)
		.value("Mode_Pos", Actuator::Mode_Pos)
		.value("Mode_Teaching", Actuator::Mode_Teaching)
		.value("Mode_Profile_Pos", Actuator::Mode_Profile_Pos)
		.value("Mode_Profile_Vel", Actuator::Mode_Profile_Vel)
		.value("Mode_Homing", Actuator::Mode_Homing)
		.export_values();

;

	// Actuator::ActuatorAttribute file: line:172
	pybind11::enum_<Actuator::ActuatorAttribute>(M("Actuator"), "ActuatorAttribute", pybind11::arithmetic(), "执行器属性，标识了执行器所有相关属性：")
		.value("CUR_IQ_SETTING", Actuator::CUR_IQ_SETTING)
		.value("CUR_PROPORTIONAL", Actuator::CUR_PROPORTIONAL)
		.value("CUR_INTEGRAL", Actuator::CUR_INTEGRAL)
		.value("CUR_ID_SETTING", Actuator::CUR_ID_SETTING)
		.value("CUR_MINIMUM", Actuator::CUR_MINIMUM)
		.value("CUR_MAXIMUM", Actuator::CUR_MAXIMUM)
		.value("CUR_NOMINAL", Actuator::CUR_NOMINAL)
		.value("CUR_OUTPUT", Actuator::CUR_OUTPUT)
		.value("CUR_MAXSPEED", Actuator::CUR_MAXSPEED)
		.value("ACTUAL_CURRENT", Actuator::ACTUAL_CURRENT)
		.value("VEL_SETTING", Actuator::VEL_SETTING)
		.value("VEL_PROPORTIONAL", Actuator::VEL_PROPORTIONAL)
		.value("VEL_INTEGRAL", Actuator::VEL_INTEGRAL)
		.value("VEL_OUTPUT_LIMITATION_MINIMUM", Actuator::VEL_OUTPUT_LIMITATION_MINIMUM)
		.value("VEL_OUTPUT_LIMITATION_MAXIMUM", Actuator::VEL_OUTPUT_LIMITATION_MAXIMUM)
		.value("ACTUAL_VELOCITY", Actuator::ACTUAL_VELOCITY)
		.value("POS_SETTING", Actuator::POS_SETTING)
		.value("POS_PROPORTIONAL", Actuator::POS_PROPORTIONAL)
		.value("POS_INTEGRAL", Actuator::POS_INTEGRAL)
		.value("POS_DIFFERENTIAL", Actuator::POS_DIFFERENTIAL)
		.value("POS_OUTPUT_LIMITATION_MINIMUM", Actuator::POS_OUTPUT_LIMITATION_MINIMUM)
		.value("POS_OUTPUT_LIMITATION_MAXIMUM", Actuator::POS_OUTPUT_LIMITATION_MAXIMUM)
		.value("POS_LIMITATION_MINIMUM", Actuator::POS_LIMITATION_MINIMUM)
		.value("POS_LIMITATION_MAXIMUM", Actuator::POS_LIMITATION_MAXIMUM)
		.value("HOMING_POSITION", Actuator::HOMING_POSITION)
		.value("ACTUAL_POSITION", Actuator::ACTUAL_POSITION)
		.value("PROFILE_POS_MAX_SPEED", Actuator::PROFILE_POS_MAX_SPEED)
		.value("PROFILE_POS_ACC", Actuator::PROFILE_POS_ACC)
		.value("PROFILE_POS_DEC", Actuator::PROFILE_POS_DEC)
		.value("PROFILE_VEL_MAX_SPEED", Actuator::PROFILE_VEL_MAX_SPEED)
		.value("PROFILE_VEL_ACC", Actuator::PROFILE_VEL_ACC)
		.value("PROFILE_VEL_DEC", Actuator::PROFILE_VEL_DEC)
		.value("CHART_FREQUENCY", Actuator::CHART_FREQUENCY)
		.value("CHART_THRESHOLD", Actuator::CHART_THRESHOLD)
		.value("CHART_SWITCH", Actuator::CHART_SWITCH)
		.value("POS_OFFSET", Actuator::POS_OFFSET)
		.value("VOLTAGE", Actuator::VOLTAGE)
		.value("POS_LIMITATION_SWITCH", Actuator::POS_LIMITATION_SWITCH)
		.value("HOMING_CUR_MAXIMUM", Actuator::HOMING_CUR_MAXIMUM)
		.value("HOMING_CUR_MINIMUM", Actuator::HOMING_CUR_MINIMUM)
		.value("CURRENT_SCALE", Actuator::CURRENT_SCALE)
		.value("VELOCITY_SCALE", Actuator::VELOCITY_SCALE)
		.value("FILTER_C_STATUS", Actuator::FILTER_C_STATUS)
		.value("FILTER_C_VALUE", Actuator::FILTER_C_VALUE)
		.value("FILTER_V_STATUS", Actuator::FILTER_V_STATUS)
		.value("FILTER_V_VALUE", Actuator::FILTER_V_VALUE)
		.value("FILTER_P_STATUS", Actuator::FILTER_P_STATUS)
		.value("FILTER_P_VALUE", Actuator::FILTER_P_VALUE)
		.value("INERTIA", Actuator::INERTIA)
		.value("LOCK_ENERGY", Actuator::LOCK_ENERGY)
		.value("ACTUATOR_TEMPERATURE", Actuator::ACTUATOR_TEMPERATURE)
		.value("INVERTER_TEMPERATURE", Actuator::INVERTER_TEMPERATURE)
		.value("ACTUATOR_PROTECT_TEMPERATURE", Actuator::ACTUATOR_PROTECT_TEMPERATURE)
		.value("ACTUATOR_RECOVERY_TEMPERATURE", Actuator::ACTUATOR_RECOVERY_TEMPERATURE)
		.value("INVERTER_PROTECT_TEMPERATURE", Actuator::INVERTER_PROTECT_TEMPERATURE)
		.value("INVERTER_RECOVERY_TEMPERATURE", Actuator::INVERTER_RECOVERY_TEMPERATURE)
		.value("CALIBRATION_SWITCH", Actuator::CALIBRATION_SWITCH)
		.value("CALIBRATION_ANGLE", Actuator::CALIBRATION_ANGLE)
		.value("ACTUATOR_SWITCH", Actuator::ACTUATOR_SWITCH)
		.value("FIRMWARE_VERSION", Actuator::FIRMWARE_VERSION)
		.value("ONLINE_STATUS", Actuator::ONLINE_STATUS)
		.value("DEVICE_ID", Actuator::DEVICE_ID)
		.value("SN_ID", Actuator::SN_ID)
		.value("MODE_ID", Actuator::MODE_ID)
		.value("ERROR_ID", Actuator::ERROR_ID)
		.value("CUMULATIVE_TIME", Actuator::CUMULATIVE_TIME)
		.value("CURRENT_LIMIT", Actuator::CURRENT_LIMIT)
		.value("VELOCITY_LIMIT", Actuator::VELOCITY_LIMIT)
		.value("ACTUATOR_BRAKE", Actuator::ACTUATOR_BRAKE)
		.value("COMMUNICATION_ID", Actuator::COMMUNICATION_ID)
		.value("INIT_STATE", Actuator::INIT_STATE)
		.value("LOADER_VERSION", Actuator::LOADER_VERSION)
		.value("VERSION_430", Actuator::VERSION_430)
		.value("FRENQUENCY_430", Actuator::FRENQUENCY_430)
		.value("RESERVE_0", Actuator::RESERVE_0)
		.value("RESERVE_1", Actuator::RESERVE_1)
		.value("RESERVE_2", Actuator::RESERVE_2)
		.value("RESERVE_3", Actuator::RESERVE_3)
		.value("RESERVE_4", Actuator::RESERVE_4)
		.value("RESERVE_5", Actuator::RESERVE_5)
		.value("RESERVE_6", Actuator::RESERVE_6)
		.value("RESERVE_7", Actuator::RESERVE_7)
		.value("RESERVE_8", Actuator::RESERVE_8)
		.value("DATA_CNT", Actuator::DATA_CNT)
		.value("DATA_CHART", Actuator::DATA_CHART)
		.value("DATA_INVALID", Actuator::DATA_INVALID)
		.export_values();

;

	// Actuator::Directives file: line:343
	pybind11::enum_<Actuator::Directives>(M("Actuator"), "Directives", pybind11::arithmetic(), "")
		.value("D_HANDSHAKE", Actuator::D_HANDSHAKE)
		.value("D_READ_VERSION", Actuator::D_READ_VERSION)
		.value("D_READ_ADDRESS", Actuator::D_READ_ADDRESS)
		.value("D_READ_CONFIG", Actuator::D_READ_CONFIG)
		.value("D_READ_CUR_CURRENT", Actuator::D_READ_CUR_CURRENT)
		.value("D_READ_CUR_VELOCITY", Actuator::D_READ_CUR_VELOCITY)
		.value("D_READ_CUR_POSITION", Actuator::D_READ_CUR_POSITION)
		.value("D_SET_MODE", Actuator::D_SET_MODE)
		.value("D_SET_CURRENT", Actuator::D_SET_CURRENT)
		.value("D_SET_VELOCITY", Actuator::D_SET_VELOCITY)
		.value("D_SET_POSITION", Actuator::D_SET_POSITION)
		.value("D_SET_PAIRS", Actuator::D_SET_PAIRS)
		.value("D_SET_CURRENT_ID", Actuator::D_SET_CURRENT_ID)
		.value("D_SAVE_PARAM", Actuator::D_SAVE_PARAM)
		.value("D_SET_CURRENT_P", Actuator::D_SET_CURRENT_P)
		.value("D_SET_CURRENT_I", Actuator::D_SET_CURRENT_I)
		.value("D_SET_VELOCITY_P", Actuator::D_SET_VELOCITY_P)
		.value("D_SET_VELOCITY_I", Actuator::D_SET_VELOCITY_I)
		.value("D_SET_POSITION_P", Actuator::D_SET_POSITION_P)
		.value("D_SET_POSITION_I", Actuator::D_SET_POSITION_I)
		.value("D_SET_POSITION_D", Actuator::D_SET_POSITION_D)
		.value("D_READ_CUR_P", Actuator::D_READ_CUR_P)
		.value("D_READ_CUR_I", Actuator::D_READ_CUR_I)
		.value("D_READ_VEL_P", Actuator::D_READ_VEL_P)
		.value("D_READ_VEL_I", Actuator::D_READ_VEL_I)
		.value("D_READ_POS_P", Actuator::D_READ_POS_P)
		.value("D_READ_POS_I", Actuator::D_READ_POS_I)
		.value("D_READ_POS_D", Actuator::D_READ_POS_D)
		.value("D_READ_PROFILE_POS_MAX_SPEED", Actuator::D_READ_PROFILE_POS_MAX_SPEED)
		.value("D_READ_PROFILE_POS_ACC", Actuator::D_READ_PROFILE_POS_ACC)
		.value("D_READ_PROFILE_POS_DEC", Actuator::D_READ_PROFILE_POS_DEC)
		.value("D_SET_PROFILE_POS_MAX_SPEED", Actuator::D_SET_PROFILE_POS_MAX_SPEED)
		.value("D_SET_PROFILE_POS_ACC", Actuator::D_SET_PROFILE_POS_ACC)
		.value("D_SET_PROFILE_POS_DEC", Actuator::D_SET_PROFILE_POS_DEC)
		.value("D_READ_PROFILE_VEL_MAX_SPEED", Actuator::D_READ_PROFILE_VEL_MAX_SPEED)
		.value("D_READ_PROFILE_VEL_ACC", Actuator::D_READ_PROFILE_VEL_ACC)
		.value("D_READ_PROFILE_VEL_DEC", Actuator::D_READ_PROFILE_VEL_DEC)
		.value("D_SET_PROFILE_VEL_MAX_SPEED", Actuator::D_SET_PROFILE_VEL_MAX_SPEED)
		.value("D_SET_PROFILE_VEL_ACC", Actuator::D_SET_PROFILE_VEL_ACC)
		.value("D_SET_PROFILE_VEL_DEC", Actuator::D_SET_PROFILE_VEL_DEC)
		.value("D_READ_CURRENT_MAXSPEED", Actuator::D_READ_CURRENT_MAXSPEED)
		.value("D_SET_CURRENT_MAXSPEED", Actuator::D_SET_CURRENT_MAXSPEED)
		.value("D_SET_SWITCH_MOTORS", Actuator::D_SET_SWITCH_MOTORS)
		.value("D_READ_MOTORS_SWITCH", Actuator::D_READ_MOTORS_SWITCH)
		.value("D_SET_MOTOR_MAC", Actuator::D_SET_MOTOR_MAC)
		.value("D_SET_CURRENT_PID_MIN", Actuator::D_SET_CURRENT_PID_MIN)
		.value("D_SET_CURRENT_PID_MAX", Actuator::D_SET_CURRENT_PID_MAX)
		.value("D_SET_VELOCITY_PID_MIN", Actuator::D_SET_VELOCITY_PID_MIN)
		.value("D_SET_VELOCITY_PID_MAX", Actuator::D_SET_VELOCITY_PID_MAX)
		.value("D_SET_POSITION_PID_MIN", Actuator::D_SET_POSITION_PID_MIN)
		.value("D_SET_POSITION_PID_MAX", Actuator::D_SET_POSITION_PID_MAX)
		.value("D_READ_CURRENT_PID_MIN", Actuator::D_READ_CURRENT_PID_MIN)
		.value("D_READ_CURRENT_PID_MAX", Actuator::D_READ_CURRENT_PID_MAX)
		.value("D_READ_VELOCITY_PID_MIN", Actuator::D_READ_VELOCITY_PID_MIN)
		.value("D_READ_VELOCITY_PID_MAX", Actuator::D_READ_VELOCITY_PID_MAX)
		.value("D_READ_POSITION_PID_MIN", Actuator::D_READ_POSITION_PID_MIN)
		.value("D_READ_POSITION_PID_MAX", Actuator::D_READ_POSITION_PID_MAX)
		.value("D_READ_CHANNEL_2", Actuator::D_READ_CHANNEL_2)
		.value("D_READ_CHANNEL_3", Actuator::D_READ_CHANNEL_3)
		.value("D_READ_CHANNEL_4", Actuator::D_READ_CHANNEL_4)
		.value("D_SET_DEVICE_ID", Actuator::D_SET_DEVICE_ID)
		.value("D_SOFTWARE_CLOSE", Actuator::D_SOFTWARE_CLOSE)
		.value("D_SET_CHART_THRESHOLD", Actuator::D_SET_CHART_THRESHOLD)
		.value("D_SET_CHART_FREQUENCY", Actuator::D_SET_CHART_FREQUENCY)
		.value("D_READ_CHART_THRESHOLD", Actuator::D_READ_CHART_THRESHOLD)
		.value("D_READ_CHART_FREQUENCY", Actuator::D_READ_CHART_FREQUENCY)
		.value("D_CHART_DATA_STATR", Actuator::D_CHART_DATA_STATR)
		.value("D_CAN_CONNECT", Actuator::D_CAN_CONNECT)
		.value("D_READ_VOLTAGE", Actuator::D_READ_VOLTAGE)
		.value("D_CHART_OPEN", Actuator::D_CHART_OPEN)
		.value("D_CHART_CLOSE", Actuator::D_CHART_CLOSE)
		.value("D_CHANNEL2_OPEN", Actuator::D_CHANNEL2_OPEN)
		.value("D_CHANNEL2_CLOSE", Actuator::D_CHANNEL2_CLOSE)
		.value("D_CHANNEL3_OPEN", Actuator::D_CHANNEL3_OPEN)
		.value("D_CHANNEL3_CLOSE", Actuator::D_CHANNEL3_CLOSE)
		.value("D_CHANNEL4_OPEN", Actuator::D_CHANNEL4_OPEN)
		.value("D_CHANNEL4_CLOSE", Actuator::D_CHANNEL4_CLOSE)
		.value("D_READ_CHANNEL_1", Actuator::D_READ_CHANNEL_1)
		.value("D_SET_VOLTAGE", Actuator::D_SET_VOLTAGE)
		.value("D_CRC_ERROR", Actuator::D_CRC_ERROR)
		.value("D_CHANNEL1_OPEN", Actuator::D_CHANNEL1_OPEN)
		.value("D_CHANNEL1_CLOSE", Actuator::D_CHANNEL1_CLOSE)
		.value("D_READ_CURRENT_SCALE", Actuator::D_READ_CURRENT_SCALE)
		.value("D_SET_CUR_TRIGGER_MODE", Actuator::D_SET_CUR_TRIGGER_MODE)
		.value("D_READ_MOTOR_MODE", Actuator::D_READ_MOTOR_MODE)
		.value("D_READ_CURRENT_LIMIT", Actuator::D_READ_CURRENT_LIMIT)
		.value("D_SET_CURRENT_LIMIT", Actuator::D_SET_CURRENT_LIMIT)
		.value("D_READ_VELOCITY_LIMIT", Actuator::D_READ_VELOCITY_LIMIT)
		.value("D_SET_VELOCITY_LIMIT", Actuator::D_SET_VELOCITY_LIMIT)
		.value("D_READ_TEMP_MOTOR", Actuator::D_READ_TEMP_MOTOR)
		.value("D_READ_TEMP_INVERTER", Actuator::D_READ_TEMP_INVERTER)
		.value("D_SET_TEMP_PROTECT", Actuator::D_SET_TEMP_PROTECT)
		.value("D_READ_TEMP_PROTECT", Actuator::D_READ_TEMP_PROTECT)
		.value("D_SET_TEMP_RECOVERY", Actuator::D_SET_TEMP_RECOVERY)
		.value("D_READ_TEMP_RECOVERY", Actuator::D_READ_TEMP_RECOVERY)
		.value("D_READ_CUMULATIVE_TIME", Actuator::D_READ_CUMULATIVE_TIME)
		.value("D_SET_INVERTER_TEMP_PROTECT", Actuator::D_SET_INVERTER_TEMP_PROTECT)
		.value("D_READ_INVERTER_TEMP_PROTECT", Actuator::D_READ_INVERTER_TEMP_PROTECT)
		.value("D_SET_INVERTER_TEMP_RECOVERY", Actuator::D_SET_INVERTER_TEMP_RECOVERY)
		.value("D_READ_INVERTER_TEMP_RECOVERY", Actuator::D_READ_INVERTER_TEMP_RECOVERY)
		.value("D_SET_FILTER_C_STATUS", Actuator::D_SET_FILTER_C_STATUS)
		.value("D_READ_FILTER_C_STATUS", Actuator::D_READ_FILTER_C_STATUS)
		.value("D_SET_FILTER_C_VALUE", Actuator::D_SET_FILTER_C_VALUE)
		.value("D_READ_FILTER_C_VALUE", Actuator::D_READ_FILTER_C_VALUE)
		.value("D_SET_FILTER_V_STATUS", Actuator::D_SET_FILTER_V_STATUS)
		.value("D_READ_FILTER_V_STATUS", Actuator::D_READ_FILTER_V_STATUS)
		.value("D_SET_FILTER_V_VALUE", Actuator::D_SET_FILTER_V_VALUE)
		.value("D_READ_FILTER_V_VALUE", Actuator::D_READ_FILTER_V_VALUE)
		.value("D_SET_FILTER_P_STATUS", Actuator::D_SET_FILTER_P_STATUS)
		.value("D_READ_FILTER_P_STATUS", Actuator::D_READ_FILTER_P_STATUS)
		.value("D_SET_FILTER_P_VALUE", Actuator::D_SET_FILTER_P_VALUE)
		.value("D_READ_FILTER_P_VALUE", Actuator::D_READ_FILTER_P_VALUE)
		.value("D_SET_INERTIA", Actuator::D_SET_INERTIA)
		.value("D_READ_INERTIA", Actuator::D_READ_INERTIA)
		.value("D_SET_LOCK_ENERGY", Actuator::D_SET_LOCK_ENERGY)
		.value("D_READ_LOCK_ENERGY", Actuator::D_READ_LOCK_ENERGY)
		.value("D_SET_MAX_POS", Actuator::D_SET_MAX_POS)
		.value("D_SET_MIN_POS", Actuator::D_SET_MIN_POS)
		.value("D_READ_MAX_POS", Actuator::D_READ_MAX_POS)
		.value("D_READ_MIN_POS", Actuator::D_READ_MIN_POS)
		.value("D_SET_HOMING_POS", Actuator::D_SET_HOMING_POS)
		.value("D_CLEAR_HOMING", Actuator::D_CLEAR_HOMING)
		.value("D_SET_POS_OFFSET", Actuator::D_SET_POS_OFFSET)
		.value("D_READ_POS_OFFSET", Actuator::D_READ_POS_OFFSET)
		.value("D_READ_HOMING_LIMIT", Actuator::D_READ_HOMING_LIMIT)
		.value("D_SET_HOMING_LIMIT", Actuator::D_SET_HOMING_LIMIT)
		.value("D_SET_HOMING_OPERATION", Actuator::D_SET_HOMING_OPERATION)
		.value("D_SET_HOMING_MIN", Actuator::D_SET_HOMING_MIN)
		.value("D_SET_HOMING_MAX", Actuator::D_SET_HOMING_MAX)
		.value("D_SET_HOMING_CUR_MIN", Actuator::D_SET_HOMING_CUR_MIN)
		.value("D_SET_HOMING_CUR_MAX", Actuator::D_SET_HOMING_CUR_MAX)
		.value("D_READ_HOMING_CUR_MIN", Actuator::D_READ_HOMING_CUR_MIN)
		.value("D_READ_HOMING_CUR_MAX", Actuator::D_READ_HOMING_CUR_MAX)
		.value("D_READ_ACTUAL_CVP", Actuator::D_READ_ACTUAL_CVP)
		.value("D_READ_LOADER_VERSION", Actuator::D_READ_LOADER_VERSION)
		.value("D_SWITCH_CALIBRATION", Actuator::D_SWITCH_CALIBRATION)
		.value("D_READ_CALIBRATION_SWITCH", Actuator::D_READ_CALIBRATION_SWITCH)
		.value("D_START_CALIBRATION", Actuator::D_START_CALIBRATION)
		.value("D_SET_CALIBRATION_ANGLE", Actuator::D_SET_CALIBRATION_ANGLE)
		.value("D_READ_CALIBRATION_ANGLE", Actuator::D_READ_CALIBRATION_ANGLE)
		.value("D_SWITCH_CALIBRATION_VEL", Actuator::D_SWITCH_CALIBRATION_VEL)
		.value("D_SET_430_FREQUENCY", Actuator::D_SET_430_FREQUENCY)
		.value("D_READ_430_FREQUENCY", Actuator::D_READ_430_FREQUENCY)
		.value("D_SET_ACTUATOR_BRAKE", Actuator::D_SET_ACTUATOR_BRAKE)
		.value("D_READ_ACTUATOR_BRAKE", Actuator::D_READ_ACTUATOR_BRAKE)
		.value("D_INIT_430", Actuator::D_INIT_430)
		.value("D_READ_430_VERSION", Actuator::D_READ_430_VERSION)
		.value("D_READ_RESERVE_0", Actuator::D_READ_RESERVE_0)
		.value("D_READ_RESERVE_1", Actuator::D_READ_RESERVE_1)
		.value("D_READ_RESERVE_2", Actuator::D_READ_RESERVE_2)
		.value("D_READ_RESERVE_3", Actuator::D_READ_RESERVE_3)
		.value("D_READ_RESERVE_4", Actuator::D_READ_RESERVE_4)
		.value("D_READ_RESERVE_5", Actuator::D_READ_RESERVE_5)
		.value("D_READ_RESERVE_6", Actuator::D_READ_RESERVE_6)
		.value("D_READ_RESERVE_7", Actuator::D_READ_RESERVE_7)
		.value("D_READ_RESERVE_8", Actuator::D_READ_RESERVE_8)
		.value("D_READ_LAST_STATE", Actuator::D_READ_LAST_STATE)
		.value("D_IP_BROADCAST", Actuator::D_IP_BROADCAST)
		.value("D_TMP_COMMAND", Actuator::D_TMP_COMMAND)
		.value("D_CLEAR_ERROR", Actuator::D_CLEAR_ERROR)
		.value("D_CHECK_ERROR", Actuator::D_CHECK_ERROR)
		.value("DIRECTIVES_INVALID", Actuator::DIRECTIVES_INVALID)
		.export_values();

;

}
