/**
 * @file PhicubeRosComm.h
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PHICUBE_ROS_COMM
#define PHICUBE_ROS_COMM

#include "Phicube.h"
#include "PhicubeConfiguration.h"
#include "PhicubeCalibration.h"
#include "TimeStampConverter.h"

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <phicube_msgs/msg/cmd_state.h>
#include <phicube_msgs/msg/robot_diagnostics.h>
#include <phicube_msgs/msg/motor_diagnostics.h>
#include <phicube_msgs/msg/hardware_config.h>
#include <phicube_msgs/msg/control_config.h>
#include <phicube_msgs/srv/set_homing.h>
#include <phicube_msgs/srv/set_pid_params.h>
#include <phicube_msgs/srv/set_led_state.h>
#include <phicube_msgs/srv/set_controller.h>
#include <phicube_msgs/srv/set_configuration.h>
#include <phicube_msgs/srv/set_decoupled_params.h>
#include <phicube_msgs/srv/set_coupled_params.h>

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


class PhicubeRosComm
{
    public:
        
        /**
         * @brief Construct a new object for ROS2 communication
         * 
         */
        PhicubeRosComm();

        /**
         * @brief Initialization of the communication object.
         * 
         * @param cube The phicube object that will communicate with the upper levels.
         */
        void Init(Phicube &cube);

        /**
         * @brief Method called for every iteration of the thread responsible for sending and receiving control messages.
         * 
         */
        void Update();

        /**
         * @brief Method called at a lower frequency responsible for providing status updates to the upper levels.
         * 
         */
        void UpdateStatus();

    private:

        /**
         * @brief Method called at startup to instantiate publishers, subscribers and service servers.
         * 
         * @return true 
         * @return false 
         */
        bool CreateEntities();

        /**
         * @brief Method called at every Update() to let the ROS2 entities run.
         * 
         */
        void RunEntities();

        /**
         * @brief Method called at disconnection to clean the communication pipes.
         * 
         */
        void DestroyEntities();

        /**
         * @brief The possible states for the ROS2 agent.
         * 
         */
        enum agentState {
            WAITING_AGENT,
            AGENT_AVAILABLE,
            AGENT_CONNECTED,
            AGENT_DISCONNECTED
        } agentState;

        /**
         * @brief Callback receiving command messages from the upper levels.
         * 
         * @param msgIn The received message.
         * @param context The PhicubeRosComm self object.
         */
        static void CmdStateCB(const void * msgIn, void * context);

        /**
         * @brief Callback receiving the homing initialization request.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void InitCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the new parameters for the velocity estimator related to the assistive part of the coupled/decoupled controllers.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetAssRefVelEstPIDParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the new parameters for the velocity estimator related to the bilateral part of the decoupled controllers.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetBimRefVelEstPIDParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the new parameters for the estimator computing the motors velocity.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetVelEstPIDParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the PID parameters for the position controller.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetPosPIDParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the PID parameters for the velocity controller.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetVelPIDParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the requested state for one of the led rings mounted on the device.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetLedStateCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the name of the handles mounted on the device from the upper level.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetConfigurationCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the index of the controller that needs to be activated.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetControllerCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the new parameters requested for the decoupled controller.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetDecoupledParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Callback receiving the new parameters requested for the coupled controller.
         * 
         * @param reqMsg The message containing the request.
         * @param resMsg The message containing the response.
         * @param context The PhicubeRosComm self object.
         */
        static void SetCoupledParamsCB(const void * reqMsg, void * resMsg, void * context);

        /**
         * @brief Variable storing the number of motors installed on the device.
         * 
         */
        const uint nMotors = 2;

        /**
         * @brief Array storing the names used to indentify the motors in the JointState message.
         * 
         */
        const char *names[2] = {"motor_1","motor_2"};

        /**
         * @brief Array storing the current position of the motors.
         * 
         */
        float pos[2];

        /**
         * @brief Array storing the current velocity of the motors.
         * 
         */
        float vel[2];

        /**
         * @brief Array storing the value of absorbed current for each motor.
         * 
         */
        float eff[2];

        /**
         * @brief Array storing the value of overheat index for each motor.
         * 
         */
        float overheat_index[2];

        /**
         * @brief Array storing the value of encoders (abs VS inc) difference for each motor.
         * 
         */
        float encoders_diff[2];

        /**
         * @brief Variable that is set to true every time the device status messages need to be republished.
         * 
         */
        bool needStatusUpdate;

        /**
         * @brief The Phicube object.
         * 
         */
        Phicube *phicube;

        /**
         * @brief An empty message used as hearbeat for the devices communication.
         * 
         */
        std_msgs__msg__Empty heartBeatMsg;

        /**
         * @brief A JointState message carrying the information about current state of the motors.
         * 
         */
        sensor_msgs__msg__JointState motorStateMsg;
        
        /**
         * @brief A Diagnostic message carrying the information about any detected error.
         * 
         */
        phicube_msgs__msg__RobotDiagnostics diagnosticMsg;

        /**
         * @brief A HardwareConfig message carrying the information about the current hardware configuration of the device.
         * 
         */
        phicube_msgs__msg__HardwareConfig hardwareConfigMsg;

        /**
         * @brief A ControlConfig message carrying the information about the current motor control configuration of the device.
         * 
         */
        phicube_msgs__msg__ControlConfig controlConfigMsg;

        /**
         * @brief A CmdState message containing the command to be executed by the motors.
         * 
         */
        phicube_msgs__msg__CmdState cmdState;

        /**
         * @brief A message containing the request for the homing procedure.
         * 
         */
        phicube_msgs__srv__SetHoming_Request initRequest;

        /**
         * @brief A message containing the response for the homing procedure request.
         * 
         */
        phicube_msgs__srv__SetHoming_Response initResponse;

        /**
         * @brief A message containing the request to set the led state.
         * 
         */
        phicube_msgs__srv__SetLedState_Request setLedStateRequest;

        /**
         * @brief A message containing the response for the led state request.
         * 
         */
        phicube_msgs__srv__SetLedState_Response setLedStateResponse;

        /**
         * @brief A message containing the request to set the hardware configuration.
         * 
         */
        phicube_msgs__srv__SetConfiguration_Request setConfigurationRequest;

        /**
         * @brief A message containing the response for the hardware configuration request.
         * 
         */
        phicube_msgs__srv__SetConfiguration_Response setConfigurationResponse;

        /**
         * @brief A message containing the request to activate a certain controller.
         * 
         */
        phicube_msgs__srv__SetController_Request setControllerRequest;

        /**
         * @brief A message containing the response to the controller activation request.
         * 
         */
        phicube_msgs__srv__SetController_Response setControllerResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the decoupled controller.
         * 
         */
        phicube_msgs__srv__SetDecoupledParams_Request setDecoupledParamsRequest;

        /**
         * @brief A message containing the response to the decoupled controller parameters request.
         * 
         */
        phicube_msgs__srv__SetDecoupledParams_Response setDecoupledParamsResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the coupled controller.
         * 
         */
        phicube_msgs__srv__SetCoupledParams_Request setCoupledParamsRequest;

        /**
         * @brief A message containing the response to the coupled controller parameters request.
         * 
         */
        phicube_msgs__srv__SetCoupledParams_Response setCoupledParamsResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the assistance velocity estimator.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Request setAssRefVelEstPIDParamsRequest;

        /**
         * @brief A message containing the response to the assistance velocity estimator request.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Response setAssRefVelEstPIDParamsResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the bimanual velocity estimator.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Request setBimRefVelEstPIDParamsRequest;

        /**
         * @brief A message containing the response to the bimanual velocity estimator request.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Response setBimRefVelEstPIDParamsResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the motors velocity estimator.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Request setVelEstPIDParamsRequest;

        /**
         * @brief A message containing the response to the motors velocity estimator request.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Response setVelEstPIDParamsResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the position controller.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Request setPosPIDParamsRequest;

        /**
         * @brief A message containing the response to the position controller request.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Response setPosPIDParamsResponse;

        /**
         * @brief A message containing the request for setting the new parameters of the velocity controller.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Request setVelPIDParamsRequest;

        /**
         * @brief A message containing the response to the velocity controller request.
         * 
         */
        phicube_msgs__srv__SetPIDParams_Response setVelPIDParamsResponse;

        /**
         * @brief The ROS2 microros node running on the microcontroller.
         * 
         */
        rcl_node_t node;

        /**
         * @brief The ROS2 microros support object.
         * 
         */
        rclc_support_t support;

        /**
         * @brief The ROS2 microros allocator object.
         * 
         */
        rcl_allocator_t allocator;

        /**
         * @brief The ROS2 microros executor object.
         * 
         */
        rclc_executor_t executor;

        /**
         * @brief The ROS2 microros publisher for the heartbeat message.
         * 
         */
        rcl_publisher_t heartBeatPublisher;

        /**
         * @brief The ROS2 microros publisher for the motor state message.
         * 
         */
        rcl_publisher_t motorStatePublisher;

        /**
         * @brief The ROS2 microros publisher for the diagnostic message.
         * 
         */

        rcl_publisher_t diagnosticsPublisher;

        /**
         * @brief The ROS2 microros publisher for the hardware config message.
         * 
         */
        rcl_publisher_t hardwareConfigPublisher;

        /**
         * @brief The ROS2 microros publisher for the control message.
         * 
         */
        rcl_publisher_t controlConfigPublisher;

        /**
         * @brief The ROS2 microros subscriber for the command state message.
         * 
         */
        rcl_subscription_t cmdStateSubscriber;

        /**
         * @brief The ROS2 microros server for the homing procedure service.
         * 
         */
        rcl_service_t initServer;

        /**
         * @brief The ROS2 microros server for the assistance velocity estimator service.
         * 
         */
        rcl_service_t setAssRefVelEstPIDParamsServer;

        /**
         * @brief The ROS2 microros server for the bimanual velocity estimator service.
         * 
         */
        rcl_service_t setBimRefVelEstPIDParamsServer;

        /**
         * @brief The ROS2 microros server for the motor velocity estimator service.
         * 
         */
        rcl_service_t setVelEstPIDParamsServer;

        /**
         * @brief The ROS2 microros server for the position controller parameters service.
         * 
         */
        rcl_service_t setPosPIDParamsServer;

        /**
         * @brief The ROS2 microros server for the velocity controller parameters service.
         * 
         */
        rcl_service_t setVelPIDParamsServer;

        /**
         * @brief The ROS2 microros server for the led state service.
         * 
         */
        rcl_service_t setLedStateServer;

        /**
         * @brief The ROS2 microror server for the hardware configuration service.
         * 
         */
        rcl_service_t setConfigurationServer;

        /**
         * @brief The ROS2 microror server for the controller activation service.
         * 
         */
        rcl_service_t setControllerServer;

        /**
         * @brief The ROS2 microror server for the decoupled controller parameters service.
         * 
         */
        rcl_service_t setDecoupledParamsServer;

        /**
         * @brief The ROS2 microror server for the coupled controller parameters service.
         * 
         */
        rcl_service_t setCoupledParamsServer;
};


#endif