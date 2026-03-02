/**
 * @file PhicubeCommunication.h
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief Class used to manage all ROS2 communications between the firmware and the rest of the system.
 * @version 0.1
 * @date 2026-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PHICUBE_COMMUNICATION
#define PHICUBE_COMMUNICATION

#include "PhicubeSN.h"
#include "PhicubeRobot.h"
#include "PhicubeConfiguration.h"
#include "TimeStampConverter.h"

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>

#include <phicube_msgs/msg/motors_state.h>
#include <phicube_msgs/msg/robot_state.h>
#include <phicube_msgs/msg/gim6010_info.h>
#include <phicube_msgs/srv/motors_home.h>
#include <phicube_msgs/srv/motors_off.h>

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


class PhicubeCommunication
{
    public:
        
        PhicubeCommunication();

        void Init(PhicubeRobot &phicubeRobot);
        void UpdateROSComm();

    private:

        enum agentState {
            WAITING_AGENT,
            AGENT_AVAILABLE,
            AGENT_CONNECTED,
            AGENT_DISCONNECTED
        } agentState;

        bool CheckAgent();
        bool CreateEntities();
        void DestroyEntities();
        void RunEntities();

        static void MotorsHomeCB(const void * reqMsg, void * resMsg, void * context);
        static void MotorsOffCB(const void * reqMsg, void * resMsg, void * context);

        const uint nMotors = 2;
        const char *names[2] = {"motor_1","motor_2"};
        PhicubeRobot *robot;
        phicube_msgs__msg__GIM6010Info inf[2];
        float pos[2];
        float vel[2];
        float trq[2];
        float cur[2];

        rcl_node_t node;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rclc_executor_t executor;

        phicube_msgs__srv__MotorsHome_Request motorsHomeReq;
        phicube_msgs__srv__MotorsHome_Response motorsHomeRes;
        rcl_service_t motorsHomeSrv;
        phicube_msgs__srv__MotorsOff_Request motorsOffReq;
        phicube_msgs__srv__MotorsOff_Response motorsOffRes;
        rcl_service_t motorsOffSrv;

        uint robotStateCounter;
        phicube_msgs__msg__RobotState robotStateMsg;
        rcl_publisher_t robotStatePublisher;
        uint motorsStateCounter;
        phicube_msgs__msg__MotorsState motorsStateMsg;
        rcl_publisher_t motorsStatePublisher;        
};


#endif