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

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


class PhicubeCommunication
{
    public:
        
        PhicubeCommunication();

        void Init(PhicubeRobot &robot);
        void UpdateMotorsState();
        void UpdateRobotState();

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

        const uint nMotors = 2;
        const char *names[2] = {"motor_1","motor_2"};

        PhicubeRobot *phicube;
        float pos[2];
        float vel[2];
        float eff[2];

        std_msgs__msg__Empty heartBeatMsg;
        sensor_msgs__msg__JointState motorStateMsg;

        rcl_node_t node;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rclc_executor_t executor;
        rcl_publisher_t heartBeatPublisher;
        rcl_publisher_t motorStatePublisher;
};


#endif