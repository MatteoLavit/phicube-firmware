/**
 * @file PhicubeCommunication.cpp
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief Class used to manage all ROS2 communications between the firmware and the rest of the system.
 * @version 0.1
 * @date 2026-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "PhicubeCommunication.h"


PhicubeCommunication::PhicubeCommunication()
{
    agentState = WAITING_AGENT;

    // mallocs for motorState msg
    motorStateMsg.header.frame_id.capacity = 20;
    motorStateMsg.header.frame_id.data = (char*) malloc(motorStateMsg.header.frame_id.capacity * sizeof(char));
    motorStateMsg.header.frame_id.size = 0;
    strcpy(motorStateMsg.header.frame_id.data, PHICUBE_SERIAL_NUMBER);
    motorStateMsg.header.frame_id.size = strlen(motorStateMsg.header.frame_id.data);
    motorStateMsg.name.capacity = nMotors;
    motorStateMsg.name.data = (rosidl_runtime_c__String*) malloc(motorStateMsg.name.capacity * sizeof(rosidl_runtime_c__String));
    motorStateMsg.name.size = 0;
    motorStateMsg.position.capacity = motorStateMsg.name.capacity;
    motorStateMsg.position.data = (double*) malloc(motorStateMsg.position.capacity * sizeof(double));
    motorStateMsg.position.size = 0;
    motorStateMsg.velocity.capacity = motorStateMsg.name.capacity;
    motorStateMsg.velocity.data = (double*) malloc(motorStateMsg.position.capacity * sizeof(double));
    motorStateMsg.velocity.size = 0;
    motorStateMsg.effort.capacity = motorStateMsg.name.capacity;
    motorStateMsg.effort.data = (double*) malloc(motorStateMsg.position.capacity * sizeof(double));
    motorStateMsg.effort.size = 0;
    for (uint ii = 0; ii < motorStateMsg.name.capacity; ii++) {
        motorStateMsg.name.data[ii].capacity = 10;
        motorStateMsg.name.data[ii].data = (char*) malloc(motorStateMsg.name.data[ii].capacity * sizeof(char));
        motorStateMsg.name.data[ii].size = 0;
        strcpy(motorStateMsg.name.data[ii].data, names[ii]);
        motorStateMsg.name.data[ii].size = strlen(motorStateMsg.name.data[ii].data);
        motorStateMsg.position.data[ii] = 0.0;
        motorStateMsg.velocity.data[ii] = 0.0;
        motorStateMsg.effort.data[ii] = 0.0;
        motorStateMsg.name.size++;
        motorStateMsg.position.size++;
        motorStateMsg.velocity.size++;
        motorStateMsg.effort.size++;
    }
}


void PhicubeCommunication::Init(PhicubeRobot &robot)
{
    set_microros_serial_transports(Serial);
    phicube = &robot;
}


bool PhicubeCommunication::CreateEntities()
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, PHICUBE_NODE_NAME, "", &support));
    RCCHECK(rclc_executor_init(&executor, &support.context, 16, &allocator));

    RCCHECK(rclc_publisher_init_default(&heartBeatPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/phicube/heartbeat"));
    RCCHECK(rclc_publisher_init_default(&motorStatePublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/phicube/motor_state"));

    return true;
}


void PhicubeCommunication::DestroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&heartBeatPublisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&motorStatePublisher, &node));

    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}


bool PhicubeCommunication::CheckAgent()
{
    switch (agentState)
    {
        case WAITING_AGENT:
            agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            return false;
        break;
        case AGENT_AVAILABLE:
            agentState = (true == CreateEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (agentState == WAITING_AGENT) {
                DestroyEntities();
            }
            return false;
        break;
        case AGENT_CONNECTED:
            agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (agentState == AGENT_CONNECTED) {
                return true;
            }
            else {
                return false;
            }
        break;
        case AGENT_DISCONNECTED:
            DestroyEntities();
            agentState = WAITING_AGENT;
            return false;
        break;
        default:
            return false;
        break;
    }
}


void PhicubeCommunication::UpdateMotorsState()
{
    if (CheckAgent()) {
        struct timespec ts;
        ClockGetTime(CLOCK_REALTIME, &ts);
        phicube->GetCurrentPosition(pos);
        phicube->GetCurrentVelocity(vel);
        phicube->GetCurrentEffort(eff);
        motorStateMsg.header.stamp.sec = ts.tv_sec;
        motorStateMsg.header.stamp.nanosec = ts.tv_nsec;
        for (uint ii = 0; ii < motorStateMsg.name.capacity; ii++) {
            motorStateMsg.position.data[ii] = pos[ii];
            motorStateMsg.velocity.data[ii] = vel[ii];
            motorStateMsg.effort.data[ii] = eff[ii];
        }

        RCSOFTCHECK(rcl_publish(&motorStatePublisher, &motorStateMsg, NULL));
        RCSOFTCHECK(rclc_executor_spin_some(&executor, 1000));
    }
}


void PhicubeCommunication::UpdateRobotState()
{
    if (CheckAgent()) {
        RCSOFTCHECK(rcl_publish(&heartBeatPublisher, &heartBeatMsg, NULL));
    }
}