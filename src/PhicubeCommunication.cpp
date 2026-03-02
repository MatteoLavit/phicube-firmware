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

    // mallocs for Motors State msg
    motorsStateMsg.name.capacity = nMotors;
    motorsStateMsg.name.data = (rosidl_runtime_c__String*) malloc(motorsStateMsg.name.capacity * sizeof(rosidl_runtime_c__String));
    motorsStateMsg.name.size = 0;
    motorsStateMsg.position.capacity = motorsStateMsg.name.capacity;
    motorsStateMsg.position.data = (double*) malloc(motorsStateMsg.position.capacity * sizeof(double));
    motorsStateMsg.position.size = 0;
    motorsStateMsg.velocity.capacity = motorsStateMsg.name.capacity;
    motorsStateMsg.velocity.data = (double*) malloc(motorsStateMsg.velocity.capacity * sizeof(double));
    motorsStateMsg.velocity.size = 0;
    motorsStateMsg.torque.capacity = motorsStateMsg.name.capacity;
    motorsStateMsg.torque.data = (double*) malloc(motorsStateMsg.torque.capacity * sizeof(double));
    motorsStateMsg.torque.size = 0;
    motorsStateMsg.current.capacity = motorsStateMsg.name.capacity;
    motorsStateMsg.current.data = (double*) malloc(motorsStateMsg.current.capacity * sizeof(double));
    motorsStateMsg.current.size = 0;
    for (uint ii = 0; ii < motorsStateMsg.name.capacity; ii++) {
        motorsStateMsg.name.data[ii].capacity = 10;
        motorsStateMsg.name.data[ii].data = (char*) malloc(motorsStateMsg.name.data[ii].capacity * sizeof(char));
        motorsStateMsg.name.data[ii].size = 0;
        strcpy(motorsStateMsg.name.data[ii].data, names[ii]);
        motorsStateMsg.name.data[ii].size = strlen(motorsStateMsg.name.data[ii].data);
        motorsStateMsg.position.data[ii] = 0.0;
        motorsStateMsg.velocity.data[ii] = 0.0;
        motorsStateMsg.torque.data[ii] = 0.0;
        motorsStateMsg.current.data[ii] = 0.0;
        motorsStateMsg.name.size++;
        motorsStateMsg.position.size++;
        motorsStateMsg.velocity.size++;
        motorsStateMsg.torque.size++;
        motorsStateMsg.current.size++;
    }

    // mallocs for Robot State msg
    robotStateMsg.sn.capacity = 20;
    robotStateMsg.sn.data = (char*) malloc(robotStateMsg.sn.capacity * sizeof(char));
    robotStateMsg.sn.size = 0;
    strcpy(robotStateMsg.sn.data, PHICUBE_SERIAL_NUMBER);
    robotStateMsg.sn.size = strlen(robotStateMsg.sn.data);
}


void PhicubeCommunication::Init(PhicubeRobot &phicubeRobot)
{
    set_microros_serial_transports(Serial);
    robot = &phicubeRobot;
}


void PhicubeCommunication::UpdateROSComm()
{
    switch (agentState)
    {
        case WAITING_AGENT:
            agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        break;
        case AGENT_AVAILABLE:
            agentState = (true == CreateEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (agentState == WAITING_AGENT) {
                DestroyEntities();
            }
        break;
        case AGENT_CONNECTED:
            agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (agentState == AGENT_CONNECTED) {
                if (robot->IsSystemReady()) RunEntities();
            }
        break;
        case AGENT_DISCONNECTED:
            DestroyEntities();
            agentState = WAITING_AGENT;
        break;
        default:
        break;
    }
}


bool PhicubeCommunication::CreateEntities()
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, PHICUBE_NODE_NAME, "", &support));
    RCCHECK(rclc_executor_init(&executor, &support.context, 16, &allocator));

    RCCHECK(rclc_publisher_init_default(&robotStatePublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(phicube_msgs, msg, RobotState), "/phicube/robot_state"));
    RCCHECK(rclc_publisher_init_default(&motorsStatePublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(phicube_msgs, msg, MotorsState), "/phicube/motors_state"));

    RCCHECK(rclc_service_init_default(&motorsHomeSrv, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, MotorsHome), "/phicube/motors_home"));
    RCCHECK(rclc_executor_add_service_with_context(&executor, &motorsHomeSrv, &motorsHomeReq, &motorsHomeRes, &PhicubeCommunication::MotorsHomeCB, this));
    RCCHECK(rclc_service_init_default(&motorsOffSrv, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, MotorsOff), "/phicube/motors_off"));
    RCCHECK(rclc_executor_add_service_with_context(&executor, &motorsOffSrv, &motorsOffReq, &motorsOffRes, &PhicubeCommunication::MotorsOffCB, this));

    return true;
}


void PhicubeCommunication::DestroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&robotStatePublisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&motorsStatePublisher, &node));

    RCSOFTCHECK(rcl_service_fini(&motorsHomeSrv, &node));
    RCSOFTCHECK(rcl_service_fini(&motorsOffSrv, &node));

    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}


void PhicubeCommunication::RunEntities()
{
    robotStateCounter++;
    if (robotStateCounter >= PHICUBE_ROBOTSTATE_PERIOD_MS/PHICUBE_ROSCOMM_PERIOD_MS) {
        struct timespec ts;
        ClockGetTime(CLOCK_REALTIME, &ts);
        robot->GetMotorsInfo(inf);
        robotStateMsg.timestamp.sec = ts.tv_sec;
        robotStateMsg.timestamp.nanosec = ts.tv_nsec;
        robotStateMsg.ready = robot->IsSystemReady();
        robotStateMsg.info_m1 = inf[0];
        robotStateMsg.info_m2 = inf[1];
        RCSOFTCHECK(rcl_publish(&robotStatePublisher, &robotStateMsg, NULL));
        robotStateCounter = 0;
    }

    motorsStateCounter++;
    if (motorsStateCounter >= PHICUBE_MOTORSSTATE_PERIOD_MS/PHICUBE_ROSCOMM_PERIOD_MS) {
        struct timespec ts;
        ClockGetTime(CLOCK_REALTIME, &ts);
        robot->GetMotorsPosition(pos);
        robot->GetMotorsVelocity(vel);
        robot->GetMotorsTorque(trq);
        robot->GetMotorsCurrent(cur);
        motorsStateMsg.timestamp.sec = ts.tv_sec;
        motorsStateMsg.timestamp.nanosec = ts.tv_nsec;
        for (uint ii = 0; ii < motorsStateMsg.name.capacity; ii++) {
            motorsStateMsg.position.data[ii] = pos[ii];
            motorsStateMsg.velocity.data[ii] = vel[ii];
            motorsStateMsg.torque.data[ii] = trq[ii];
            motorsStateMsg.current.data[ii] = cur[ii];
        }
        RCSOFTCHECK(rcl_publish(&motorsStatePublisher, &motorsStateMsg, NULL));
        motorsStateCounter = 0;
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, 1000));
}


void PhicubeCommunication::MotorsHomeCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__MotorsHome_Request * req = (phicube_msgs__srv__MotorsHome_Request *) reqMsg;
    phicube_msgs__srv__MotorsHome_Response * res = (phicube_msgs__srv__MotorsHome_Response *) resMsg;
    PhicubeCommunication * phicubeContext = (PhicubeCommunication *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    if (phicubeContext->robot->MotorsHome(req->home_m1, req->home_m2)) {
        if (req->home_m1 && req->home_m2) strcpy(res->message.data, "M1 and M2 going back home by shortest path.");
        else if (req->home_m1) strcpy(res->message.data, "M1 going back home by shortest path.");
        else if (req->home_m2) strcpy(res->message.data, "M2 going back home by shortest path.");
        else strcpy(res->message.data, "No action as requested.");
        res->message.size = strlen(res->message.data);
    }
    else {
        strcpy(res->message.data, "It was not possible to execute the request.");
        res->message.size = strlen(res->message.data);
    }
    res->success = true;
}


void PhicubeCommunication::MotorsOffCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__MotorsOff_Request * req = (phicube_msgs__srv__MotorsOff_Request *) reqMsg;
    phicube_msgs__srv__MotorsOff_Response * res = (phicube_msgs__srv__MotorsOff_Response *) resMsg;
    PhicubeCommunication * phicubeContext = (PhicubeCommunication *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    if (phicubeContext->robot->MotorsOff(req->off_m1, req->off_m2)) {
        if (req->off_m1 && req->off_m2) strcpy(res->message.data, "M1 and M2 turned off.");
        else if (req->off_m1) strcpy(res->message.data, "M1 turned off.");
        else if (req->off_m2) strcpy(res->message.data, "M2 turned off.");
        else strcpy(res->message.data, "No action as requested.");
        res->message.size = strlen(res->message.data);
    }
    else {
        strcpy(res->message.data, "It was not possible to execute the request.");
        res->message.size = strlen(res->message.data);
    }
    res->success = true;
}