/**
 * @file PhicubeRosComm.cpp
 * @author phicube (info@phicube.it)
 * @brief 
 * @version 0.1
 * @date 2022-02-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "PhicubeRosComm.h"


/**
 * @brief Instantiation of messages and link to their corresponding publisher/service
 * 
 */
PhicubeRosComm::PhicubeRosComm()
{
    agentState = WAITING_AGENT;
    needStatusUpdate = false;

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

    //mallocs for hardware config msg
    hardwareConfigMsg.orientation.capacity = 20;
    hardwareConfigMsg.orientation.data = (char*) malloc(hardwareConfigMsg.orientation.capacity * sizeof(char));
    hardwareConfigMsg.orientation.size = 0;
    hardwareConfigMsg.handle_1.type.capacity = 20;
    hardwareConfigMsg.handle_1.type.data = (char*) malloc(hardwareConfigMsg.handle_1.type.capacity * sizeof(char));
    hardwareConfigMsg.handle_1.type.size = 0;
    strcpy(hardwareConfigMsg.handle_1.type.data, "none");
    hardwareConfigMsg.handle_1.type.size = strlen(hardwareConfigMsg.handle_1.type.data);
    hardwareConfigMsg.handle_2.type.capacity = 20;
    hardwareConfigMsg.handle_2.type.data = (char*) malloc(hardwareConfigMsg.handle_2.type.capacity * sizeof(char));
    hardwareConfigMsg.handle_2.type.size = 0;
    strcpy(hardwareConfigMsg.handle_2.type.data, "none");
    hardwareConfigMsg.handle_2.type.size = strlen(hardwareConfigMsg.handle_2.type.data);
    hardwareConfigMsg.handle_1.param_names.capacity = 2;
    hardwareConfigMsg.handle_1.param_names.data = (rosidl_runtime_c__String*) malloc(hardwareConfigMsg.handle_1.param_names.capacity * sizeof(rosidl_runtime_c__String));
    hardwareConfigMsg.handle_1.param_names.size = 0;
    hardwareConfigMsg.handle_2.param_names.capacity = 2;
    hardwareConfigMsg.handle_2.param_names.data = (rosidl_runtime_c__String*) malloc(hardwareConfigMsg.handle_2.param_names.capacity * sizeof(rosidl_runtime_c__String));
    hardwareConfigMsg.handle_2.param_names.size = 0;
    hardwareConfigMsg.handle_1.param_vals.capacity = 2;
    hardwareConfigMsg.handle_1.param_vals.data = (float*) malloc(hardwareConfigMsg.handle_1.param_vals.capacity * sizeof(float));
    hardwareConfigMsg.handle_1.param_vals.size = 0;
    hardwareConfigMsg.handle_2.param_vals.capacity = 2;
    hardwareConfigMsg.handle_2.param_vals.data = (float*) malloc(hardwareConfigMsg.handle_2.param_vals.capacity * sizeof(float));
    hardwareConfigMsg.handle_2.param_vals.size = 0;
    for (uint ii = 0; ii < hardwareConfigMsg.handle_1.param_names.capacity; ii++) {
        hardwareConfigMsg.handle_1.param_names.data[ii].capacity = 10;
        hardwareConfigMsg.handle_1.param_names.data[ii].data = (char*) malloc(hardwareConfigMsg.handle_1.param_names.data[ii].capacity * sizeof(char));
        hardwareConfigMsg.handle_1.param_names.data[ii].size = 0;
        hardwareConfigMsg.handle_2.param_names.data[ii].capacity = 10;
        hardwareConfigMsg.handle_2.param_names.data[ii].data = (char*) malloc(hardwareConfigMsg.handle_2.param_names.data[ii].capacity * sizeof(char));
        hardwareConfigMsg.handle_2.param_names.data[ii].size = 0;
    }

    //mallocs for control config
    controlConfigMsg.motors_control.motor_1.type.capacity = 20;
    controlConfigMsg.motors_control.motor_1.type.data = (char*) malloc(controlConfigMsg.motors_control.motor_1.type.capacity * sizeof(char));
    controlConfigMsg.motors_control.motor_1.type.size = 0;
    controlConfigMsg.motors_control.motor_2.type.capacity = 20;
    controlConfigMsg.motors_control.motor_2.type.data = (char*) malloc(controlConfigMsg.motors_control.motor_2.type.capacity * sizeof(char));
    controlConfigMsg.motors_control.motor_2.type.size = 0;
}


/**
 * @brief Creates the node and advertises topics and services
 * 
 * @param cube Phicube object used to send and retrieve information to be transmitted
 */
void PhicubeRosComm::Init(Phicube &cube)
{
    set_microros_serial_transports(Serial);
    phicube = &cube;
}


bool PhicubeRosComm::CreateEntities()
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, PHICUBE_NODE_NAME, "", &support));
    RCCHECK(rclc_executor_init(&executor, &support.context, 16, &allocator));

    RCCHECK(rclc_publisher_init_default(&heartBeatPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/phicube/heartbeat"));
    RCCHECK(rclc_publisher_init_default(&motorStatePublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/phicube/motor_state"));
    RCCHECK(rclc_publisher_init_default(&diagnosticsPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(phicube_msgs, msg, RobotDiagnostics), "/phicube/log/diagnostic"));
    RCCHECK(rclc_publisher_init_default(&hardwareConfigPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(phicube_msgs, msg, HardwareConfig), "/phicube/hardware_config"));
    RCCHECK(rclc_publisher_init_default(&controlConfigPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(phicube_msgs, msg, ControlConfig), "/phicube/control_config"));

    RCCHECK(rclc_subscription_init_default(&cmdStateSubscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(phicube_msgs, msg, CmdState), "/phicube/cmd_state"));
    RCCHECK(rclc_executor_add_subscription_with_context(&executor, &cmdStateSubscriber, &cmdState, &PhicubeRosComm::CmdStateCB, this, ON_NEW_DATA))

    RCCHECK(rclc_service_init_default(&setConfigurationServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetConfiguration), "/phicube/set_configuration"));
    RCCHECK(rclc_service_init_default(&setControllerServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetController), "/phicube/set_controller"));
    RCCHECK(rclc_service_init_default(&setDecoupledParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetDecoupledParams), "/phicube/set_decoupled_params"));
    RCCHECK(rclc_service_init_default(&setCoupledParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetCoupledParams), "/phicube/set_coupled_params"));
    RCCHECK(rclc_executor_add_service_with_context(&executor, &setConfigurationServer, &setConfigurationRequest, &setConfigurationResponse, &PhicubeRosComm::SetConfigurationCB, this));
    RCCHECK(rclc_executor_add_service_with_context(&executor, &setControllerServer, &setControllerRequest, &setControllerResponse, &PhicubeRosComm::SetControllerCB, this));
    RCCHECK(rclc_executor_add_service_with_context(&executor, &setDecoupledParamsServer, &setDecoupledParamsRequest, &setDecoupledParamsResponse, &PhicubeRosComm::SetDecoupledParamsCB, this));
    RCCHECK(rclc_executor_add_service_with_context(&executor, &setCoupledParamsServer, &setCoupledParamsRequest, &setCoupledParamsResponse, &PhicubeRosComm::SetCoupledParamsCB, this));

    if (PHICUBE_ENABLE_HAPTICS) {
        RCCHECK(rclc_service_init_default(&setLedStateServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetLedState), "/phicube/set_led_state"));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &setLedStateServer, &setLedStateRequest, &setLedStateResponse, &PhicubeRosComm::SetLedStateCB, this));
    }
    
    if (PHICUBE_ENABLE_HOMING) {
        RCCHECK(rclc_service_init_default(&initServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetHoming), "/phicube/init"));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &initServer, &initRequest, &initResponse, &PhicubeRosComm::InitCB, this));
    }

    if (PHICUBE_ENABLE_EXPERT_MODE) {
        RCCHECK(rclc_service_init_default(&setAssRefVelEstPIDParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetPIDParams), "/phicube/set_ass_ref_vel_est_PID_params"));
        RCCHECK(rclc_service_init_default(&setBimRefVelEstPIDParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetPIDParams), "/phicube/set_bim_ref_vel_est_PID_params"));
        RCCHECK(rclc_service_init_default(&setVelEstPIDParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetPIDParams), "/phicube/set_vel_est_PID_params"));
        RCCHECK(rclc_service_init_default(&setPosPIDParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetPIDParams), "/phicube/set_pos_PID_params"));
        RCCHECK(rclc_service_init_default(&setVelPIDParamsServer, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(phicube_msgs, srv, SetPIDParams), "/phicube/set_vel_PID_params"));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &setAssRefVelEstPIDParamsServer, &setAssRefVelEstPIDParamsRequest, &setAssRefVelEstPIDParamsResponse, &PhicubeRosComm::SetAssRefVelEstPIDParamsCB, this));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &setBimRefVelEstPIDParamsServer, &setBimRefVelEstPIDParamsRequest, &setBimRefVelEstPIDParamsResponse, &PhicubeRosComm::SetBimRefVelEstPIDParamsCB, this));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &setVelEstPIDParamsServer, &setVelEstPIDParamsRequest, &setVelEstPIDParamsResponse, &PhicubeRosComm::SetVelEstPIDParamsCB, this));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &setPosPIDParamsServer, &setPosPIDParamsRequest, &setPosPIDParamsResponse, &PhicubeRosComm::SetPosPIDParamsCB, this));
        RCCHECK(rclc_executor_add_service_with_context(&executor, &setVelPIDParamsServer, &setVelPIDParamsRequest, &setVelPIDParamsResponse, &PhicubeRosComm::SetVelPIDParamsCB, this));
    }
    return true;
}


void PhicubeRosComm::RunEntities()
{
    struct timespec ts;
	ClockGetTime(CLOCK_REALTIME, &ts);

    if (needStatusUpdate) {
        //publish heartBeat
        if (PHICUBE_ENABLE_HEARTBEAT) {
            RCSOFTCHECK(rcl_publish(&heartBeatPublisher, &heartBeatMsg, NULL));
        }

        //publish diagnostic data
        if (PHICUBE_ENABLE_DIAGNOSTIC) {
            phicube->GetOverheatIndex(overheat_index);
            phicube->GetEncodersDiff(encoders_diff);
            PhicubeRobot::RobotError robotError = phicube->GetRobotError();
            diagnosticMsg.timestamp.sec = ts.tv_sec;
            diagnosticMsg.timestamp.nanosec = ts.tv_nsec;
            diagnosticMsg.missing_power = robotError.noPower;
            diagnosticMsg.motor_1.motor_overheat = robotError.motor1Overheat;
            diagnosticMsg.motor_1.overheat_index = overheat_index[0];
            diagnosticMsg.motor_2.motor_overheat = robotError.motor2Overheat;
            diagnosticMsg.motor_2.overheat_index = overheat_index[1];
            diagnosticMsg.motor_1.encoders_disaligned = robotError.encoders1Disaligned;
            diagnosticMsg.motor_1.encoders_diff = encoders_diff[0];
            diagnosticMsg.motor_2.encoders_disaligned = robotError.encoders2Disaligned;
            diagnosticMsg.motor_2.encoders_diff = encoders_diff[1];
            RCSOFTCHECK(rcl_publish(&diagnosticsPublisher, &diagnosticMsg, NULL));
        }

        //publish hardware config
        switch(phicube->GetOrientation()) {
            case Phicube::PhicubeOrientation::FRONTAL:
                strcpy(hardwareConfigMsg.orientation.data,"frontal");
            break;
            case Phicube::PhicubeOrientation::VERTICAL:
                strcpy(hardwareConfigMsg.orientation.data,"vertical");
            break;
            case Phicube::PhicubeOrientation::SAGITTAL:
                strcpy(hardwareConfigMsg.orientation.data,"sagittal");
            break;
        }
        hardwareConfigMsg.orientation.size = strlen(hardwareConfigMsg.orientation.data);
        RCSOFTCHECK(rcl_publish(&hardwareConfigPublisher, &hardwareConfigMsg, NULL));
        
        //publish control config
        switch(phicube->GetRobotMode()) {
            case PhicubeRobot::RobotMode::EMPTY_MODE:
                strcpy(controlConfigMsg.motors_control.motor_1.type.data,"empty");
                strcpy(controlConfigMsg.motors_control.motor_2.type.data,"empty");
            break;
            case PhicubeRobot::RobotMode::NOMODE:
                strcpy(controlConfigMsg.motors_control.motor_1.type.data,"nomode");
                strcpy(controlConfigMsg.motors_control.motor_2.type.data,"nomode");
            break;
            case PhicubeRobot::RobotMode::COUPLED:
                strcpy(controlConfigMsg.motors_control.motor_1.type.data,"coupled");
                strcpy(controlConfigMsg.motors_control.motor_2.type.data,"coupled");
            break;
            case PhicubeRobot::RobotMode::DECOUPLED:
                strcpy(controlConfigMsg.motors_control.motor_1.type.data,"decoupled");
                strcpy(controlConfigMsg.motors_control.motor_2.type.data,"decoupled");
            break;
            case PhicubeRobot::RobotMode::POSITION:
                strcpy(controlConfigMsg.motors_control.motor_1.type.data,"position");
                strcpy(controlConfigMsg.motors_control.motor_2.type.data,"position");
            break;
            case PhicubeRobot::RobotMode::VELOCITY:
                strcpy(controlConfigMsg.motors_control.motor_1.type.data,"velocity");
                strcpy(controlConfigMsg.motors_control.motor_2.type.data,"velocity");
            break;
        }
        controlConfigMsg.motors_control.motor_1.type.size = strlen(controlConfigMsg.motors_control.motor_1.type.data);
        controlConfigMsg.motors_control.motor_2.type.size = strlen(controlConfigMsg.motors_control.motor_2.type.data);
        RCSOFTCHECK(rcl_publish(&controlConfigPublisher, &controlConfigMsg, NULL));
        
        needStatusUpdate = false;
    }

    // publish motorState
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

    // spin to receive callbacks
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 1000));
}


void PhicubeRosComm::DestroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&heartBeatPublisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&motorStatePublisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&diagnosticsPublisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&hardwareConfigPublisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&controlConfigPublisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&cmdStateSubscriber, &node));

    RCSOFTCHECK(rcl_service_fini(&initServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setAssRefVelEstPIDParamsServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setBimRefVelEstPIDParamsServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setVelEstPIDParamsServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setPosPIDParamsServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setVelPIDParamsServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setLedStateServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setConfigurationServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setControllerServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setDecoupledParamsServer, &node));
    RCSOFTCHECK(rcl_service_fini(&setCoupledParamsServer, &node));

    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}


/**
 * @brief Spins the node handle to run ROS communication
 * 
 */
void PhicubeRosComm::Update()
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
            };
        break;
        case AGENT_CONNECTED:
            agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (agentState == AGENT_CONNECTED) {
                RunEntities();
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


void PhicubeRosComm::UpdateStatus()
{
    needStatusUpdate = true;
}


void PhicubeRosComm::CmdStateCB(const void * msgIn, void * context)
{
    const phicube_msgs__msg__CmdState * cmdState = (const phicube_msgs__msg__CmdState *) msgIn;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    if (phicubeContext->phicube->GetRobotState() == PhicubeRobot::RobotState::READY) {
        phicubeContext->phicube->SetTarget(cmdState->cmd1, cmdState->cmd2);
    }
}


void PhicubeRosComm::InitCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetHoming_Request * req = (phicube_msgs__srv__SetHoming_Request *) reqMsg;
    phicube_msgs__srv__SetHoming_Response * res = (phicube_msgs__srv__SetHoming_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    if (phicubeContext->phicube->GetRobotState() == PhicubeRobot::RobotState::OFF) {
        phicubeContext->phicube->SetHoming(req->motor_1_homing, req->motor_2_homing);
        strcpy(res->message.data, "Homing procedure started.");
        res->message.size = strlen(res->message.data);
    }
    else if (phicubeContext->phicube->GetRobotState() == PhicubeRobot::RobotState::READY) {
        phicubeContext->phicube->SetHoming(req->motor_1_homing, req->motor_2_homing);
        strcpy(res->message.data, "Reset procedure started.");
        res->message.size = strlen(res->message.data);
    }
    else {
        strcpy(res->message.data, "It was not possible to execute the request.");
        res->message.size = strlen(res->message.data);
    }
    res->success = true;
}


void PhicubeRosComm::SetConfigurationCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetConfiguration_Request * req = (phicube_msgs__srv__SetConfiguration_Request *) reqMsg;
    phicube_msgs__srv__SetConfiguration_Response * res = (phicube_msgs__srv__SetConfiguration_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;
    
    phicubeContext->hardwareConfigMsg.handle_1 = req->handle_1;
    phicubeContext->hardwareConfigMsg.handle_2 = req->handle_2;
    //TODO: Check the handling of handle 'params' field.

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    strcpy(res->message.data, "The configuration has been received.");
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetControllerCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetController_Request * req = (phicube_msgs__srv__SetController_Request *) reqMsg;
    phicube_msgs__srv__SetController_Response * res = (phicube_msgs__srv__SetController_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    if (phicubeContext->phicube->GetRobotState() == PhicubeRobot::RobotState::READY) {

        switch(req->controller) {
            case PhicubeRobot::RobotMode::NOMODE:
            case PhicubeRobot::RobotMode::EMPTY_MODE:
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = 0.0;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.is_active = false;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.dominance = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.transmission = 0.0;
            break;
            case PhicubeRobot::RobotMode::POSITION:
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = PHICUBE_POS_PID_KP;
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = PHICUBE_POS_PID_KD;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = PHICUBE_POS_PID_KP;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = PHICUBE_POS_PID_KD;
                phicubeContext->controlConfigMsg.bilateral_control.is_active = false;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.dominance = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.transmission = 0.0;
            break;
            case PhicubeRobot::RobotMode::VELOCITY:
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = PHICUBE_VEL_PID_KP;
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = PHICUBE_VEL_PID_KD;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = PHICUBE_VEL_PID_KP;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = PHICUBE_VEL_PID_KD;
                phicubeContext->controlConfigMsg.bilateral_control.is_active = false;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.dominance = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.transmission = 0.0;
            break;
            case PhicubeRobot::RobotMode::DECOUPLED:
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = PHICUBE_K1_ASSISTANCE;
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = PHICUBE_D1_ASSISTANCE;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = PHICUBE_K2_ASSISTANCE;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = PHICUBE_D2_ASSISTANCE;
                phicubeContext->controlConfigMsg.bilateral_control.is_active = false;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.stiffness = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.damping = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.dominance = 0.0;
                phicubeContext->controlConfigMsg.bilateral_control.transmission = 0.0;
            break;
            case PhicubeRobot::RobotMode::COUPLED:
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = PHICUBE_K1_ASSISTANCE;
                phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = PHICUBE_D1_ASSISTANCE;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = PHICUBE_K2_ASSISTANCE;
                phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = PHICUBE_D2_ASSISTANCE;
                phicubeContext->controlConfigMsg.bilateral_control.is_active = true;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.stiffness = PHICUBE_K1_BILATERAL;
                phicubeContext->controlConfigMsg.bilateral_control.side_1_params.damping = PHICUBE_D1_BILATERAL;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.stiffness = PHICUBE_K2_BILATERAL;
                phicubeContext->controlConfigMsg.bilateral_control.side_2_params.damping = PHICUBE_D2_BILATERAL;
                phicubeContext->controlConfigMsg.bilateral_control.dominance = PHICUBE_BETA_BILATERAL;
                phicubeContext->controlConfigMsg.bilateral_control.transmission = PHICUBE_COUPLED_RHO;
            break;
        }

        phicubeContext->phicube->SetController(req->controller);
        strcpy(res->message.data, "Controller activation request accepted.");
    }
    else {
        strcpy(res->message.data, "Controller activation request refused. Robot is not ready.");
    }
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetAssRefVelEstPIDParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetPIDParams_Request * req = (phicube_msgs__srv__SetPIDParams_Request *) reqMsg;
    phicube_msgs__srv__SetPIDParams_Response * res = (phicube_msgs__srv__SetPIDParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    float kp1 = req->motor_1_kp;
    float ki1 = req->motor_1_ki;
    float kd1 = req->motor_1_kd;
    float kp2 = req->motor_2_kp;
    float ki2 = req->motor_2_ki;
    float kd2 = req->motor_2_kd;
    phicubeContext->phicube->SetPIDAssRefVelEstParameters(kp1, ki1, kd1, kp2, ki2, kd2);

    strcpy(res->message.data, "The assistance reference velocity estimator has been retuned according to request");
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetBimRefVelEstPIDParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetPIDParams_Request * req = (phicube_msgs__srv__SetPIDParams_Request *) reqMsg;
    phicube_msgs__srv__SetPIDParams_Response * res = (phicube_msgs__srv__SetPIDParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    float kp1 = req->motor_1_kp;
    float ki1 = req->motor_1_ki;
    float kd1 = req->motor_1_kd;
    float kp2 = req->motor_2_kp;
    float ki2 = req->motor_2_ki;
    float kd2 = req->motor_2_kd;
    phicubeContext->phicube->SetPIDBimRefVelEstParameters(kp1, ki1, kd1, kp2, ki2, kd2);

    strcpy(res->message.data, "The bimanual reference velocity estimator has been retuned according to request");
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetVelEstPIDParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetPIDParams_Request * req = (phicube_msgs__srv__SetPIDParams_Request *) reqMsg;
    phicube_msgs__srv__SetPIDParams_Response * res = (phicube_msgs__srv__SetPIDParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    float kp1 = req->motor_1_kp;
    float ki1 = req->motor_1_ki;
    float kd1 = req->motor_1_kd;
    float kp2 = req->motor_2_kp;
    float ki2 = req->motor_2_ki;
    float kd2 = req->motor_2_kd;
    phicubeContext->phicube->SetPIDVelocityEstParameters(kp1, ki1, kd1, kp2, ki2, kd2);

    strcpy(res->message.data, "The motor velocity estimator has been retuned according to request");
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetPosPIDParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetPIDParams_Request * req = (phicube_msgs__srv__SetPIDParams_Request *) reqMsg;
    phicube_msgs__srv__SetPIDParams_Response * res = (phicube_msgs__srv__SetPIDParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    float kp1 = req->motor_1_kp;
    float ki1 = req->motor_1_ki;
    float kd1 = req->motor_1_kd;
    float kp2 = req->motor_2_kp;
    float ki2 = req->motor_2_ki;
    float kd2 = req->motor_2_kd;
    phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = kp1;
    phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = kd1;
    phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = kp2;
    phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = kd2;
    phicubeContext->phicube->SetPIDPositionParameters(kp1, ki1, kd1, kp2, ki2, kd2);

    strcpy(res->message.data, "The low-level position controller has been retuned according to request");
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetVelPIDParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetPIDParams_Request * req = (phicube_msgs__srv__SetPIDParams_Request *) reqMsg;
    phicube_msgs__srv__SetPIDParams_Response * res = (phicube_msgs__srv__SetPIDParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    float kp1 = req->motor_1_kp;
    float ki1 = req->motor_1_ki;
    float kd1 = req->motor_1_kd;
    float kp2 = req->motor_2_kp;
    float ki2 = req->motor_2_ki;
    float kd2 = req->motor_2_kd;
    phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = kp1;
    phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = kd1;
    phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = kp2;
    phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = kd2;
    phicubeContext->phicube->SetPIDVelocityParameters(kp1, ki1, kd1, kp2, ki2, kd2);

    strcpy(res->message.data, "The low-level velocity controller has been retuned according to request");
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetDecoupledParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetDecoupledParams_Request * req = (phicube_msgs__srv__SetDecoupledParams_Request *) reqMsg;
    phicube_msgs__srv__SetDecoupledParams_Response * res = (phicube_msgs__srv__SetDecoupledParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    if (phicubeContext->phicube->GetRobotMode() == PhicubeRobot::RobotMode::DECOUPLED) {
        phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = req->k1;
        phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = req->d1;
        phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = req->k2;
        phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = req->d2;
        phicubeContext->phicube->SetDecoupledParams(req->k1, req->k2, req->d1, req->d2);
        strcpy(res->message.data, "Decoupled controller params accepted.");
    }
    else {
        strcpy(res->message.data, "Decoupled controller params refused. Controller is not active.");
    }
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetCoupledParamsCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetCoupledParams_Request * req = (phicube_msgs__srv__SetCoupledParams_Request *) reqMsg;
    phicube_msgs__srv__SetCoupledParams_Response * res = (phicube_msgs__srv__SetCoupledParams_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    if (phicubeContext->phicube->GetRobotMode() == PhicubeRobot::RobotMode::COUPLED) {
        phicubeContext->controlConfigMsg.motors_control.motor_1.params.stiffness = req->k1a;
        phicubeContext->controlConfigMsg.motors_control.motor_1.params.damping = req->d1a;
        phicubeContext->controlConfigMsg.motors_control.motor_2.params.stiffness = req->k2a;
        phicubeContext->controlConfigMsg.motors_control.motor_2.params.damping = req->d2a;
        phicubeContext->controlConfigMsg.bilateral_control.is_active = true;
        phicubeContext->controlConfigMsg.bilateral_control.side_1_params.stiffness = req->k1b;
        phicubeContext->controlConfigMsg.bilateral_control.side_1_params.damping = req->d1b;
        phicubeContext->controlConfigMsg.bilateral_control.side_2_params.stiffness = req->k2b;
        phicubeContext->controlConfigMsg.bilateral_control.side_2_params.damping = req->d2b;
        phicubeContext->controlConfigMsg.bilateral_control.dominance = req->beta;
        phicubeContext->controlConfigMsg.bilateral_control.transmission = req->rho;
        phicubeContext->phicube->SetCoupledParams(req->k1a, req->k2a, req->d1a, req->d2a, req->k1b, req->k2b, req->d1b, req->d2b, req->rho, req->beta);
        strcpy(res->message.data, "Coupled controller params accepted.");
    }
    else {
        strcpy(res->message.data, "Coupled controller params refused. Controller is not active.");
    }
    res->message.size = strlen(res->message.data);
    res->success = true;
}


void PhicubeRosComm::SetLedStateCB(const void * reqMsg, void * resMsg, void * context)
{
    phicube_msgs__srv__SetLedState_Request * req = (phicube_msgs__srv__SetLedState_Request *) reqMsg;
    phicube_msgs__srv__SetLedState_Response * res = (phicube_msgs__srv__SetLedState_Response *) resMsg;
    PhicubeRosComm * phicubeContext = (PhicubeRosComm *) context;

    res->message.capacity = 100;
    res->message.data = (char*) malloc(res->message.capacity * sizeof(char));
    res->message.size = 0;

    phicubeContext->phicube->SetLedState(LedHW::LedState(req->state), req->ring, req->brightness, req->speed, req->timeout, req->timespan, req->color_r, req->color_g, req->color_b);

    strcpy(res->message.data, "The led ring command has been sent successfully");
    res->message.size = strlen(res->message.data);
    res->success = true;
}