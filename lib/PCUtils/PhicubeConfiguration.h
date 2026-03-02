/**
 * @file PhicubeConfiguration.h
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief Convenience file to store all PhiCube constants.
 * @version 0.1
 * @date 2026-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PHICUBE_CONFIGURATION
#define PHICUBE_CONFIGURATION

#define PHICUBE_CANCOMM_PERIOD_MS      1           // PhiCube CAN communication cycle running at 1000 Hz
#define PHICUBE_ROBOTCTRL_PERIOD_MS    2           // PhiCube motors control cycle running at 1000 Hz
#define PHICUBE_ROSCOMM_PERIOD_MS      20          // ROS2 node spin cycle running at at 50 Hz (max pub frequency)

#define PHICUBE_MOTORSSTATE_PERIOD_MS  20          // ROS2 topic for motors state msg published at 50 Hz
#define PHICUBE_ROBOTSTATE_PERIOD_MS   1000         // ROS2 topic for robot state msg published at 2 Hz

#define CANCOMM_TIMEOUT_MS             20          // Max time to wait between sending a CAN command and receiving the response.
#define CANCOMM_QUEUE_SIZE             30          // Max number of CAN commands that can be enqueued.
#define CANCOMM_M1_CANID               0x101       // CAN ID for sending msgs to Motor 1
#define CANCOMM_M2_CANID               0x102       // CAN ID for sending msgs to Motor 2

#define PRIMARY_ENCODER_CPR            16384       // Counts per revolutions of the primary encoder for GIM6010-8 motors

#endif