/**
 * @file PhicubeRobot.h
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief Class used for the low level control of PhiCube devices
 * @version 0.1
 * @date 2026-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PHICUBE_ROBOT
#define PHICUBE_ROBOT

#include "GIM6010Driver.h"
#include "CANManager.h"
#include "PhicubeConfiguration.h"

#include <phicube_msgs/msg/gim6010_info.h>


class PhicubeRobot
{
    public:

        PhicubeRobot();

        bool InitManager();
        bool InitMotors();
        void UpdateCANComm();
        void UpdateRobotCtrl();

        void GetMotorsPosition(float(&pos)[2]);
        void GetMotorsVelocity(float(&vel)[2]);
        void GetMotorsTorque(float(&trq)[2]);
        void GetMotorsCurrent(float(&cur)[2]);
        void GetMotorsInfo(phicube_msgs__msg__GIM6010Info(&inf)[2]);

        bool IsSystemReady();
        bool MotorsHome(bool hM1, bool hM2);
        bool MotorsOff(bool oM1, bool oM2);

    private:

        bool managerInitialized;
        bool motorsInitialized;
        bool systemReady;

        CANManager manager;
        GIM6010Driver driverM1;
        GIM6010Driver driverM2;
};


#endif