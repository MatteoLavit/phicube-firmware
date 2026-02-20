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


class PhicubeRobot
{
    public:

        PhicubeRobot();

        void Init();
        void UpdateRobotCtrl();

        void GetCurrentPosition(float(&pos)[2]);
        void GetCurrentVelocity(float(&vel)[2]);
        void GetCurrentEffort(float(&eff)[2]);

    private:

        CANManager manager;
        GIM6010Driver driverM1;
        GIM6010Driver driverM2;
};


#endif