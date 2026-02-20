#include "PhicubeRobot.h"


PhicubeRobot::PhicubeRobot()
    :   manager(),
        driverM1(CANCOMM_M1_CANID, manager),
        driverM2(CANCOMM_M2_CANID, manager) {}


void PhicubeRobot::Init()
{
    driverM1.Init();
    driverM2.Init();
}


void PhicubeRobot::UpdateRobotCtrl()
{
    driverM1.ReadPosition();
    driverM1.ReadVelocity();
    driverM1.ReadCurrent();
    driverM2.ReadPosition();
    driverM2.ReadVelocity();
    driverM2.ReadCurrent();

    manager.Update();
}


void PhicubeRobot::GetCurrentPosition(float(&pos)[2])
{
    pos[0] = driverM1.GetMTPosition();
    pos[1] = driverM2.GetMTPosition();
}


void PhicubeRobot::GetCurrentVelocity(float(&vel)[2])
{
    vel[0] = driverM1.GetVelocity();
    vel[1] = driverM2.GetVelocity();
}


void PhicubeRobot::GetCurrentEffort(float(&eff)[2])
{
    eff[0] = driverM1.GetTorque();
    eff[1] = driverM2.GetTorque();
}