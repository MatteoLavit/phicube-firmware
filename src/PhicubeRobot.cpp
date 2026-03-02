#include "PhicubeRobot.h"


// -----------------------------------------------------
// ---------------------- PUBLIC -----------------------
// -----------------------------------------------------


PhicubeRobot::PhicubeRobot()
    :   manager(),
        driverM1(CANCOMM_M1_CANID, manager),
        driverM2(CANCOMM_M2_CANID, manager) {}


bool PhicubeRobot::InitManager()
{
    managerInitialized = manager.Init();
    systemReady = managerInitialized && motorsInitialized;
    return managerInitialized;
}


bool PhicubeRobot::InitMotors()
{
    bool initM1 = true;
    if (driverM1.GetBootVersion() < 0) {
        initM1 = false;
        if (!driverM1.ReadVersions()) return false;
    }
    if (driverM1.GetPolePairs() < 0) {
        initM1 = false;
        if (!driverM1.ReadInfos()) return false;
    }
    if (driverM1.GetPosKp() < 0) {
        initM1 = false;
        if (!driverM1.ReadPosKp()) return false;
    }
    if (driverM1.GetPosKi() < 0) {
        initM1 = false;
        if (!driverM1.ReadPosKi()) return false;
    }
    if (driverM1.GetVelKp() < 0) {
        initM1 = false;
        if (!driverM1.ReadVelKp()) return false;
    }
    if (driverM1.GetVelKi() < 0) {
        initM1 = false;
        if (!driverM1.ReadVelKi()) return false;
    }

    bool initM2 = true;
    if (driverM2.GetBootVersion() < 0) {
        initM2 = false;
        if (!driverM2.ReadVersions()) return false;    
    }
    if (driverM2.GetPolePairs() < 0) {
        initM2 = false;
        if (!driverM2.ReadInfos()) return false;
    }
    if (driverM2.GetPosKp() < 0) {
        initM2 = false;
        if (!driverM2.ReadPosKp()) return false;
    }
    if (driverM2.GetPosKi() < 0) {
        initM2 = false;
        if (!driverM2.ReadPosKi()) return false;
    }
    if (driverM2.GetVelKp() < 0) {
        initM2 = false;
        if (!driverM2.ReadVelKp()) return false;
    }
    if (driverM2.GetVelKi() < 0) {
        initM2 = false;
        if (!driverM2.ReadVelKi()) return false;
    }
    
    motorsInitialized = initM1 && initM2;
    systemReady = managerInitialized && motorsInitialized;
    return motorsInitialized;
}


void PhicubeRobot::UpdateCANComm()
{
    manager.Update();
}


void PhicubeRobot::UpdateRobotCtrl()
{
    driverM1.ReadPosition();
    driverM1.ReadVelocity();
    driverM1.ReadCurrent();

    driverM2.ReadPosition();
    driverM2.ReadVelocity();
    driverM2.ReadCurrent();
}


void PhicubeRobot::GetMotorsPosition(float(&pos)[2])
{
    pos[0] = driverM1.GetMTPosition();
    pos[1] = driverM2.GetMTPosition();
}


void PhicubeRobot::GetMotorsVelocity(float(&vel)[2])
{
    vel[0] = driverM1.GetVelocity();
    vel[1] = driverM2.GetVelocity();
}


void PhicubeRobot::GetMotorsTorque(float(&trq)[2])
{
    trq[0] = driverM1.GetTorque();
    trq[1] = driverM2.GetTorque();
}


void PhicubeRobot::GetMotorsCurrent(float(&cur)[2])
{
    cur[0] = driverM1.GetCurrent();
    cur[1] = driverM2.GetCurrent();
}


void PhicubeRobot::GetMotorsInfo(phicube_msgs__msg__GIM6010Info(&inf)[2])
{
    inf[0].boot_version = driverM1.GetBootVersion();
    inf[0].sw_version = driverM1.GetSWVersion();
    inf[0].hw_version = driverM1.GetHWVersion();
    inf[0].can_version = driverM1.GetCANVersion();
    inf[0].pole_pairs = driverM1.GetPolePairs();
    inf[0].reduction_ratio = driverM1.GetReductionRatio();
    inf[0].torque_constant = driverM1.GetTorqueConstant();
    inf[0].pos_kp = driverM1.GetPosKp();
    inf[0].pos_ki = driverM1.GetPosKi();
    inf[0].vel_kp = driverM1.GetVelKp();
    inf[0].vel_ki = driverM1.GetVelKi();
    inf[0].bus_voltage = driverM1.GetBusVoltage();
    inf[0].bus_current = driverM1.GetBusCurrent();
    inf[0].bus_temperature = driverM1.GetTemperature();
    inf[0].bus_mode = driverM1.GetMode();
    inf[0].bus_error = driverM1.GetError();

    inf[1].boot_version = driverM2.GetBootVersion();
    inf[1].sw_version = driverM2.GetSWVersion();
    inf[1].hw_version = driverM2.GetHWVersion();
    inf[1].can_version = driverM2.GetCANVersion();
    inf[1].pole_pairs = driverM2.GetPolePairs();
    inf[1].reduction_ratio = driverM2.GetReductionRatio();
    inf[1].torque_constant = driverM2.GetTorqueConstant();
    inf[1].pos_kp = driverM2.GetPosKp();
    inf[1].pos_ki = driverM2.GetPosKi();
    inf[1].vel_kp = driverM2.GetVelKp();
    inf[1].vel_ki = driverM2.GetVelKi();
    inf[1].bus_voltage = driverM2.GetBusVoltage();
    inf[1].bus_current = driverM2.GetBusCurrent();
    inf[1].bus_temperature = driverM2.GetTemperature();
    inf[1].bus_mode = driverM2.GetMode();
    inf[1].bus_error = driverM2.GetError();

    driverM1.ReadStatus();
    driverM2.ReadStatus();
}


bool PhicubeRobot::IsSystemReady()
{
    return systemReady;
}


bool PhicubeRobot::MotorsHome(bool hM1, bool hM2)
{
    bool succM1 = true;
    if (hM1) succM1 = driverM1.GoHomeByShortest();
    bool succM2 = true;
    if (hM2) succM2 = driverM2.GoHomeByShortest();

    return succM1 && succM2;
}


bool PhicubeRobot::MotorsOff(bool oM1, bool oM2)
{
    bool succM1 = true;
    if (oM1) succM1 = driverM1.MotorOff();
    bool succM2 = true;
    if (oM2) succM2 = driverM2.MotorOff();

    return succM1 && succM2;
}