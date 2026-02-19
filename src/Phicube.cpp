#include "Phicube.h"


Phicube::Phicube() {}


void Phicube::Init()
{
    phicubeRobot.Init();
    phicubeLed.Init();

    switch (PHICUBE_IMU_WIRE) {
        case 0:
            imu.setBus(&Wire);
        break;
        case 1:
            imu.setBus(&Wire1);
        break;
        case 2:
            imu.setBus(&Wire2);
        break;
    }
    imu.init();
    imu.enableDefault();
}


void Phicube::UpdateRobot()
{
    phicubeRobot.Update();
}


void Phicube::UpdateLed()
{
    phicubeLed.Update();
}


PhicubeRobot::RobotState Phicube::GetRobotState()
{
    PhicubeRobot::RobotState state = phicubeRobot.GetRobotState();
    return state;
}


PhicubeRobot::RobotMode Phicube::GetRobotMode()
{
    PhicubeRobot::RobotMode mode = phicubeRobot.GetRobotMode();
    return mode;
}


PhicubeRobot::RobotError Phicube::GetRobotError()
{
    PhicubeRobot::RobotError error = phicubeRobot.GetRobotError();
    return error;
}


//------- published as topics -------//

void Phicube::GetCurrentPosition(float(&pos)[2])
{
    phicubeRobot.GetCurrentPosition(pos);
}


void Phicube::GetCurrentVelocity(float(&vel)[2])
{
    phicubeRobot.GetCurrentVelocity(vel);
}


void Phicube::GetCurrentEffort(float(&eff)[2])
{
    phicubeRobot.GetCurrentEffort(eff);
}

void Phicube::GetOverheatIndex(float(&overheat_index)[2])
{
    phicubeRobot.GetOverheatIndex(overheat_index);
}

void Phicube::GetEncodersDiff(float(&encoders_diff)[2])
{
    phicubeRobot.GetEncodersDiff(encoders_diff);
}


//------- exposed through interrupts -------//

void Phicube::SetChAMotor1State(bool state)
{
    phicubeRobot.SetChAMotor1State(state);
}


void Phicube::SetChBMotor1State(bool state)
{
    phicubeRobot.SetChBMotor1State(state);
}


void Phicube::SetChAMotor2State(bool state)
{
    phicubeRobot.SetChAMotor2State(state);
}


void Phicube::SetChBMotor2State(bool state)
{
    phicubeRobot.SetChBMotor2State(state);
}


//------- exposed through services -------//

void Phicube::SetPIDAssRefVelEstParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2)
{
    phicubeRobot.SetPIDAssRefVelEstParameters(kp1, ki1, kd1, kp2, ki2, kd2);
}


void Phicube::SetPIDBimRefVelEstParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2)
{
    phicubeRobot.SetPIDBimRefVelEstParameters(kp1, ki1, kd1, kp2, ki2, kd2);
}


void Phicube::SetPIDVelocityEstParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2)
{
    phicubeRobot.SetPIDVelocityEstParameters(kp1, ki1, kd1, kp2, ki2, kd2);
}


void Phicube::SetPIDPositionParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2)
{
    phicubeRobot.SetPIDPositionParameters(kp1, ki1, kd1, kp2, ki2, kd2);
}


void Phicube::SetPIDVelocityParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2)
{
    phicubeRobot.SetPIDVelocityParameters(kp1, ki1, kd1, kp2, ki2, kd2);
}


void Phicube::SetHoming(bool reqMot1, bool reqMot2)
{
    phicubeRobot.SetHoming(reqMot1, reqMot2);
}


void Phicube::SetLedState(LedHW::LedState state, int index, int brightness, int speed, bool timeout, int duration, int colorR, int colorG, int colorB)
{
    phicubeLed.SetLedState(state, index, brightness, speed, timeout, duration, colorR, colorG, colorB);
}


void Phicube::SetController(int ctrl)
{
    phicubeRobot.SetController(ctrl);
}


void Phicube::SetTarget(float tgt1, float tgt2)
{
    phicubeRobot.SetTarget(tgt1, tgt2);
}


void Phicube::SetDecoupledParams(float k1, float k2, float d1, float d2)
{
    phicubeRobot.SetDecoupledParams(k1, k2, d1, d2);
}


void Phicube::SetCoupledParams(float k1a, float k2a, float d1a, float d2a, float k1b, float k2b, float d1b, float d2b, float rho, float beta)
{
    phicubeRobot.SetCoupledParams(k1a, k2a, d1a, d2a, k1b, k2b, d1b, d2b, rho, beta);
}


Phicube::PhicubeOrientation Phicube::GetOrientation()
{
    Phicube::PhicubeOrientation orientation;

    imu.readAcc();
    int16_t ax = abs(imu.a.x);
    int16_t ay = abs(imu.a.y);
    int16_t az = abs(imu.a.z);

    if (ax > ay) {
        if (ax > az) {
            orientation = Phicube::PhicubeOrientation::VERTICAL;
        }
        else {
            orientation = Phicube::PhicubeOrientation::FRONTAL;
        }
    }
    else {
        if (ay > az) {
            orientation = Phicube::PhicubeOrientation::SAGITTAL;
        }
        else {
            orientation = Phicube::PhicubeOrientation::FRONTAL;
        }
    }

    return orientation;
}