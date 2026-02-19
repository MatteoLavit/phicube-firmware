/**
 * @file main.cpp
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <mbed.h>
#include <rtos.h>
#include "PhicubeConfiguration.h"
#include "PhicubeCtrl.h"
#include "PhicubeRosComm.h"

using namespace mbed;
using namespace rtos;
using namespace rtos::Kernel;
using namespace std::chrono_literals;


Thread RobotThread;
Thread CommThread;
Thread StatusThread;
PhicubeCtrl phicubeCtrl;
PhicubeRosComm phicubeRosComm;


void PhicubeRobotCtrlThread()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_ROBOT_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicube.UpdateRobotCtrl();
  }
}

void PhicubeMotorStateThread()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_MOTORSTATE_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeRosComm.UpdateMotorState();
  }
}

void PhicubeDeviceStateThread()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_DEVICESTATE_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeRosComm.UpdateDeviceState();
  }
}


/**
 * @brief Set the current state for Ch A Motor 1 incremental encoder depending on the corresponding pin status.
 * 
 */
void SetChAMotor1State()
{
  phicube.SetChAMotor1State(digitalRead(INCENC_M1_CHA));
}


/**
 * @brief Set the current state for Ch B Motor 1 incremental encoder depending on the corresponding pin status.
 * 
 */
void SetChBMotor1State()
{
  phicube.SetChBMotor1State(digitalRead(INCENC_M1_CHB));
}


/**
 * @brief Set the current state for Ch A Motor 2 incremental encoder depending on the corresponding pin status.
 * 
 */
void SetChAMotor2State()
{
  phicube.SetChAMotor2State(digitalRead(INCENC_M2_CHA));
}


/**
 * @brief Set the current state for Ch B Motor 2 incremental encoder depending on the corresponding pin status.
 * 
 */
void SetChBMotor2State()
{
  phicube.SetChBMotor2State(digitalRead(INCENC_M2_CHB));
}


/**
 * @brief Enables the logic level shifter installed on the control board if requested.
 * 
 */
void ShifterSetup()
{
  if (PHICUBE_ENABLE_SHIFTER) {
    pinMode(OE_SHIFTER, OUTPUT);
    digitalWrite(OE_SHIFTER, HIGH);
  }
}


/**
 * @brief Sets all the interrupts for the on-board incremental encoders.
 * 
 */
void EncoderSetup()
{
  attachInterrupt(INCENC_M1_CHA, SetChAMotor1State, CHANGE);
  attachInterrupt(INCENC_M1_CHB, SetChBMotor1State, CHANGE);
  attachInterrupt(INCENC_M2_CHA, SetChAMotor2State, CHANGE);
  attachInterrupt(INCENC_M2_CHB, SetChBMotor2State, CHANGE);

  pinMode(INCENC_M1_CHA, INPUT_PULLUP);
  pinMode(INCENC_M1_CHB, INPUT_PULLUP);
  pinMode(INCENC_M2_CHA, INPUT_PULLUP);
  pinMode(INCENC_M2_CHB, INPUT_PULLUP);
}


/**
 * @brief Sets all the Serial and I2C communication pipelines.
 * 
 */
void CommSetup()
{
  Serial.begin(PHICUBE_SERIAL_BAUDRATE);
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
}



/**
 * @brief Initilizes all the objects and starts the threads. This function will run once at the MCU startup
 * 
 */
void setup()
{ 
  CommSetup();
  ShifterSetup();
  EncoderSetup();

  phicube.Init();
  phicubeRosComm.Init(phicube);
  
  RobotThread.start(PhicubeRobotCtrlThread);
  CommThread.start(PhicubeRosCommThread);
  StatusThread.start(PhicubeStatusUpdateThread);
  if (PHICUBE_ENABLE_HAPTICS) {
    HapticsThread.start(PhicubeHapticsCtrlThread);
  }
}


/**
 * @brief This function remains empty to let the ChRt threads run
 * 
 */
void loop() {}