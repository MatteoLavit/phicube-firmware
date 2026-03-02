/**
 * @file main.cpp
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief Main arduino script launching and running threads
 * @version 0.1
 * @date 2026-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <mbed.h>
#include <rtos.h>

#include "PhicubeConfiguration.h"
#include "PhicubeRobot.h"
#include "PhicubeCommunication.h"

using namespace mbed;
using namespace rtos;
using namespace rtos::Kernel;
using namespace std::chrono_literals;


Thread CANCommThread;
Thread RobotCtrlThread;
Thread ROSCommThread;

PhicubeRobot phicubeRobot;
PhicubeCommunication phicubeCommunication;


void CANCommThreadCB()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_CANCOMM_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeRobot.UpdateCANComm();
  }
}

void RobotCtrlThreadCB()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_ROBOTCTRL_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeRobot.UpdateRobotCtrl();
  }
}

void ROSCommThreadCB()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_ROSCOMM_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeCommunication.UpdateROSComm();
  }
}

void setup()
{ 
  phicubeCommunication.Init(phicubeRobot);
  ROSCommThread.start(ROSCommThreadCB);

  while (!phicubeRobot.InitManager()) delay(200);
  CANCommThread.start(CANCommThreadCB);

  while (!phicubeRobot.InitMotors()) delay(200);
  RobotCtrlThread.start(RobotCtrlThreadCB);
}

void loop() {}