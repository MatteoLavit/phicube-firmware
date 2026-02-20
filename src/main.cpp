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


Thread RobotCtrlThread;
Thread RobotStateThread;
Thread MotorsStateThread;

PhicubeRobot phicubeRobot;
PhicubeCommunication phicubeCommunication;


void RobotCtrlThreadCB()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_ROBOTCTRL_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeRobot.UpdateRobotCtrl();
  }
}

void RobotStateThreadCB()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_ROBOTSTATE_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeCommunication.UpdateRobotState();
  }
}

void MotorsStateThreadCB()
{
  Clock::time_point tp = Clock::now();

  while (1) {
    tp += Clock::duration(PHICUBE_MOTORSTATE_PERIOD_MS);
    ThisThread::sleep_until(tp);
    phicubeCommunication.UpdateMotorsState();
  }
}

void setup()
{ 
  phicubeRobot.Init();
  phicubeCommunication.Init(phicubeRobot);
  
  RobotCtrlThread.start(RobotCtrlThreadCB);
  RobotStateThread.start(RobotStateThreadCB);
  MotorsStateThread.start(MotorsStateThreadCB);
}

void loop() {}