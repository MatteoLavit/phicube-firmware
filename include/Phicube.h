/**
 * @file Phicube.h
 * @author Matteo Lavit Nicora (matteo.lavit@rehabiliatechnologies.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef PHICUBE
#define PHICUBE


#include <Wire.h>
#include <LSM6.h>
#include "PhicubeRobot.h"
#include "PhicubeLed.h"


class Phicube
{
    public:

        /**
         * @brief The three possible orientations of phicube main body.
         * 
         */
        enum PhicubeOrientation {
            FRONTAL,
            VERTICAL,
            SAGITTAL
        };

        /**
         * @brief Construct a new Phicube object
         * 
         */
        Phicube();

        /**
         * @brief Calls the initialization of the robot and decoration objects.
         * 
         */
        void Init();

        /**
         * @brief Call the update of the robot object.
         * 
         */
        void UpdateRobot();

        /**
         * @brief Call the update of the decoration object.
         * 
         */
        void UpdateLed();

        /**
         * @brief Get the current state set for the robot object
         * 
         * @return PhicubeRobot::RobotState 
         */
        PhicubeRobot::RobotState GetRobotState();

        /**
         * @brief Get the current mode set for the robot object.
         * 
         * @return PhicubeRobot::RobotMode 
         */
        PhicubeRobot::RobotMode GetRobotMode();

        /**
         * @brief Get the current error known for the robot object.
         * 
         * @return PhicubeRobot::RobotError 
         */
        PhicubeRobot::RobotError GetRobotError();

        /**
         * @brief Get the current motors position.
         * 
         * @param pos Pointer to an array that gets filled with the position of phicube motors.
         */
        void GetCurrentPosition(float(&pos)[2]);

        /**
         * @brief Get the current motors velocity.
         * 
         * @param vel Pointer to an array that gets filled with the velocity of phicube motors.
         */
        void GetCurrentVelocity(float(&vel)[2]);

        /**
         * @brief Get the current motors effort.
         * 
         * @param eff Pointer to an array that gets filled with the effor of phicube motors.
         */
        void GetCurrentEffort(float(&eff)[2]);

        /**
         * @brief Get the overheat index of motors.
         * 
         * @param overheat_index Pointer to an array that gets filled with the overheat indexes.
         */
        void GetOverheatIndex(float(&overheat_index)[2]);

        /**
         * @brief Get the redundant encoders differences.
         * 
         * @param encoders_diff Pointer to an array that gets filled with the encoders differences.
         */
        void GetEncodersDiff(float(&encoders_diff)[2]);

        /**
         * @brief Set the state of Ch A Motor 1 incremental encoder as requested.
         * 
         * @param state Requested state of Ch A Motor 1 incremental encoder.
         */
        void SetChAMotor1State(bool state);

        /**
         * @brief Set the state of Ch B Motor 1 incremental encoder as requested.
         * 
         * @param state Requested state of Ch B Motor 1 incremental encoder.
         */
        void SetChBMotor1State(bool state);

        /**
         * @brief Set the state of Ch A Motor 2 incremental encoder as requested.
         * 
         * @param state Requested state of Ch A Motor 2 incremental encoder.
         */
        void SetChAMotor2State(bool state);

        /**
         * @brief Set the state of Ch B Motor 2 incremental encoder as requested.
         * 
         * @param state Requested state of Ch B Motor 2 incremental encoder.
         */
        void SetChBMotor2State(bool state);

        /**
         * @brief Set the parameters of the reference PID velocity estimator of the bimanual controller as requested.
         * 
         * @param kp1 Requested proportional gain for motor 1.
         * @param ki1 Requested integral gain for motor 1.
         * @param kd1 Requested derivative gain for motor 1.
         * @param kp2 Requested proportional gain for motor 2.
         * @param ki2 Requested integral gain for motor 2.
         * @param kd2 Requested derivative gain for motor 2.
         */
        void SetPIDBimRefVelEstParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2);

        /**
         * @brief Set the parameters of the reference PID velocity estimator of the assistive controller as requested.
         * 
         * @param kp1 Requested proportional gain for motor 1.
         * @param ki1 Requested integral gain for motor 1.
         * @param kd1 Requested derivative gain for motor 1.
         * @param kp2 Requested proportional gain for motor 2.
         * @param ki2 Requested integral gain for motor 2.
         * @param kd2 Requested derivative gain for motor 2.
         */
        void SetPIDAssRefVelEstParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2);

        /**
         * @brief Set the parameters of the internal PID velocity estimator as requested.
         * 
         * @param kp1 Requested proportional gain for motor 1.
         * @param ki1 Requested integral gain for motor 1.
         * @param kd1 Requested derivative gain for motor 1.
         * @param kp2 Requested proportional gain for motor 2.
         * @param ki2 Requested integral gain for motor 2.
         * @param kd2 Requested derivative gain for motor 2.
         */
        void SetPIDVelocityEstParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2);

        /**
         * @brief Set the parameters of the PID position controller as requested.
         * 
         * @param kp1 Requested proportional gain for motor 1.
         * @param ki1 Requested integral gain for motor 1.
         * @param kd1 Requested derivative gain for motor 1.
         * @param kp2 Requested proportional gain for motor 2.
         * @param ki2 Requested integral gain for motor 2.
         * @param kd2 Requested derivative gain for motor 2.
         */
        void SetPIDPositionParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2);

        /**
         * @brief Set the parameters of the PID velocity controller as requested.
         * 
         * @param kp1 Requested proportional gain for motor 1.
         * @param ki1 Requested integral gain for motor 1.
         * @param kd1 Requested derivative gain for motor 1.
         * @param kp2 Requested proportional gain for motor 2.
         * @param ki2 Requested integral gain for motor 2.
         * @param kd2 Requested derivative gain for motor 2.
         */
        void SetPIDVelocityParameters(float kp1, float ki1, float kd1, float kp2, float ki2, float kd2);

        /**
         * @brief Request homing procedure of the incremental encoders.
         * 
         * @param reqMot1 True if homing procedure is requested for motor 1.
         * @param reqMot2 True if homing procedure is requested for motor 2.
         */
        void SetHoming(bool reqMot1, bool reqMot2);

        /**
         * @brief Set the requested current state for the on-board LED rings.
         * 
         * @param state Requested effect type.
         * @param index Index of the LED ring on which the effect should be applied.
         * @param brightness Brightness level (0-255).
         * @param speed Speed of the effect (ms).
         * @param timeout True if the effect should end after a certain duration.
         * @param duration Duration of the effect (ms).
         * @param colorR Level of red for the RGB color code (0-255).
         * @param colorG Level of green for the RGB color code (0-255).
         * @param colorB Level of blue for the RGB color code (0-255).
         */
        void SetLedState(LedHW::LedState state, int index, int brightness, int speed, bool timeout, int duration, int colorR, int colorG, int colorB);

        /**
         * @brief Request controller activation by index.
         * 
         * @param ctrl Index of requested controller.
         */
        void SetController(int ctrl);

        /**
         * @brief Set the target values that are then interpreted by the active controller.
         * 
         * @param tgt1 Float target value 1.
         * @param tgt2 Float target value 2.
         */
        void SetTarget(float tgt1, float tgt2);

        /**
         * @brief Set the parameters for the decoupled controller.
         * 
         * @param k1 Requested virtual stiffness for motor 1.
         * @param k2 Requested virtual damping for motor 2.
         * @param d1 Requested virtual stiffness for motor 1.
         * @param d2 Requested virtual damping for motor 2.
         */
        void SetDecoupledParams(float k1, float k2,float d1, float d2);

        /**
         * @brief Set the parameters for the coupled controller.
         * 
         * @param k1a Requested stiffness for the exercise assistance system of motor 1.
         * @param k2a Requested stiffness for the exercise assistance system of motor 2.
         * @param d1a Requested damping for the exercise assistance system of motor 1.
         * @param d2a Requested damping for the exercise assistance system of motor 2.
         * @param k1b Requested stiffness bilateral assistance system of motor 1.
         * @param k2b Requested stiffness bilateral assistance system of motor 2.
         * @param d1b Requested damping bilateral assistance system of motor 1.
         * @param d2b Requested damping bilateral assistance system of motor 2.
         * @param rho Requested transmission ratio of side 1 over side 2.
         * @param beta Requested bilateral dominance of side 1 over side 2.
         */
        void SetCoupledParams(float k1a, float k2a,float d1a, float d2a, float k1b, float k2b,float d1b, float d2b, float rho, float beta);

        /**
         * @brief Get the current orientation of phicube central body
         * 
         * @return PhicubeOrientation 
         */
        PhicubeOrientation GetOrientation();

    private:

        /**
         * @brief The Phicube robot object.
         * 
         */
        PhicubeRobot phicubeRobot;

        /**
         * @brief The Phicube decoration object.
         * 
         */
        PhicubeLed phicubeLed;
        
        /**
         * @brief Object for interacting with the on-board IMU sensor.
         * 
         */
        LSM6 imu;
};


#endif