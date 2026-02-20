#ifndef GIM6010_DRIVER
#define GIM6010_DRIVER

#include "CANUtils.h"
#include "CANManager.h"

class GIM6010Driver
{
  public:

    enum GIM6010Command {
      //System commmands
      RESTART_SLAVE = 0x00,
      READ_VERSIONS = 0xA0,
      READ_CURRENT = 0xA1,
      READ_VELOCITY = 0xA2,
      READ_POSITION = 0xA3,
      READ_STATUS = 0xAE,
      CLEAR_FAULT = 0xAF,
      //Parameter commands
      READ_INFOS = 0xB0,
      SET_ORIGIN = 0xB1,
      SET_MAXVEL = 0xB2,
      SET_MAXCURR = 0xB3,
      SET_MAXDCURR = 0xB4,
      SET_ACCEL = 0xB5,
      SET_POSKP = 0xB6,
      SET_POSKI = 0xB7,
      SET_VELKP = 0xB8,
      SET_VELKI = 0xB9,
      //Control commands
      CURR_CTRL = 0xC0,
      VEL_CTRL = 0xC1,
      ABSPOS_CTRL = 0xC2,
      RELPOS_CTRL = 0xC3,
      BACK_HOME = 0xC4,
      MOTOR_OFF = 0xCF
    };

    enum GIM6010Mode {
      OFF_MODE = 0,
      VOLTAGE_MODE = 1,
      CURRENT_MODE = 2,
      SPEED_MODE = 3,
      POSITION_MODE = 4
    };

    enum GIM6010Error {
      NO_ERROR = 0b00000000,
      VOLTAGE_ERROR = 0b10000000,
      CURRENT_ERROR = 0b01000000,
      TEMPERATURE_ERROR = 0b00100000,
      ENCODER_ERROR = 0b00010000,
      HARDWARE_ERROR = 0b00000010,
      SOFTWARE_ERROR = 0b00000001
    };

    GIM6010Driver(const uint32_t id, CANManager &manager);

    bool Init();
    
    bool ReadVersions();
    int GetBootVersion();
    int GetSWVersion();
    int GetHWVersion();
    int GetCANVersion();

    bool ReadInfos();
    int GetPolePairs();
    int GetReductionRatio();
    float GetTorqueConstant(); // Nm/A

    bool ReadStatus();
    float GetBusVoltage(); // V
    float GetBusCurrent(); // A
    int GetTemperature(); // °C
    GIM6010Mode GetMode();
    GIM6010Error GetError();
    
    bool ClearFault();
    bool RestartDriver();

    bool ReadPosition();
    float GetSTPosition(); // rad
    float GetMTPosition(); // rad

    bool ReadVelocity();
    float GetVelocity(); // rad/s

    bool ReadCurrent();
    float GetCurrent(); // A
    float GetTorque(); // Nm

    bool SetOrigin();
    float GetOffset(); // rad

    bool SetPosMaxSpeed(float maxSpeed); // rad/s
    float GetPosMaxSpeed();
    bool SetVelMaxAcceleration(float maxAcceleration); // rad/s^2
    float GetVelMaxAcceleration();
    bool SetPosVelMaxCurrent(float maxCurrent); // A
    float GetPosVelMaxCurrent();
    bool SetPosVelMaxTorque(float maxTorque); // Nm
    float GetPosVelMaxTorque();
    bool SetQMaxDCurrent(float maxDCurrent); // A/s
    float GetQMaxDCurrent();
    bool SetQMaxDTorque(float maxDTorque); // Nm/s
    float GetQMaxDTorque();

    bool SetPosKp(float posKp);
    bool ReadPosKp();
    float GetPosKp();
    bool SetPosKi(float posKi);
    bool ReadPosKi();
    float GetPosKi();
    bool SetVelKp(float velKp);
    bool ReadVelKp();
    float GetVelKp();
    bool SetVelKi(float velKi);
    bool ReadVelKi();
    float GetVelKi();

    //TODO - from here
    bool SetCurrentTarget(float currentTarget); // A
    bool SetTorqueTarget(float torqueTarget); // Nm
    bool SetSpeedTarget(float speedTarget); // rad/s
    bool SetAbsolutePositionTarget(float absolutePositionTarget); // rad
    bool SetRelativePositionTarget(float relativePositionTarget); // rad
    //TODO - till here

    bool GoHomeByShortest();
    bool MotorOff();

    static void ReadVersionsCB(void* context, const CanMsg& msg);
    static void ReadInfosCB(void* context, const CanMsg& msg);
    static void ReadStatusCB(void* context, const CanMsg& msg);
    static void ReadPositionCB(void* context, const CanMsg& msg);
    static void ReadVelocityCB(void* context, const CanMsg& msg);
    static void ReadCurrentCB(void* context, const CanMsg& msg);
    static void ClearFaultCB(void* context, const CanMsg& msg);
    static void SetOriginCB(void* context, const CanMsg& msg);
    static void SetPosMaxSpeedCB(void* context, const CanMsg& msg);
    static void SetVelMaxAccelerationCB(void* context, const CanMsg& msg);
    static void SetPosVelMaxCurrentCB(void* context, const CanMsg& msg);
    static void SetQMaxDCurrentCB(void* context, const CanMsg& msg);
    static void SetPosKpCB(void* context, const CanMsg& msg);
    static void SetPosKiCB(void* context, const CanMsg& msg);
    static void SetVelKpCB(void* context, const CanMsg& msg);
    static void SetVelKiCB(void* context, const CanMsg& msg);

  private:

    uint32_t canID;
    CANManager &canManager;

    void OnReadVersions(const CanMsg& msg);
    int bootVersion;
    int SWVersion;
    int HWVersion;
    int CANVersion;

    void OnReadInfos(const CanMsg& msg);
    int polePairs;
    int reductionRatio;
    float torqueConstant;

    void OnReadStatus(const CanMsg& msg);
    int currTemperature;
    float busVoltage;
    float busCurrent;
    GIM6010Mode currMode;

    void OnClearFault(const CanMsg& msg);
    GIM6010Error currError;

    void OnReadPosition(const CanMsg& msg);
    float currSTPosition;
    float currMTPosition;

    void OnReadVelocity(const CanMsg& msg);
    float currVelocity;

    void OnReadCurrent(const CanMsg& msg);
    float currCurrent;  
    float currTorque;

    void OnSetOrigin(const CanMsg &msg);
    float mechOffset; 

    void OnSetPosMaxSpeed(const CanMsg& msg);
    float posMaxSpeed;

    void OnSetVelMaxAcceleration(const CanMsg& msg);
    float velMaxAcceleration;

    void OnSetPosVelMaxCurrent(const CanMsg& msg);
    float posVelMaxCurrent;
    float posVelMaxTorque;  

    void OnSetQMaxDCurrent(const CanMsg& msg);
    float qMaxDCurrent;
    float qMaxDTorque;

    void OnSetPosKp(const CanMsg &msg);
    float posKp;

    void OnSetPosKi(const CanMsg &msg);
    float posKi;

    void OnSetVelKp(const CanMsg &msg);
    float velKp;

    void OnSetVelKi(const CanMsg &msg);
    float velKi;
};

#endif