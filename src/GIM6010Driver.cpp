#include "GIM6010Driver.h"

// -----------------------------------------------------
// ---------------------- PUBLIC -----------------------
// -----------------------------------------------------

GIM6010Driver::GIM6010Driver(uint32_t id, CANManager& manager)
  : canManager(manager),
    canID(id) {
  currError = GIM6010Error::NO_ERROR;
}

bool GIM6010Driver::Init() {
  if (!ReadVersions()) return false;
  if (!ReadInfos()) return false;
  if (!ReadPosKp()) return false;
  if (!ReadPosKi()) return false;
  if (!ReadVelKp()) return false;
  if (!ReadVelKi()) return false;

  return true;
}

bool GIM6010Driver::ReadVersions() {
  uint8_t data[] = {GIM6010Command::READ_VERSIONS};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ReadVersionsCB)) return false;

  return true;
}

int GIM6010Driver::GetBootVersion() {
  return bootVersion;
}

int GIM6010Driver::GetSWVersion() {
  return SWVersion;
}

int GIM6010Driver::GetHWVersion() {
  return HWVersion;
}

int GIM6010Driver::GetCANVersion() {
  return CANVersion;
}

bool GIM6010Driver::ReadInfos() {
  uint8_t data[] = { GIM6010Command::READ_INFOS };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ReadInfosCB)) return false;

  return true;
}

int GIM6010Driver::GetPolePairs() {
  return polePairs;
}

int GIM6010Driver::GetReductionRatio() {
  return reductionRatio;
}

float GIM6010Driver::GetTorqueConstant() {
  return torqueConstant;
}

bool GIM6010Driver::ReadStatus() {
  uint8_t data[] = { GIM6010Command::READ_STATUS };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ReadStatusCB)) return false;

  return true;
}

float GIM6010Driver::GetBusVoltage() {
  return busVoltage;
}

float GIM6010Driver::GetBusCurrent() {
  return busCurrent;
}

int GIM6010Driver::GetTemperature() {
  return currTemperature;
}

GIM6010Driver::GIM6010Mode GIM6010Driver::GetMode() {
  return currMode;
}

bool GIM6010Driver::ClearFault() {
  uint8_t data[] = { GIM6010Command::CLEAR_FAULT };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ClearFaultCB)) return false;

  return true;
}

GIM6010Driver::GIM6010Error GIM6010Driver::GetError() {
  return currError;
}

bool GIM6010Driver::RestartDriver() {
  uint8_t data[] = { GIM6010Command::RESTART_SLAVE, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.WriteCANMessage(sndMsg)) return false;

  return true;
}

bool GIM6010Driver::ReadPosition() {
  uint8_t data[] = { GIM6010Command::READ_POSITION };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ReadPositionCB)) return false;

  return true;
}

float GIM6010Driver::GetSTPosition() {
  return currSTPosition;
}

float GIM6010Driver::GetMTPosition() {
  return currMTPosition;
}

bool GIM6010Driver::ReadVelocity() {
  uint8_t data[] = { GIM6010Command::READ_VELOCITY };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ReadVelocityCB)) return false;

  return true;
}

float GIM6010Driver::GetVelocity() {
  return currVelocity;
}

bool GIM6010Driver::ReadCurrent() {
  uint8_t data[] = { GIM6010Command::READ_CURRENT };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::ReadCurrentCB)) return false;

  return true;
}

float GIM6010Driver::GetCurrent() {
  return currCurrent;
}

float GIM6010Driver::GetTorque() {
  return currTorque;
}

bool GIM6010Driver::SetOrigin() {
  uint8_t data[] = { GIM6010Command::SET_ORIGIN };
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetOriginCB)) return false;

  return true;
}

float GIM6010Driver::GetOffset() {
  return mechOffset;
}

bool GIM6010Driver::SetPosMaxSpeed(float maxSpeed)  // rad/s
{
  uint8_t data[5];
  uint32_t mS = maxSpeed * 60.0 * 100 / (2 * PI);
  data[0] = GIM6010Command::SET_MAXVEL;
  CANUtils::ComposeU32LE(data, 1, mS);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetPosMaxSpeedCB)) return false;

  return true;
}

bool GIM6010Driver::SetVelMaxAcceleration(float maxAcceleration) // rad/s^2
{
  uint8_t data[5];
  uint32_t mA = maxAcceleration * 60.0 * 100 / (2 * PI);
  data[0] = GIM6010Command::SET_ACCEL;
  CANUtils::ComposeU32LE(data, 1, mA);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetVelMaxAccelerationCB)) return false;

  return true;
}

bool GIM6010Driver::SetPosVelMaxCurrent(float maxCurrent) // A
{
  uint8_t data[5];
  uint32_t mC = maxCurrent * 1000;
  data[0] = GIM6010Command::SET_MAXCURR;
  CANUtils::ComposeU32LE(data, 1, mC);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetPosVelMaxCurrentCB)) return false;

  return true;
}

bool GIM6010Driver::SetPosVelMaxTorque(float maxTorque) // Nm
{
  if(!SetPosVelMaxCurrent(maxTorque/torqueConstant)) return false;
  return true;
}

bool GIM6010Driver::SetQMaxDCurrent(float maxDCurrent) // A/s
{
  uint8_t data[5];
  uint32_t mDC = maxDCurrent * 1000;
  data[0] = GIM6010Command::SET_MAXDCURR;
  CANUtils::ComposeU32LE(data, 1, mDC);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetQMaxDCurrentCB)) return false;

  return true;
}

bool GIM6010Driver::SetQMaxDTorque(float maxDTorque) // Nm/s
{
  if(!SetQMaxDCurrent(maxDTorque/torqueConstant)) return false;
  return true;
}


bool GIM6010Driver::SetPosKp(float posKp)
{
  uint8_t data[5];
  data[0] = GIM6010Command::SET_POSKP;
  CANUtils::ComposeFloatLE(data, 1, posKp);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetPosKpCB)) return false;

  return true;
}

bool GIM6010Driver::ReadPosKp()
{
  uint8_t data[] = {GIM6010Command::SET_POSKP};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetPosKpCB)) return false;

  return true;
}

float GIM6010Driver::GetPosKp()
{
  return posKp;
}

bool GIM6010Driver::SetPosKi(float posKi)
{
  uint8_t data[5];
  data[0] = GIM6010Command::SET_POSKI;
  CANUtils::ComposeFloatLE(data, 1, posKi);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetPosKiCB)) return false;

  return true;
}

bool GIM6010Driver::ReadPosKi()
{
  uint8_t data[] = {GIM6010Command::SET_POSKI};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetPosKiCB)) return false;

  return true;
}

float GIM6010Driver::GetPosKi()
{
  return posKi;
}

bool GIM6010Driver::SetVelKp(float velKp)
{
  uint8_t data[5];
  data[0] = GIM6010Command::SET_VELKP;
  CANUtils::ComposeFloatLE(data, 1, velKp);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetVelKpCB)) return false;

  return true;
}

bool GIM6010Driver::ReadVelKp()
{
  uint8_t data[] = {GIM6010Command::SET_VELKP};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetVelKpCB)) return false;

  return true;
}

float GIM6010Driver::GetVelKp()
{
  return velKp;
}

bool GIM6010Driver::SetVelKi(float velKi)
{
  uint8_t data[5];
  data[0] = GIM6010Command::SET_VELKI;
  CANUtils::ComposeFloatLE(data, 1, velKi);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetVelKiCB)) return false;

  return true;
}

bool GIM6010Driver::ReadVelKi()
{
  uint8_t data[] = {GIM6010Command::SET_VELKI};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, &GIM6010Driver::SetVelKiCB)) return false;

  return true;
}

float GIM6010Driver::GetVelKi()
{
  return velKi;
}

bool GIM6010Driver::SetCurrentTarget(float currentTarget) // A
{
  uint8_t data[5];
  int32_t cT = currentTarget * 1000;
  data[0] = GIM6010Command::CURR_CTRL;
  CANUtils::ComposeS32LE(data, 1, cT);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, NULL)) return false;

  return true;
}

bool GIM6010Driver::SetTorqueTarget(float torqueTarget) // Nm
{
  if (!SetCurrentTarget(torqueTarget/torqueConstant)) return false;
  return true;
}

bool GIM6010Driver::SetSpeedTarget(float speedTarget) // rad/s
{
  uint8_t data[5];
  int32_t sT = speedTarget * 60.0 * 100 / (2 * PI);
  data[0] = GIM6010Command::VEL_CTRL;
  CANUtils::ComposeS32LE(data, 1, sT);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, NULL)) return false;

  return true;
}

bool GIM6010Driver::SetAbsolutePositionTarget(float absolutePositionTarget) // rad
{
  uint8_t data[5];
  int32_t apT = absolutePositionTarget * cpr / (2 * PI);  // rad;
  data[0] = GIM6010Command::ABSPOS_CTRL;
  CANUtils::ComposeS32LE(data, 1, apT);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, NULL)) return false;

  return true;
}

bool GIM6010Driver::SetRelativePositionTarget(float relativePositionTarget) // rad
{
  uint8_t data[5];
  int32_t rpT = relativePositionTarget * cpr / (2 * PI);  // rad;
  data[0] = GIM6010Command::RELPOS_CTRL;
  CANUtils::ComposeS32LE(data, 1, rpT);
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, NULL)) return false;

  return true;
}

bool GIM6010Driver::GoHomeByShortest()
{
  uint8_t data[] = {GIM6010Command::BACK_HOME};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, NULL)) return false;

  return true;
}

bool GIM6010Driver::MotorOff()
{
  uint8_t data[] = {GIM6010Command::MOTOR_OFF};
  CanMsg sndMsg(canID, sizeof(data), data);
  if (!canManager.EnqueueCANMessage(sndMsg, this, NULL)) return false;

  return true;
}

// -----------------------------------------------------
// ---------------------- CALLBACKS --------------------
// -----------------------------------------------------

void GIM6010Driver::ReadVersionsCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnReadVersions(msg);
}

void GIM6010Driver::ReadInfosCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnReadInfos(msg);
}

void GIM6010Driver::ReadStatusCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnReadStatus(msg);
}

void GIM6010Driver::ReadPositionCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnReadPosition(msg);
}

void GIM6010Driver::ReadVelocityCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnReadVelocity(msg);
}

void GIM6010Driver::ReadCurrentCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnReadCurrent(msg);
}

void GIM6010Driver::ClearFaultCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnClearFault(msg);
}

void GIM6010Driver::SetOriginCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetOrigin(msg);
}

void GIM6010Driver::SetPosMaxSpeedCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetPosMaxSpeed(msg);
}

void GIM6010Driver::SetVelMaxAccelerationCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetVelMaxAcceleration(msg);
}

void GIM6010Driver::SetPosVelMaxCurrentCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetPosVelMaxCurrent(msg);
}

void GIM6010Driver::SetQMaxDCurrentCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetQMaxDCurrent(msg);
}

void GIM6010Driver::SetPosKpCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetPosKp(msg);
}

void GIM6010Driver::SetPosKiCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetPosKi(msg);
}

void GIM6010Driver::SetVelKpCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetVelKp(msg);
}

void GIM6010Driver::SetVelKiCB(void* context, const CanMsg& msg) {
  auto* self = static_cast<GIM6010Driver*>(context);
  self->OnSetVelKi(msg);
}

// -----------------------------------------------------
// ---------------------- PRIVATE ----------------------
// -----------------------------------------------------

void GIM6010Driver::OnReadVersions(const CanMsg& msg) {
  bootVersion = CANUtils::ParseU16LE(msg.data, 1);
  SWVersion = CANUtils::ParseU16LE(msg.data, 3);
  HWVersion = CANUtils::ParseU16LE(msg.data, 5);
  CANVersion = msg.data[7];
}

void GIM6010Driver::OnReadInfos(const CanMsg& msg) {
  polePairs = msg.data[1];
  torqueConstant = CANUtils::ParseFloatLE(msg.data, 2);  // Nm/A
  reductionRatio = msg.data[6];
}

void GIM6010Driver::OnReadStatus(const CanMsg& msg) {
  busVoltage = CANUtils::ParseU16LE(msg.data, 1) * 0.01;  // V
  busCurrent = CANUtils::ParseU16LE(msg.data, 3) * 0.01;  // A
  currTemperature = msg.data[5];
  currMode = (GIM6010Mode)msg.data[6];
  currError = (GIM6010Error)msg.data[7];
}

void GIM6010Driver::OnReadPosition(const CanMsg& msg) {
  currSTPosition = CANUtils::ParseU16LE(msg.data, 1) * (2 * PI) / cpr;  // rad
  currMTPosition = CANUtils::ParseS32LE(msg.data, 3) * (2 * PI) / cpr;  // rad
}

void GIM6010Driver::OnReadVelocity(const CanMsg& msg) {
  currVelocity = CANUtils::ParseS32LE(msg.data, 1) * 0.01 * (2 * PI) / 60.0;  // rad/s
}

void GIM6010Driver::OnReadCurrent(const CanMsg& msg) {
  currCurrent = CANUtils::ParseS32LE(msg.data, 1) * 0.001;  // A
  currTorque = currCurrent * torqueConstant;                // Nm
}

void GIM6010Driver::OnClearFault(const CanMsg& msg) {
  currError = (GIM6010Error)msg.data[1];
}

void GIM6010Driver::OnSetOrigin(const CanMsg& msg) {
  mechOffset = CANUtils::ParseU16LE(msg.data, 1) * (2 * PI) / cpr;  // rad
}

void GIM6010Driver::OnSetPosMaxSpeed(const CanMsg& msg) {
  posMaxSpeed = CANUtils::ParseU32LE(msg.data, 1) * (2 * PI) / (60.0 * 100.0);  // rad/s
}

void GIM6010Driver::OnSetVelMaxAcceleration(const CanMsg& msg) {
  velMaxAcceleration = CANUtils::ParseU32LE(msg.data, 1) * (2 * PI) / (60.0 * 100.0);  // rad/s^2
}

void GIM6010Driver::OnSetPosVelMaxCurrent(const CanMsg& msg) {
  posVelMaxCurrent = CANUtils::ParseU32LE(msg.data, 1) / 1000.0;;  // A
  posVelMaxTorque = posVelMaxCurrent * torqueConstant;
}

void GIM6010Driver::OnSetQMaxDCurrent(const CanMsg& msg) {
  qMaxDCurrent = CANUtils::ParseU32LE(msg.data, 1) / 1000.0;;  // A
  qMaxDTorque = qMaxDCurrent * torqueConstant;
}

void GIM6010Driver::OnSetPosKp(const CanMsg& msg) {
  posKp = CANUtils::ParseFloatLE(msg.data, 1);
}

void GIM6010Driver::OnSetPosKi(const CanMsg& msg) {
  posKi = CANUtils::ParseFloatLE(msg.data, 1);
}

void GIM6010Driver::OnSetVelKp(const CanMsg& msg) {
  velKp = CANUtils::ParseFloatLE(msg.data, 1);
}

void GIM6010Driver::OnSetVelKi(const CanMsg& msg) {
  velKi = CANUtils::ParseFloatLE(msg.data, 1);
}