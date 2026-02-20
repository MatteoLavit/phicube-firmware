#ifndef CAN_UTILS
#define CAN_UTILS

#include <cstring>
#include <stdint.h>

namespace CANUtils
{
  uint16_t ParseU16LE(const uint8_t* data, uint8_t index);
  uint16_t ParseU16BE(const uint8_t* data, uint8_t index);

  int16_t ParseS16LE(const uint8_t* data, uint8_t index);
  int16_t ParseS16BE(const uint8_t* data, uint8_t index);

  uint32_t ParseU32LE(const uint8_t* data, uint8_t index);
  uint32_t ParseU32BE(const uint8_t* data, uint8_t index);

  int32_t ParseS32LE(const uint8_t* data, uint8_t index);
  int32_t ParseS32BE(const uint8_t* data, uint8_t index);

  float ParseFloatLE(const uint8_t* data, uint8_t index);
  float ParseFloatBE(const uint8_t* data, uint8_t index);

  void ComposeU16LE(uint8_t* data, uint8_t index, uint16_t value);
  void ComposeU16BE(uint8_t* data, uint8_t index, uint16_t value);

  void ComposeS16LE(uint8_t* data, uint8_t index, int16_t value);
  void ComposeS16BE(uint8_t* data, uint8_t index, int16_t value);

  void ComposeU32LE(uint8_t* data, uint8_t index, uint32_t value);
  void ComposeU32BE(uint8_t* data, uint8_t index, uint32_t value);

  void ComposeS32LE(uint8_t* data, uint8_t index, int32_t value);
  void ComposeS32BE(uint8_t* data, uint8_t index, int32_t value);

  void ComposeFloatLE(uint8_t* data, uint8_t index, float value);
  void ComposeFloatBE(uint8_t* data, uint8_t index, float value);
}

#endif