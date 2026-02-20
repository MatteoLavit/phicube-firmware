#include "CANUtils.h"

namespace CANUtils
{
  uint16_t ParseU16LE(const uint8_t* d, uint8_t i)
  {
    return d[i] | (d[i + 1] << 8);
  }

  uint16_t ParseU16BE(const uint8_t* d, uint8_t i)
  {
    return (d[i] << 8) | d[i + 1];
  }

  int16_t ParseS16LE(const uint8_t* d, uint8_t i)
  {
    return (int16_t)ParseU16LE(d, i);
  }

  int16_t ParseS16BE(const uint8_t* d, uint8_t i)
  {
    return (int16_t)ParseU16BE(d, i);
  }

  uint32_t ParseU32LE(const uint8_t* d, uint8_t i)
  {
    return  (uint32_t)d[i] |
            ((uint32_t)d[i + 1] << 8) |
            ((uint32_t)d[i + 2] << 16) |
            ((uint32_t)d[i + 3] << 24);
  }

  uint32_t ParseU32BE(const uint8_t* d, uint8_t i)
  {
    return ((uint32_t)d[i] << 24) |
            ((uint32_t)d[i + 1] << 16) |
            ((uint32_t)d[i + 2] << 8) |
            (uint32_t)d[i + 3];
  }

  int32_t ParseS32LE(const uint8_t* d, uint8_t i)
  {
    return (int32_t)ParseU32LE(d, i);
  }

  int32_t ParseS32BE(const uint8_t* d, uint8_t i)
  {
    return (int32_t)ParseU32BE(d, i);
  }

  float ParseFloatLE(const uint8_t* d, uint8_t i)
  {
    uint32_t tmp = ParseS32LE(d, i);
    float f;
    memcpy(&f, &tmp, sizeof(float));
    return f;
  }

  float ParseFloatBE(const uint8_t* d, uint8_t i)
  {
    uint32_t tmp = ParseS32BE(d, i);
    float f;
    memcpy(&f, &tmp, sizeof(float));
    return f;
  }

  void ComposeU16LE(uint8_t* d, uint8_t i, uint16_t v)
  {
    d[i]     = v & 0xFF;
    d[i + 1] = (v >> 8) & 0xFF;
  }

  void ComposeU16BE(uint8_t* d, uint8_t i, uint16_t v)
  {
    d[i]     = (v >> 8) & 0xFF;
    d[i + 1] = v & 0xFF;
  }

  void ComposeS16LE(uint8_t* d, uint8_t i, int16_t v)
  {
    ComposeU16LE(d, i, (uint16_t)v);
  }

  void ComposeS16BE(uint8_t* d, uint8_t i, int16_t v)
  {
    ComposeU16BE(d, i, (uint16_t)v);
  }

  void ComposeU32LE(uint8_t* d, uint8_t i, uint32_t v)
  {
    d[i]     = v & 0xFF;
    d[i + 1] = (v >> 8) & 0xFF;
    d[i + 2] = (v >> 16) & 0xFF;
    d[i + 3] = (v >> 24) & 0xFF;
  }

  void ComposeU32BE(uint8_t* d, uint8_t i, uint32_t v)
  {
    d[i]     = (v >> 24) & 0xFF;
    d[i + 1] = (v >> 16) & 0xFF;
    d[i + 2] = (v >> 8) & 0xFF;
    d[i + 3] = v & 0xFF;
  }

  void ComposeS32LE(uint8_t* d, uint8_t i, int32_t v)
  {
    return ComposeU32LE(d, i, (uint32_t)v);
  }

  void ComposeS32BE(uint8_t* d, uint8_t i, int32_t v)
  {
    return ComposeU32BE(d, i, (uint32_t)v);
  }

  void ComposeFloatLE(uint8_t* d, uint8_t i, float v)
  {
    uint32_t tmp;
    memcpy(&tmp, &v, sizeof(float));
    ComposeU32LE(d, i, tmp);
  }

  void ComposeFloatBE(uint8_t* d, uint8_t i, float v)
  {
    uint32_t tmp;
    memcpy(&tmp, &v, sizeof(float));
    ComposeU32BE(d, i, tmp);
  }
}