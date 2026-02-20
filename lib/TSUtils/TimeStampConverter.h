#ifndef TIMESTAMPCONVERTER
#define TIMESTAMPCONVERTER
#define micro_rollover_useconds 4294967295


#include <Arduino.h>


int ClockGetTime(clockid_t unused, struct timespec *tp);


#endif