#include <PGMWrap.h>

char * const fatal_messages[] = {

#define FATAL_BLOCKED   0
  "Fatal: Motor is blocked",

#define FATAL_AS5601    1
  "Fatal: AS5601 error",

#define FATAL_SEMAPHORE 2
  "Fatal: Semaphore error",

#define FATAL_HOMING 3
  "Fatal: Homing failed",

#define FATAL_USER 4
  "Fatal: USer action",

};


