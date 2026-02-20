#ifndef CAN_MANAGER
#define CAN_MANAGER

#include <Arduino_CAN.h>
#include "PhicubeConfiguration.h"

class CANManager
{
  public:

    typedef void (*CANResponseCB)(void* context, const CanMsg& response);

    CANManager();

    bool EnqueueCANMessage(CanMsg msg, void* context, CANResponseCB cb);
    bool WriteCANMessage(CanMsg msg);
    bool IsQueueEmpty() const;
    bool IsQueueFull() const;
    void Update();

  private:

    struct CanCommand {
      CanMsg msg;
      void* context;
      CANResponseCB callback;
      uint8_t retries;
      uint32_t timestamp;
    };

    CanCommand queue[CANCOMM_QUEUE_SIZE];
    uint8_t queueHead;
    uint8_t queueTail;

    void HandleNext(const CanMsg& msg);
    bool SendNext();
};

#endif