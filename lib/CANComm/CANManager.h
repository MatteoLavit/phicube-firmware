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

    bool Init();
    void Update();

  private:

    struct CanCommand {
      CanMsg msg;
      void* context;
      CANResponseCB callback;
      uint32_t timestamp;
      bool waitingReply;
    };

    CanCommand queue[CANCOMM_QUEUE_SIZE];
    uint8_t queueHead;
    uint8_t queueTail;

    bool IsQueueFull() const;
    bool IsMatching(const CanCommand& cmd, const CanMsg& msg);
    bool IsAlreadyPending(uint32_t id, uint8_t opCode);

    void RemoveFromQueue(int idx);
    void CheckTimeouts();

    void HandleRX();
    void HandleTX();
};

#endif