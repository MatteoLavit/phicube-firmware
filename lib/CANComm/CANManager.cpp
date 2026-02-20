#include "CANManager.h"

// -----------------------------------------------------
// ---------------------- PUBLIC -----------------------
// -----------------------------------------------------

CANManager::CANManager() :
  queueHead(0),
  queueTail(0)
{
  CAN.begin(CanBitRate::BR_1000k);
}


bool CANManager::EnqueueCANMessage(CanMsg msg, void* context, CANResponseCB cb)
{
  if (IsQueueFull()) {
    return false;
  }

  queue[queueTail].msg = msg;
  queue[queueTail].context = context;
  queue[queueTail].callback = cb;
  queue[queueTail].retries = 0;
  queue[queueTail].timestamp = 0;

  queueTail = (queueTail+1) % CANCOMM_QUEUE_SIZE;
  return true;
}

bool CANManager::WriteCANMessage(CanMsg msg)
{
  return CAN.write(msg) > 0;
}

bool CANManager::IsQueueEmpty() const
{
  return queueHead == queueTail;
}

bool CANManager::IsQueueFull() const
{
  return ((queueTail+1) % CANCOMM_QUEUE_SIZE) == queueHead;
}

void CANManager::Update()
{
  //TODO: Se si accumulano molte risposte mi blocco qui? Capire se mettere un massimo di letture per Update.
  while (CAN.available()) {
    CanMsg msg = CAN.read();
    HandleNext(msg);
  }

  if (IsQueueEmpty()) return;

  CanCommand& cmd = queue[queueHead];
  if (millis()-cmd.timestamp >= CANCOMM_TIMEOUT_MS) {
    if (cmd.retries < CANCOMM_MAX_RETRIES) {
      cmd.retries++;
      SendNext();
    }
    else {
      queueHead = (queueHead+1) % CANCOMM_QUEUE_SIZE;
      SendNext();
    }
  }
}

// -----------------------------------------------------
// ---------------------- PRIVATE ----------------------
// -----------------------------------------------------

void CANManager::HandleNext(const CanMsg& msg)
{
  if (IsQueueEmpty()) return;
  CanCommand& cmd = queue[queueHead];
  if (msg.data[0] == cmd.msg.data[0]) {
    if (cmd.callback) cmd.callback(cmd.context, msg);
    queueHead = (queueHead+1) % CANCOMM_QUEUE_SIZE;
    SendNext();
  }
}

bool CANManager::SendNext()
{
  if (IsQueueEmpty()) return false;
  CanCommand& cmd = queue[queueHead];
  cmd.timestamp = millis();
  bool succ = WriteCANMessage(cmd.msg);
  return succ;
}