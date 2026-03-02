#include "CANManager.h"

// -----------------------------------------------------
// ---------------------- PUBLIC -----------------------
// -----------------------------------------------------


CANManager::CANManager() :
  queueHead(0),
  queueTail(0) {}

bool CANManager::Init()
{
  return CAN.begin(CanBitRate::BR_1000k);
}


bool CANManager::EnqueueCANMessage(CanMsg msg, void* context, CANResponseCB cb)
{
  if (IsQueueFull()) return false;
  if (IsAlreadyPending(msg.id, msg.data[0])) return false;

  queue[queueTail].msg = msg;
  queue[queueTail].context = context;
  queue[queueTail].callback = cb;
  queue[queueTail].timestamp = 0;
  queue[queueTail].waitingReply = false;

  queueTail = (queueTail+1) % CANCOMM_QUEUE_SIZE;
  return true;
}


void CANManager::Update()
{
  HandleRX();
  HandleTX();
  CheckTimeouts();
}


// -----------------------------------------------------
// ---------------------- PRIVATE ----------------------
// -----------------------------------------------------


bool CANManager::IsQueueFull() const
{
  return ((queueTail+1) % CANCOMM_QUEUE_SIZE) == queueHead;
}


bool CANManager::IsMatching(const CanCommand& cmd, const CanMsg& msg)
{
  return (cmd.msg.id == 0x100 + msg.id) && (cmd.msg.data[0] == msg.data[0]);
}


bool CANManager::IsAlreadyPending(uint32_t id, uint8_t opCode)
{
  int idx = queueHead;
  while (idx != queueTail) {
    CanCommand& cmd = queue[idx];
    if (cmd.msg.id == id && cmd.msg.data[0] == opCode) {
      return true;
    }
    idx = (idx + 1) % CANCOMM_QUEUE_SIZE;
  }
  return false;
}


void CANManager::RemoveFromQueue(int idx)
{
  int next = (idx + 1) % CANCOMM_QUEUE_SIZE;
  while (next != queueTail) {
    queue[idx] = queue[next];
    idx = next;
    next = (next + 1) % CANCOMM_QUEUE_SIZE; 
  }
  queueTail = (queueTail - 1 + CANCOMM_QUEUE_SIZE) % CANCOMM_QUEUE_SIZE;
}


void CANManager::CheckTimeouts()
{
  uint32_t now = millis();
  int idx = queueHead;
  while (idx != queueTail) {
    CanCommand& cmd = queue[idx];
    if (cmd.waitingReply && (now - cmd.timestamp > CANCOMM_TIMEOUT_MS)) {
      RemoveFromQueue(idx);
      return;
    }
    idx = (idx + 1) % CANCOMM_QUEUE_SIZE;
  }
}


void CANManager::HandleRX()
{
  if (!CAN.available()) return;
  CanMsg msg = CAN.read();
  
  int idx = queueHead;
  while (idx != queueTail) {
    CanCommand& cmd = queue[idx];
    if (cmd.waitingReply && IsMatching(cmd, msg)) {
      if (cmd.callback) {
        cmd.callback(cmd.context, msg);
      }
      RemoveFromQueue(idx);
      return;
    }
    idx = (idx + 1) % CANCOMM_QUEUE_SIZE;
  }
  //NOTA: Se arrivo qui ho ricevuto una risposta non attesa. Aggiungere contatore errori o log su rosout?
}


void CANManager::HandleTX()
{
  int idx = queueHead;
  while (idx != queueTail) {
    if (!queue[idx].waitingReply) {
      if (CAN.write(queue[idx].msg) > 0) {
        if (!queue[idx].callback) {
          RemoveFromQueue(idx);
        }
        else {
          queue[idx].timestamp = millis();
          queue[idx].waitingReply = true;
        }
        return;
      }
      else {
        return; // Se non riesco a scrivere tengo il messaggio in coda e salto il giro per lasciare che il bus si liberi
      }
    }
    idx = (idx + 1) % CANCOMM_QUEUE_SIZE;
  }
}