#ifndef DCCOMMS_COMMSDEVICESERVICE_H_
#define DCCOMMS_COMMSDEVICESERVICE_H_

#include <condition_variable>
#include <dccomms/StreamCommsDevice.h>
#include <dccomms/IPacketBuilder.h>
#include <dccomms/Packet.h>
#include <dccomms/Utils.h>
#include <fcntl.h> /* Defines O_* constants */
#include <memory>
#include <mqueue.h>
#include <mutex>
#include <queue>
#include <sys/stat.h> /* Defines mode constants */
#include <thread>

namespace dccomms {

class CommsDeviceService;

typedef std::shared_ptr<CommsDeviceService> CommsDeviceServicePtr;

class CommsDeviceService : public StreamCommsDevice {
public:
  enum CommsDeviceServiceType { IPHY_TYPE_DLINK = 0, IPHY_TYPE_PHY };
  enum PhyState { BUSY = 0, READY };

  static CommsDeviceServicePtr
  BuildCommsDeviceService(PacketBuilderPtr pb, int iphytype = IPHY_TYPE_DLINK,
                          int maxframesize = 8191) {
    return CommsDeviceServicePtr(
        new CommsDeviceService(pb, iphytype, maxframesize));
  }

  CommsDeviceService(PacketBuilderPtr pb, int iphytype = IPHY_TYPE_DLINK,
                     int maxframesize = 8191);

  void SetMaxQueueSize(uint32_t size);
  uint32_t GetMaxQueueSize();
  void SetBlockingTransmission(bool v);
  virtual ~CommsDeviceService();

  virtual void ReadPacket(const PacketPtr &);
  virtual void WritePacket(const PacketPtr &);

  virtual void Start();
  virtual void Stop();

  virtual unsigned int GetRxFifoSize();

  // Methods only for type IPHY_TYPE_DLINK
  virtual bool BusyTransmitting();

  // Methods only for type IPHY_TYPE_PHY
  virtual void SetPhyLayerState(const PhyState &);
  // SetPhyLayerState cambia el estado de la capa fisica y se lo hace saber a la
  // capa de arriba

  // Two instances of CommsDeviceService for the same purpose in the same
  // machine (for debug reasons) must have different namespaces
  // This method must be called before Start
  virtual void SetCommsDeviceId(std::string nspace);

  void WaitForFramesFromRxFifo();
  bool WaitForFramesFromRxFifo(unsigned int timeout);
  void WaitForDeviceReadyToTransmit();
  int type;

  //Implemented Stream methods:
  virtual int Available();
  virtual bool IsOpen();

  //TODO: implement the missing Stream methods:
  virtual int Read(void *, uint32_t, unsigned long msTimeout = 0);
  virtual int Write(const void *, uint32_t, uint32_t msTimeout = 0);
  virtual void FlushInput();
  virtual void FlushOutput();
  virtual void FlushIO();


private:
  std::string _namespace;
  // El siguiente metodo es para Debug en la misma maquina y ha de llamarse
  // antes que Start().
  // Especifica el prefijo de las colas de mensajes (sin contar el '/'
  void SetQueuePrefix(std::string _qprefix) { qprefix = _qprefix; }
  // Pide a la capa fisica su estado (solo para IPHY_TYPE_PHY)
  void ReqPhyLayerState();
  PhyState _GetPhyLayerState();

  void _SetPhyLayerState(const PhyState &state);

  PacketPtr GetNextPacket();
  void PushNewFrame(PacketPtr);

  void UpdateMQAttr();
  void ShowMQAttr(std::ostream &, int);

  long GetMaxMsgOnQueue(int);
  long GetMaxMsgSize(int);
  long GetNumMsgOnQueue(int);
  bool GetNonblockFlag(int);

  void ClearInputQueue();
  void SetNonblockFlag(bool, int);

  void SendPhyLayerState();
  void SendPhyLayerState(const PhyState &);

  struct mq_attr *GetMQAttr(int);
  mqd_t GetMQId(int);
  void Init(int type, struct mq_attr attr, int perm);

  class ServiceMessage {
  public:
    enum MsgType { FRAME = 0, REQ_STATE, CMD_STATE, NOTBUILT };

    ServiceMessage(PacketBuilderPtr pb);
    ~ServiceMessage();

    void Init(int maxmsgsize);

    MsgType GetMsgType() const { return (MsgType)*type; }
    PhyState GetPhyState() const { return (PhyState)*payload; }
    PacketPtr GetPacket() const;
    void *GetBuffer() const { return buffer; }
    unsigned int GetSize() const { return size; }
    unsigned int GetMaxSize() const { return maxSize; }
    unsigned int GetMaxPayloadSize() const { return maxPayloadSize; }

    void BuildPacketMsg(const PacketPtr &);
    void BuildReqStateMsg();
    void BuildCmdStateMsg(const PhyState &state);

  private:
    void *buffer;
    uint8_t *payload;
    uint8_t *type;
    int size;
    int maxSize, maxPayloadSize;
    PacketBuilderPtr _pktBuilder;
  };

  void Work();
  bool ReceiveMsg(ServiceMessage &);
  void SendMsg(const ServiceMessage &);

  void SavePhyStateFromMsg(const ServiceMessage &);
  void SaveFrameFromMsg(const ServiceMessage &);

  std::queue<PacketPtr> rxfifo;

  std::mutex rxfifo_mutex;
  std::mutex phyState_mutex;
  std::condition_variable rxfifo_cond, phyState_cond;

  std::string txmqname, rxmqname;
  mqd_t txmqid, rxmqid;
  struct mq_attr txattr, rxattr;

  struct mq_attr comattr;
  int comperm;
  std::string qprefix;

  unsigned int maxmsgsize, maxQueueSize, rxQueueSize;

  PhyState phyState;

  // rxmsg y replymsg se tratan en un thread diferente a txmsg.
  // Concretamente, rxmsg y replymsg se tratan en un ServiceThread, y txmsg en
  // el main thread
  ServiceMessage rxmsg, txmsg, replymsg;
  ServiceThread<CommsDeviceService> service;
  bool _started;
  bool _blockingTx;
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_COMMSDEVICESERVICE_H_ */
