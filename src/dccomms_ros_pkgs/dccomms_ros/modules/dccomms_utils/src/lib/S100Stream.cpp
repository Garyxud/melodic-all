/*
 * GironaStream.cpp
 *
 *  Created on: 16 nov. 2016
 *      Author: centelld
 */

#include <cstdint>
#include <dccomms_utils/S100Stream.h>
#include <string>

using namespace std::placeholders;
namespace dccomms_utils {

S100Stream::S100Stream(std::string serialportname,
                       SerialPortStream::BaudRate baudrate, int maxBaudrate)
    : SerialPortStream(serialportname.c_str(), baudrate) {
  _maxBaudrate = maxBaudrate;
  init();
}

S100Stream::~S100Stream() {}

void S100Stream::SetHwFlowControl(bool v) {
  SerialPortStream::SetHwFlowControl(v);
  hwFlowControl = v;
  // https://stackoverflow.com/questions/7582546/using-generic-stdfunction-objects-with-member-functions-in-one-class
  if (hwFlowControl) {
    _WritePacket = std::bind(&S100Stream::_WritePacketHwFlowControl, this, _1);
  } else {
    _WritePacket =
        std::bind(&S100Stream::_WritePacketManualFlowControl, this, _1);
  }
}

/*
Email from Nautilus Oceanica (WFS distributor):
Hola Diego:
Cuando los paquetes son <45bytes, el Seatooth S100 espera un tiempo a menos que
reciba <CR> <LF> para confirmar el final del paquete.
Si usáis control de flujo (handshake de hardware) entre PC y S100 no hay límite
para el tamaño del paquete, pero debe terminar con un <CR> <LF> (retorno de
carro y avance de línea - 0x0D y 0x0A, respectivamente en hexadecimal). Se
recomienda el uso de control de flujo (handshake de hardware)
Si no usáis control de flujo, el tamaño máximo del paquete debe ser 45 y debe
incluir <CR> <LF>
Un saludo
*/

void S100Stream::init() {
  if (_maxBaudrate > 0)
    _byteTransmissionTimeNanos = 1e9 / (_maxBaudrate / 8.);

  _maxTrunkSize = 45;
  SetHwFlowControl(false);
}

void S100Stream::_WritePacketManualFlowControl(const PacketPtr &dlf) {
  auto buffer = dlf->GetBuffer();
  auto fs = dlf->GetPacketSize();

  auto ptr = buffer;
  auto maxPtr = buffer + fs;

  unsigned int _trunkTransmissionTime;

  _trunkTransmissionTime = ceil(_maxTrunkSize * _byteTransmissionTimeNanos);

  while (ptr + _maxTrunkSize < maxPtr) {
    Log->debug("Sending trunk of {} bytes... ({} ms)", _maxTrunkSize,
               _trunkTransmissionTime);
    Write(ptr, _maxTrunkSize);
    std::this_thread::sleep_for(
        std::chrono::nanoseconds(_trunkTransmissionTime));
    ptr += _maxTrunkSize;
  }

  unsigned long left = maxPtr - ptr;
  if (left > 0) {
    _trunkTransmissionTime = ceil((left + 2) * _byteTransmissionTimeNanos);
    Log->debug("Sending trunk of {} bytes and end of packet... ({} ms)", left,
               _trunkTransmissionTime);
    Write(ptr, left);
  } else {
    _trunkTransmissionTime = ceil(2 * _byteTransmissionTimeNanos);
    Log->debug("Sending end of packet... ({} ms)", _trunkTransmissionTime);
  }
  Write(_endOfPacket, 2);
  std::this_thread::sleep_for(std::chrono::nanoseconds(_trunkTransmissionTime));
}

void S100Stream::_WritePacketHwFlowControl(const PacketPtr &dlf) {
  auto buffer = dlf->GetBuffer();
  auto fs = dlf->GetPacketSize();
  Write(buffer, fs);
  Write(_endOfPacket, 2);
}

int S100Stream::_Recv(void *dbuf, int n, bool block) {
  return Read((unsigned char *)dbuf, n, (unsigned int)block);
}

void S100Stream::LogConfig() {

  Log->info("baudrate: {} ; byte transmission time: {} ; frame trunk size: {}",
            _maxBaudrate, _byteTransmissionTimeNanos, _maxTrunkSize);
}

void S100Stream::WritePacket(const PacketPtr &dlf) { _WritePacket(dlf); }

} /* namespace merbots */
