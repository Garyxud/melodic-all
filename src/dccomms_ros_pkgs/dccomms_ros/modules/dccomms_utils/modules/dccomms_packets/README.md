# dccomms_packets
Template library thought for holding custom *dccomms::Packet* implementations. It also compiles the *dccomms* library as a git submodule, so you can use only dccomms_packets library as a submodule for your projects instead of also adding the dccomms library as a extra submodule.

The implementations of *dccomms::IPacketBuilder* can be used by linking directly to this library or using the *class_loader* library: https://github.com/dcentelles/class_loader in your programs. 

### Clonning the repository
Clone this repository with the *--recursive* option:
```bash
$ git clone --recursive https://github.com/dcentelles/dccomms_packets.git
```

### Build
Compile it with cmake:
```bash
$ cd dccomms_packets
$ mkdir build
$ cd build
$ cmake ..
$ make
```
This will generate a *.so* file in the build folder. The name of the library has the following format:
*libdccomms_packets_{git_revision}.so*. Where "*{git_revision}*" is the output of the command "*git rev-parse --short HEAD*"

## How-To: 
### Create a new *dccomms::Packet*
As an example, this library implements the class SimplePacket. The declaration is located in *./include/dccomms_packets* and the implementation in *./src*. 

If you want to create a new one create a new class module just as it was done for SimplePacket. Your class must have a public inheritance from *dccomms::Packet* and implement at least the pure virtual methods of *dccomms::Packet*:
```c++
#ifndef DCCOMMS_PACKETS_{ClassName}_H_
#define DCCOMMS_PACKETS_{ClassName}_H_

#include <dccomms/dccomms.h>
#include <dccomms_packets/types.h>

using namespace dccomms;
namespace dccomms_packets {

class {ClassName} : public Packet {
public:
  {ClassName}();
  ~{ClassName}();
  void CopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(Stream *comms);
  void PayloadUpdated(uint32_t payloadSize);
  bool PacketIsOk();
  void GetPayload(void *copy, int size);
  uint32_t SetPayload(uint8_t *data, uint32_t size);
};
...
```
In addition, you have to implement a *dccomms::IPacket* builder for your Packet:
```c++
...
class {ClassName}Builder : public IPacketBuilder {
public:
  {ClassName}Builder() ;
  PacketPtr CreateFromBuffer(void *buffer);
  PacketPtr Create();
};
}
#endif
```
In order to enable runtime loading of the implemented *dccomms::IPacketBuilder* you have to call the CLASS_LOADER_REGISTER_CLASS macro in the \*.cpp file:
```c++
#include <dccomms_packets/{ClassName}.h>
#include <class_loader/multi_library_class_loader.h>

namespace dccomms_packets {
...
CLASS_LOADER_REGISTER_CLASS({ClassName}Builder, IPacketBuilder)
}
```
This will enable another programs or libraries to found this *dccomms::IPacketBuilder* by using the *class_loader* library: https://github.com/dcentelles/class_loader.
Now you have to edit the *CMakeLists.txt* file adding the new .cpp module:
```cmake
SET(${PROJECT_NAME}_CPP_FILES
                        ${SRCPATH}SimplePacket.cpp
                        ${SRCPATH}ExternalPacketsExporter.cpp
                        ...
                        ${SRCPATH}YourModule.cpp
                        )

```
After editing the *CMakeLists.txt* file you have to regenerate the makefiles using *cmake* and then recompile the library using make as it was shown in the [build](#build) section.
### Change the generated library name
Edit the CMakeLists.txt file changing the following line:
```cmake
SET(PROJECT_NAME_ORIG new_lib_name )
```
The new library file will be:
*libnew_lib_name_{git_revision}.so*
