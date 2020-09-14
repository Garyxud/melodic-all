# dccomms_utils
C++ project for implementing [*dccomms*](https://github.com/dcentelles/dccomms_examples) utilities and extra drivers to communicate with real modems. For now, there are implemented utilities to communicate with commercial underwater modems, which is one of the main interest of the [IRSLab](http://www.irs.uji.es/) research team.
### Underwater Modems
#### S2CR 18/34
Designed and created by [Evologics](https://www.evologics.de/). Available in the CIRS facilities (Girona), where the [ViCOROB](http://vicorob.udg.edu/) team carries out its daily activity. 
- USBL and Communication: Acoustic modem for communication and positioning. The USBL version is able to communicate and compute the position of the other modems. The USBL version is installed in a buoy or a boat, and the pure modem is installed in a robot (normally *Girona500* or *Sparus* for the experiments performed by ViCOROB).
- Half-Duplex.
- Up to 13.9 kbps.
- Up to 3500 meters.
- High and Variable delay.
- Implements the D-MAC protocol (by Evologics): [D-MAC](http://ieeexplore.ieee.org/document/6003586/)

Two implementations of the *dccomms::Stream* interface have been developed to enable the ROV teleoperation with compressed visual feedback:
- *USBLStream*: All data is transmitted attaching up to 64 bytes into the D-MAC the acknowledgements (PiggyBack Messages). These bytes will encode the operator orders.
- *GironaStream*: All data is transmitted using the burst mode. These data will encode the current state of the robot and the camera image.

Using this configuration It has been achieved an asynchronous communication link, where the data rate has been up to 1.6 kbps from the ROV to the Operator and 0.2 bps from the Operator to the ROV in conditions of high level of multi-path.

#### S100
RF modem available in the IRSLab facilities. Manufactured by [WFS](http://www.wfs-tech.com/) (Wireless For Subsea).
- Half-Duplex.
- Up to 1.9 kbps
- Up to 5 meters.
- Low and Constant delay.
- Implements a basic MAC protocol similar to the native ALOHA. The ack feature can be enabled or disabled.
- Allows to change the encoding: Fast Manchester (It consumes more bandwidth, but protects against errors) or No encoding.

The implemented class derived from *dccomms::Stream* is *S100Stream*.


