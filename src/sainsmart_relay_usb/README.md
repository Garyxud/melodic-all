# sainsmart_relay_usb

ROS interface to SainSmart USB relay boards for automation

* 4 channel: SRD-5VDC-SL-C
    * https://www.sainsmart.com/products/4-channel-5v-usb-relay-module
    * https://www.amazon.com/dp/B009A524Z0/
* 8 channel: SRD-12VDC-SL-C
    * https://www.sainsmart.com/products/8-channel-12v-usb-relay-module
    * https://www.amazon.com/dp/B0093Y89DE/

# List Devices

```bash
rosrun sainsmart_relay_usb list
```

# Relay Node

```bash
rosrun sainsmart_relay_usb relay_node
```

# Parameters
* `serial` Explicitly specify device serial number. Default `<empty>`
* `desc` Explicitly specify device description. Default `<empty>`

# Published topics
* `ready`, [std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html), true when device is connected, otherwise false, (latched topic)
* `serial`, [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html), serial number of the connected device (latched topic)

# Subscribed topics
* `relay_cmd`, [std_msgs/Byte](http://docs.ros.org/api/std_msgs/html/msg/Byte.html), relay command (one bit per relay channel)

# Troubleshooting
* Unable to connect to the device (no terminal messages)
    * Can Linux see the USB device?
        * ```lsusb | grep 0403:6001```
    * Only one executable can use a hardware device at a time.
        * Make sure no other running software is using the hardware.
    * By default, Linux USB devices have very limited permissions. Udev rules are used to modify device permissions when connected.
        * When installing with binaries with apt-get, the udev rules are installed automatically.
        * Check if this package's udev rules are installed: ```ls /etc/udev/rules.d/ | grep 90-SainSmartRelayUsbRules.rules``` or ```ls /lib/udev/rules.d/ | grep sainsmart-relay-usb.rules```
        * Otherwise, follow the directions in [90-SainSmartRelayUsbRules.rules](udev/90-SainSmartRelayUsbRules.rules) to perform a manual install.
* When the board is connected to the PC and the PC is restarted, the relays will be toggled several times. This is because of the structure of the FTDI chip.

