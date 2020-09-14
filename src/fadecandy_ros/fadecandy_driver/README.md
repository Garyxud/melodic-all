# Fadecandy driver

ROS node for controlling LEDs via a fadecandy board. Uses python-usb to talk directly to the board,
so it doesn't need the fadecandy server - there are no dependencies on anything in the fadecandy
repo. All you should need is a fadecandy board with fadecandy firmware on it.

#### How to run

Start the node. Depending on permissions, this may need to run as root to talk to the USB device.

```
rosrun fadecandy_driver fadecandy_node
```

To make sure everything is working, you can send a test pattern using the example client script.

```
rosrun fadecandy_driver example_client
```

### Note about how the LEDs are addressed

The fadecandy board has 8 pairs of outputs, numbered 0-7 on the PCB. Each of these can control a
single Neopixel LED strip of up to 64 LEDs. In the LEDArray mesage, you can specify color outputs
for an arbitrary number of strips. If the "strips" array in the LEDArray has length N, those
colors are applied to the first N strips, starting with strip 0 on the board. So if you wanted to
provide colors for just strip 0 and strip 5, you would need to include three empty LEDStrip
messages in the "strips" message between them.

Within each LEDStrip message you provide a list of color values for each LED. If colors for less
than 64 LEDs are provided, the rest are set to (0,0,0) (completely off). Also, the "a" (alpha)
values of the colors aren't used.

Finally, the fadecandy board doesn't have any way of knowing how many strips are attached or
how long they each are. It just pushes out 64 values for each of the 8 strips.

### Configuring linux device permissions

By default, access the Fadecandy USB device in linux using libusb requires running as root. To
fix this, copy the rules file from the udev directory of this package into `/etc/udev/rules.d`.

```
sudo cp udev/10-fadecandy.rules /etc/udev/rules.d/
```

Now reload the udev rules.

```
sudo udevadm control --reload-rules && udevadm trigger
```

If you have the Fadecandy device plugged in, unplug and replug it. Then make sure that your user has
been added to the dialout group. Now you should be able to run without needing root!
