# Leuze Phidget Interface Kit (IK) driver

This package maps the I/Os of the Leuze RSL sensor to ROS messages. For this purpose, we use a [Phidget IK Board](https://www.phidgets.com/?tier=3&catid=2&pcid=1&prodid=1019). The outputs of the RSL scanner should be connected to the inputs of the IK board and the inputs of the RSL scanner should be connected to the outputs of the IK board.

## Requirements

The ROS driver for the IK board is available [here](https://github.com/ros-drivers/phidgets_drivers) and needs to be installed first, either from source or as a Debian package :

```
sudo apt install ros-<distro>-phidgets-ik
```
where `<distro>` is your distribution of ROS *(Kinetic/Melodic)*.

## Bringup
```
roslaunch leuze_bringup leuze_phidget_driver.launch
```
This starts up the *Phidget IK* driver that you installed, as well as the *Leuze Phidget Driver* from this package. The *Phidget IK* driver spawns a large number of topics based on the specific RSL model connected. The *Leuze Phidget Driver* from this package combines them conveniently into one single *publishing* topic for the inputs of the IK (i.e. outputs of the RSL) as `ik_show_inputs`, and a single *subscribing* topic for the outputs of the IK (i.e. inputs of the RSL) as `ik_set_outputs`. Note that the terms *inputs/outputs* in the topic names refer to the Phidget IK board.

## Usage

This driver is utilized as follows:

```
RSL Output --> IK Input --> Publish on /ik_show_inputs (leuze_msgs/PhidgetIKInputMSG) --> User can read   
User can write --> Subscribe from /ik_set_outputs (leuze_msgs/PhidgetIKOutputMSG)  --> IK Output --> RSL Inputs
```

#### !!ALERT!!
The user must note that the default *Phidget IK* driver publishes new information based on **state changes**. That is, only when there is a transition in the state of an output, does it publish a message on it's respective topic. Therefore, once the *Leuze Phidget Driver* is launched via roslaunch, the user is advised to briefly place an obstacle (a hand or a foot for example) directly in front of the RSL scanner in order to force trigger all I/Os. Until this is done, the Phidget IK may show junk values for the I/Os, but thereafter faithfully follows the correct values.

## I/O Mapping
The mapping of the I/Os of the scanner to the pins of the Phidget board can be seen below. Note that in the first table, the first column refers to the outputs of the scanner, and the second column refers to the inputs of the Phidget board. Vice versa for the second table. Depending on your model of the RSL400 laser scanner family, not every I/O seen below will be used. Please refer to the Leuze manuals to know which I/Os are available for you model. Since the I/Os of the scanner are recognized by the colors of the cables, the user is advised to be cautious when setting up the connections for different models. Refer the last section for the manuals, including cable color coding.

### Outputs of the scanner:  
| Output Name (RSL)|   Input slot (Phidget Board)|
|:-:	|:-:	|
|EA1   	|   	0|
|OSSDA1   	|   1|
|OSSDA2   	|   2|
|MELD   	|   3|
|A1         |   4|
|A2         |   5|
|A3         |   6|
|A4         |   7|
|EA2        |   8|
|OSSDB1     |   9|
|OSSDB2     |   10|


### Inputs of the scanner:
| Input Name (RSL)|   Output slot (Phidget Board)|
|:-:	|:-:	|
|RES1   	|   	0|
|F1   	|   1|
|F2   	|   2|
|F3   	|   3|
|F4         |   4|
|F5         |   5|
|SE1         |   6|
|SE2         |   7|
|F6        |   8|
|F7     |   9|
|F8     |   10|
|F9     |   11|
|F10     |   12|
|RES2     |   13|

## List of supported devices and links to their manuals:
* [RSL410](https://leuze.de/selector/ci_pages/downloads.php?supplier_aid=53800207&key=b86824a92196d89e17ffb4ab1db08090955c9440edd676fb9d082b51c0a60382)
* [RSL420](https://leuze.de/selector/ci_pages/downloads.php?supplier_aid=53800250&key=8f19397cb0883af7fa2ce68e5ba5a819f1491f1a4b69edbc04b0ceb899d695d0)
* [RSL430](https://leuze.de/selector/ci_pages/downloads.php?supplier_aid=53800223&key=fd8c443d18287906aa82dd0ed4837eb4fe8a2735c6d13d2b718fc7b07f9f00d8)
* [RSL440](https://leuze.de/selector/ci_pages/downloads.php?supplier_aid=53800235&key=bea1bb088ecd53ebd09ddbaa8b21ca1496d5cadade3f06807104ab8cd078d4b0)
* [RSL425](https://leuze.de/selector/ci_pages/downloads.php?supplier_aid=53800250&key=8f19397cb0883af7fa2ce68e5ba5a819f1491f1a4b69edbc04b0ceb899d695d0)
* [RSL445](https://leuze.de/selector/ci_pages/downloads.php?supplier_aid=53800235&key=bea1bb088ecd53ebd09ddbaa8b21ca1496d5cadade3f06807104ab8cd078d4b0)
