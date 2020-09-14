---
title: Import Functionality
permalink: /features/import

sidebar:
  nav: "docs"
---

## What is Import Functionality in VisualStates?

A complex behavior of a robot consists of several states and tranisitions. Complex automata consists of parent child hierarchies with parent state representing a particular behavior developed using several child states. Say, for an example, the behavior `ObstacleAvoidance` is an automata and a parent state with `Detect` and `Avoid` as child states. The behavior takes the Laser data of robot as the input, processes it and decides tranisitions based on coniditions given by user and outputs the robot motion controls. 
Consider a case scenario in which a VisualStates user is developing a complex behavior which requires the obstacleAvoidance behavior as a child state. VisualStates provides the Import Functionality to import the exisiting behavior in the developing behavior. Rather then devloping the `obstacleAvoidance` behavior from scratch, user could import the prebuilt automata `obstacleAvoidance` into his StateSpace and alter the auxiliary functions and variables, conditions and sensor data according to the new robot. 

How to use VisualStates Import Functionality?
1. Using the TreeView in VisualStates, navigate to the site where you want to import prebuilt behavior.
2. Click on `File -> Import`. Navigate FileManager to the VisualStates XML file of prebuilt behavior.
3. Double Click the XML file and you would see the states and transitions are imported to your StateSpace.
4. Edit the Robot configurations according to your robot using `Actions -> Config File`.

Following this, you have successfully imported the exisiting automata to your robot's developing automata.
