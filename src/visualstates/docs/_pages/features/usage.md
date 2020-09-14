---
title: Usage Guide
permalink: /features/usage

sidebar:
  nav: "docs"
---

The VisualStates GUI has four sections: an upper menu bar, a treeView, SchemaView and dialogs for code and configs.

<p align="center">
  <img width="1200" src="/assets/images/visualstates-gui.png">
</p>

The menu bars contains the main buttons, which will allow you to do most of the actions. These buttons are:

## File button
* New: it allows you to start a new empty automata. It will close your actual automata if you had anyone open, and create a new empty subautomata for starting your new automata.
* Open: it allows you to open an existing automata and continue editing it.
* Save: it save the changes in the current path of the automata you are working with. If you did not saved yet, It will show up save as dialog.
* Save as: it allows you to save the automata as an xml file in the path you choose. This path will be the new path of your project, in case you press the button save.
* Quit: it exits the tool. If you have not save your change they will be lost.

## Figures button
* State: it allows you to create new states. For this, you must click the button and then click in the Schema wherever you want to place the new state. Ater that you can edit it. Nodes are represented as blue circles.
* Transition: it allows you to create new transitions. It works similar to the state button. You click it, and then, in the Schema view, you click the first node (state) origin and then the target one. If the target node is the same that the node origin, the tool will draw an autotransition. Transitions are represented as arrows pointing to the target node.

## Data button
* Timer: opens a window allowing you to modify the time of the loop. This means, you can vary the frequency of condition check.
* Variables: it pop up a window. You can define any variable you need in the nodes or transitions.
* Functions: you can also define your own functions, in case you need them. Functions have a global scope, both in python and in C++, while the variables you may declare has a local scope. They will only be seen in the subautomata where you are declaring them. In python, you can made them have a global scope by declaring them with "self.", as then they will be a property of the python object automata.

## Actions button
* Libraries: it allows you to add any library you may need. You only have to put its name.
* Config file: it allow you to select the interfaces you may need in the code of states and transitions. You can first choose the type of the communication interface: JdeRobot Communication or ROS Nodpice. If you choose JdeRobot Communication you define your interfaces from given interface where you can set Server Type, Name, Topic, Proxy Name, IP, Port, Interface. If you choose ROS Node, you define package build dependencies and run dependencies under Package tab. You can choose your ROS topic interfaces from Topics tab. You can set the name of the topic, type of the topic and whether you would like to publish or subscribe to the topic.
* Generate C++ code: when clicking in this button, the tool will generate the C++ code of the automata saved CMakeLists.txt and the yml file with the interfaces that you have already defined if it is JdeRobot Communication interface. Otherwise, it will generate package.xml, CMakeLists.txt, python runtime codes and C++ code of the automata to the given catkin workspace.
* Generate python code: generate the python code of the automata and the yml file. It works in the same way as when clicking in Generate C++ code. It will leave the automata code in executable file .py.

Now, looking at the Tree view, you can see that it displays two column: the node name and the node ID.I n the image, it is only displaying the nodes of the root subautomata, but the arrow next to its ID shows that these nodes also have sons. If you click on the arrow, the Tree view will expands and it will show this children also, so you can choose until which level do you want to see. Additionaly, you can use the Tree view for navigating through different subautomatas. If you do double click in a subautomata that you are not seeing now in the Schema view, it will redraw the Schema and show it.

Finally, there is the Schema view. It allows you to add and edit states and transitions to your current subautomata for programming its behaviour. For adding new nodes, you just click in the menu on Figures/State, and then click on the Schema, and it will draw you a new node. If you make right click on it, you will have some options for editing it:

* Rename: allows you to change its current name. By default the name given is "state". All states must have a different name. You can also rename your states if you double click on the name of the state.
* Code: it will pop up a new window where you can add the code that will be executed when this state is active.
* Mark initial: will set this node as the initial node of the subautomata. The initial nodes is the first node executed when the subautomata start for the first time. They are represented with a concentric circle.
* Copy: allows you to copy the node and then paste it in other place by doing right click/paste.
* Remove: it will erase this node and its transitions.

Also, after you have created a transition, if you right click on the red handle of the transition it will also show some options for editing it:

* Rename: set a new name. Two transitions can have the same name.
* Code: Change the transition type. There are two types of transitions: temporal and conditional. If you set it as a temporal transition, then you must set a time (in milliseconds). After that time has passed, the node will stop of being active and the node pointed by this transition will be the active one. If you set the transition as conditional, then you must declare a boolean expression and the transition will take place when the expression is evaluated to true. If you choose conditional, you are going to write your conditional expression in the code editor at the bottom part of the dialog.
* Remove: it erase the transition.
* Additionally, if you want to create a subautomata son, then you must do double click on the node you want to be the father. Then the Schema will draw a new blank subautomata for adding the nodes and transitions you need, and they will appear in the Tree. For going back to the subautomata father you only have to click on the Up button, under the Tree. This button will redraw in the Scheme the current subautomata's father or will not do anything if your subautomata does not have any parent. You can also use the Tree view for going to another subautomata. If you want to erase a subautomata, you must erase all its nodes and transitions, and then go to another subautomata (its father, for example).

## Execution of the Python created automaton

When you press on generate python code, an executable python script and the .yml file will be created. Again, if you execute the script with the argument --displaygui=true, a runtime GUI will be shown. Its aspect is like the GUI of visualStates, but it does not have a menu bar, it only have the Schema and the Tree view just for visualization purposes.

In this case, the GUI will not allow to edit any parameter, it just shows the active nodes allowing to visualize how the automata is working and if it goes from one state to another as we were expecting. In the Tree, the active node will be shown in green and the others in white, so you can see a resume of which nodes are actives in which subautomatas, all at the same time. In the Schema, there is only one subautomata drawn. Again, the active node will be coloured in green and the rest in blue. This section allow to see which node is active in the actual subautomata and if it transit to the state that it was supposed.

The navigation is similar to the navigation in the visualStates GUI. You can go to one node child by double clicking on it and return to the current subatomata's father by clicking on the button Up, or you can just make double click on the Tree over the subautomata you want to see.
