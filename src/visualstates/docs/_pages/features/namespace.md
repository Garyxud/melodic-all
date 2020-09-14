---
title: Namespace Functionality
permalink: /features/namespace

sidebar:
  nav: "docs"
---

## What are Namespaces in VisualStates?  
VisualStates provides `Global` and `Local` Namespaces. 
* Global Namespace consists all the functions, variables and the ROS nodes with publishers and subscribers which are required globally through the complex automata.
* Local Namespace consists functions and variables required by a particular state or transition or a group of states or transitions. Every level of automata consists of a Local Namespace where its local variables and functions can be defined.

## Why Namespaces?
* While developing complex behaviors, the states and transitions can have their own auxiliary data accessible by them only.
* Seperate Namespaces avoids the auxiliary data name conflicts. While developing automata, user may provide functions and variabes names which may conflict with other automata's auxiliary data.
* While importing, existing auxiliary data might conflict with the importing automata i.e. the method and variables might collide, seperating them into Local Namespaces avoid the issue upto a great extent.

## How to use Namespaces in VisualStates while developing a complex behavior?
1. `Data -> Global Namespace`: 
* Write all the global methods and variables which are required to be used throughout the behavior in all levels of automata. 
* The Global Namespace would also contains all the ROS Publishers and Subscribers.
* Members can be accessed using `self.globalNamespace.[method or variable name]` anywhere in the whole automata.

2. `Data -> Local Namespace`:
* On the exisiting level of automata, all the states and transitions can access these methods and variables.
* Write all the local methods and variables which are required to be used on the current level of automata in states and transitions.
* On the same level, the local members can be accessed using `self.localNamespace.[method or variable]`
* to navigate through different local namespaces, navigate through different levels of automata using the TreeView provided in the VisualStates.
