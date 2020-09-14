Specification
=============

Definition
----------

A not so rigorous description of a rocon uri string:::

    rocon://concert_name/hardware_platform/name/application_framework/operating_system#rocon_app

It can be expressed more rigorously via an `ebnf`_ notation, however this gets complicated given the
various contexts in which rocon uri strings get used. A description of the various fields: 

* **concert_name** : the concert name for which a resource is attached to.
* **hardware_platform** : the hardware associated with this resource (e.g. pc, turtlebot)
* **name** : name of this resource, typically a unique robot name within a concert (e.g. Bob3)
* **application_framework** : the software framework used, generally this is ros, but can be others (e.g. ros, opros).
* **operating_system** : operating system/version that is used (e.g. precise)
* **rocon_app** : the ros resource name of a rocon app (e.g. rocon_apps/teleop)


.. _`ebnf`: http://en.wikipedia.org/wiki/Extended_Backus%E2%80%93Naur_Form

Special Operations
------------------

Typically a full rocon uri string is not required and it can also be manipulated via specially
assigned operators and wildcards. These include:

* **Empty Concert Name**

This field is typically always empty and represents the current concert (multimaster solution). It has
been reserved for future use if we ever wish to enable multi-concert solutions.

* **Empty Field/Wildcard '*'**

This represents the 'any' definition in which any valid value for that field is compatible. An example
for this usage is to define the compatibility string for a rocon app which is able to run on any *hardware_platform*.
This might be the case for something like a generic ros talker app which does not have specific hardware
requirements.

* **Discarded Fields**

Fields may be discarded from right to left depending on context. This allows brevity in specifying some
rocon uri strings for which the full context is not important. Discarded fields are assumed to take on
the same value as for empty or wildcarded fields.

* **Or Operator '|'**

The *harware_platform*, *application_framework* and *operating_system* fields all support the
use of the or operator in which several values can be specified together. This can be used for instance,
to define several platforms that a rocon app is permitted to run on. The Or operator is not valid for the *name* field.

* **Regex Patterns**

Regex patterns can be used on the *name* field. The name field for rocon resources (i.e. robots) is often
uniquely specified by postfixing a uuid/counter on the end. Requests for these resources can customise requests
for a particular name by using a regex patterns. 

.. _specifications-examples-section-label:

Examples
--------

A *retaskable client*, i.e., a robot that is available for use within a rocon concert:::

    rocon://turtlebot2/cybernetic_pirate
    rocon:/pr2/bob

An *allocated resource*, i.e., a robot that has been allocated and is performing a scheduled job within a rocon concert:::

    rocon:/turtlebot2/cybernetic_pirate#rocon_apps/im_here_to_make_you_lazy
    rocon:/pr2/bob#rocon_apps/look_menacing

For *resource requests* made by concert services requesting a robot to perform a task within a rocon concert:::
 
    rocon:/*/*#rocon_apps/im_here_to_make_you_lazy
    rocon:/pr2/*#rocon_apps/look_menacing

A *remocon* identifier, i.e. a software remote control connecting to a rocon concert via a human:::

    rocon:/note3/remocon_342ac3e813/hydro/jellybean
    rocon:/pc/bobs_remocon/hydro/precise

For *compatibility* string specifications of robot apps that define the permitted platforms on which it can run:::

    rocon:/turtlebot2
    rocon:/turtlebot2|pr2
    rocon:/

For *compatibility* string specifications of human interactions (apps running on a remocon) that define the permitted platforms on which it can run:::

    rocon:/note3/*/hydro/jellybean
    rocon:/pc/bobs_remocon/hydro/precise
