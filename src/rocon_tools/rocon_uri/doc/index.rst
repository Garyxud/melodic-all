.. rocon_uri documentation master file, created by
   sphinx-quickstart on Sun Mar 23 23:19:28 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Rocon Uri
=========

Various entities in the `Robotics in Concert`_ framework require a specification for
describing the kind of resource it is, or the kind of resource it is compatible with.
In particular, such a specification must describe entities like robots, define
compatibility strings for apps (i.e. what robots/platforms can run this app) and
shape robot requests (give me a robot with this specification and able to run this robot app).
It is the glue which will ultimately enable us to run a diverse mix of robots and
applications together to provide useful robotic solutions.

We defer to the standard specifications for `universal resource identifiers`_ to
format strings representing our resources and `ebnf`_ as a notation to express
the exact format required for these strings. This is all done in the `rocon_uri`_ package 
which provides a specification, some ebnf rules and an api for constructing and manipulating
resource identifiers used in the rocon framework.

.. _`universal resource identifiers`: http://en.wikipedia.org/wiki/Uniform_resource_identifier
.. _`ebnf`: http://en.wikipedia.org/wiki/Extended_Backus%E2%80%93Naur_Form
.. _`rocon_uri`: http://wiki.ros.org/rocon_uri
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page

Contents:

.. toctree::
   :maxdepth: 2

   specification
   rules
   usage
   modules
   changelog

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

