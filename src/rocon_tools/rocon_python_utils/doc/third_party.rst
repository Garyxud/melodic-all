
.. _third_party-section-label:

Third Party Modules
===================

Configure
---------

A configuration toolkit for yaml on pypi.

* `Home Page`_ 

.. _`Home Page`: http://configure.readthedocs.org/en/latest/

Configure is a tiny module that provides some really nice features on top of the regular
yaml library. For us specifically, the two that grab the headlines include:

* *include* : via either inheritance or composition
* *merge* : merge another yaml on top of your current one.

**Includes**

Inheritance (extending). This is a useful way of providing overrides. Note that
filenames can be relative or absolute.

.. code-block:: yaml

   # defaults.conf
   dude: joe
   
   # overrides.conf
   !extends:defaults.conf
   dude: tarzan
   dudette: jane

Composition, which puts another yaml in a namespace in the current yaml.

.. code-block:: yaml

   # lads.conf
   dude: tarzan
   
   # lasses.conf
   lads: !includes:lads.conf
   lasses:
     dudette: jane

**Merges**

.. code-block:: python

   configuration = Configuration.from_file('./configuration.yaml').configure()
   customisation = Configuration.from_file('./customisation.yaml').configure()
   new_configuration = configuration.merge(customisation)
