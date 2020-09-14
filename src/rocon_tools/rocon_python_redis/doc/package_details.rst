Package Details
===============

Upstream
--------

- `Github Sources`_
- `Documentation`_

.. _`Github Sources` : https://github.com/andymccurdy/redis-py
.. _`Documentation` : https://github.com/andymccurdy/redis-py/blob/master/README.rst

Patches
-------

This has been brought into the ros package system since the installable version has a few bugs that make
it unusable for us. Some important points:

* *Version 2.6.2*
* *redis->rocon_python_redis* : to avoid possible conflicts if someone happens to have python-redis installed
* *Patches* : fix a ctrl-c hanging problem.

**About the patch**

The python redis client has a minor issue - it hangs on ctrl-c. This is a locally patched version that
we can use directly until it is patched upstream. More information in the tracked issue:

    https://github.com/robotics-in-concert/rocon_tools/issues/3
