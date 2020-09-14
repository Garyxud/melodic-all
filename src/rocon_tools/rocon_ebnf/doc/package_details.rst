Package Details
===============

Upstream
--------

Details of LParis' original package:

- `Source v0.91 Tarball`_
- `Documentation`_

.. _`Source v0.91 Tarball` : http://lparis45.free.fr/rp-0.91.zip
.. _`Documentation` : http://lparis45.free.fr/rp.html

Patches
-------

- Renamed from ``ebnf`` to ``rocon_ebnf`` to avoid conflict with an original version on the python path.
- Two patches, `fdec1e9a`_ and `ac568daf`_ to allow underscores (oft used in ros names) to be matched
- Another patch, `18a79ca7`_ to correctly mix lowercase and uppercase together for matching.

.. _`fdec1e9a`: https://github.com/robotics-in-concert/rocon_tools/commit/fdec1e9a9fd9bc2a205e3d5ef1b8a084919351c7
.. _`ac568daf`: https://github.com/robotics-in-concert/rocon_tools/commit/ac568dafacddd0947f30de4899f14a8da7f656f5
.. _`18a79ca7`: https://github.com/robotics-in-concert/rocon_tools/commit/18a79ca796ca4eefb9e6b8fee94e772f98ae9267
