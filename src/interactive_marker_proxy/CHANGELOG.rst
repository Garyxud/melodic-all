^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interactive_marker_proxy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2014-06-09)
------------------
* cleanup
* Contributors: Russell Toris

0.1.1 (2013-02-13)
------------------
* added install target
* Contributors: David Gossow

0.1.0 (2013-01-02)
------------------
* fixed dependencies etc in package.xml/CMakelists
* better target name to avoid cmake conflict
* catkinized package
* Merge pull request #1 from KaijenHsiao/master
  added stuff to manifest to make rosmake happy
* added stuff to manifest
* removed tf transformation of frame-fixed int.markers from im proxy
* Merge branch 'master' of github.com:dgossow/interactive_marker_proxy
* added change detection for frame-fixed markers: only generate pose when translation/rotation changes
* im proxy: store pose in case of update message and for frame-locked pose update
* Update README.md
* print status only once
* added Makefile and manifest
* initial commit
* Initial commit
* Contributors: David Gossow, Kaijen Hsiao
