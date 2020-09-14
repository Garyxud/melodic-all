^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_assimp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2018-12-05)
------------------

0.4.2 (2018-09-04)
------------------

0.4.1 (2018-08-01)
------------------

0.4.0 (2018-07-25)
------------------
* since euslisp 9.25.0, we need to add 5 arguments including doc string with defun() (`#217 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/217>`_)
* [eus_assimp] add use-coordinate option to append-vertices in order to express all the vertices respect to world frame (`#210 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/210>`_)
* [eus_assimp] need to set faces before calling gl::make-glvertices-fro (`#208 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/208>`_)
* Fix for arm processor (`#209 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/209>`_)
  * fix for arm64/v8 aarch64 is 64bit

* [eus_assimp] mesh2wrl.sh: add utility scripts  (`#206 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/206>`_)

* Contributors: Eisoku Kuroiwa, Kei Okada, Yohei Kakiuchi

0.3.5 (2017-02-23)
------------------

0.3.4 (2017-02-22)
------------------

0.3.3 (2017-02-18)
------------------

0.3.2 (2017-02-18)
------------------

0.3.1 (2017-02-18)
------------------

0.3.0 (2017-01-16)
------------------

0.2.5 (2016-10-18)
------------------

0.2.4 (2016-04-30)
------------------

0.2.3 (2015-12-31)
------------------
* [eus_assimp] fix function name
* Contributors: Yohei Kakiuchi

0.2.2 (2015-12-11)
------------------

0.2.1 (2015-11-27)
------------------

0.2.0 (2015-11-26)
------------------
* move to 0.2.0, which only available from indigo

* [eus_assimp] Add .gitignore
* Contributors: Kentaro Wada

0.1.13 (2015-09-01)
-------------------
* move functions to jskeus
* add function for glvertices
* fix minor bugs
* move functions to jskeus
* add dump-glvertices-to-wrl for creating wrl mesh file from glvertices
* 0.1.12
* update CHANGELOG (For releasing 0.1.12 DRC Final version)
* [jsk_model_tools] remove old rosmake files
* Contributors: Kei Okada, Yohei Kakiuchi, Iori Yanokura

0.1.12 (2015-05-07)
-------------------
* [jsk_model_tools] remove old rosmake files
* Contributors: Kei Okada

0.1.11 (2015-04-09)
-------------------

0.1.10 (2015-04-02)
-------------------

0.1.9 (2015-04-01)
------------------

0.1.8 (2015-01-07)
------------------
* fix for new euslisp layout
* eus_assimp: build_depend on pkg-config
* Contributors: Scott K Logan, Kei Okada

0.1.7 (2014-12-19)
------------------
* fix parsing file extention, in order to use .stlb extention for exporting mesh as STL Binary format
* use tiff file for texture
* Contributors: Yohei Kakiuchi

0.1.6 (2014-06-30)
------------------

0.1.5 (2014-06-29)
------------------

0.1.4 (2014-06-15)
------------------
* update convex-decomposition parameters
* add include directory for SDL
* using SDL in hydro if exists
* Contributors: Yohei Kakiuchi

0.1.3 (2014-05-01)
------------------

0.1.2 (2014-05-01)
------------------
* Merge pull request `#34 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/34>`_ from k-okada/fix_to_pass_buildfirm
  Fix to pass buildfirm
* Contributors: Kei Okada
* add depends to euslisp
* added euslisp_SOURCE_PREFIX for finding eusisp package path in catkin.cmake
* Contributors: Masaki Murooka

0.1.1 (2014-05-01)
------------------
* (eus_assimp) update files for using assimp_devel in jsk_common (`#20 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/20>`_)
* fix eus_assimp
* change store-glvertices to save-mesh-file
* add code for dumping textures
* fix default direction
* change: arguments pass to store-glvevrtices
* add check for recalc normal
* change post process methods
* add dump-to-meshfile to eus_assimp
* using assimp-read-image-file when c-assimp-load-image being defined
* change for preventing defun of unexisting function
* update for ignoring up_direction, refere to https://github.com/assimp/assimp/pull/60
* add function update-to-original-mesh for using original meshfile as visual
* fix make-cube-from-bounding-box
* add make-cube-from-bounding-box
* add make-glvertices-from-faces
* implement scale option of store-glvertices
* add assimp-read-image-file
* add :direction keyword to load-mesh-file
* add code for treating texture
* add parameter for convex_decomposition
* fix compiling with convex decomposition
* update convex decomposition code for eus_assimp
* temporary add CMakeLists.convexdecmop.txt
* add eus_assimp for eusing assimp library on EusLisp
* Contributors: Yohei Kakiuchi
