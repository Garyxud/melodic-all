^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh_geo_fog_3d_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2020-01-21)
-----------
* Adding missing diagnostic_msgs dependency to kvh_geo_fog_3d_rviz package.
* Contributors: LaCelle, Zachary

1.3.2 (2020-01-17)
-----------
* No changes to rviz package

1.3.1 (2020-01-14)
-----------
* No changes to rviz package

1.3.0 (2019-12-2)
-----------
* Fixing warnings in the catkin_lint results.
* Updating packaging and cmakes to conform to catkin_lint
* Trying a different way to call find_package for qt5. I have no idea why it's not finding it.
* Removing the EXACT requirement for Qt libraries.
* Adding qt5 dependencies to package.xml in melodic.
* Fixing spelling issue in doxygen comment.
* Adding doxygen comments to the status panel code.
* Adding copyright file headers.
* Moving license files to be under each package, since theoretically they could be licensed separately.
* Adding license information.
* Updating cmakelists to build roslint targets correctly
* Adding devops scripts
* Contributors: LaCelle, Zachary

1.2.0 (2019-09-27)
-----------
* Merge branch 'rviz_plugin' into 'master'
  Rviz plugin
  See merge request DART/kvh_geo_fog_3d!16
* Rviz plugin
* Contributors: Bostic, Trevor R

1.1.0 (2019-08-13 16:32:35 -0400)
---------------------------------
* Adding build/clean scripts for the rviz plugin.
* Adding stubs for the rviz plugin.
