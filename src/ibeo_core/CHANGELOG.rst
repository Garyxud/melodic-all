^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ibeo_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2018-12-07)
------------------
* Merge pull request `#8 <https://github.com/astuff/ibeo_core/issues/8>`_ from ShepelIlya/master
* Deleted redunant conditions & fixed offset in ObjectData2280. It works!
* Contributors: Rinda Gunjala, Шепель Илья Олегович

2.0.1 (2018-11-19)
------------------
* Merge pull request `#7 <https://github.com/astuff/ibeo_core/issues/7>`_ from astuff/maint/add_urls
  Adding URL to package.xml and updating README.
* Merge pull request `#6 <https://github.com/astuff/ibeo_core/issues/6>`_ from astuff/fix/bad_alloc
  Fix bad_alloc SEGSIV error.
* Merge pull request `#4 <https://github.com/astuff/ibeo_core/issues/4>`_ from ShepelIlya/patch-1
  Fix for reading of object_box_orientation_angle
  Byte order for object bounding box orientation angle changed from little-endian to big-endian. According to the document "Interface Specification for ibeo LUX, ibeo LUX systems and ibeo Evaluation Suite", version 1.48 from 30.05.2017 there is big-endian byte order for all fields in Object2280. If i am using original code orientation of objects orientation changes abruptly. With that fix it seems to work correct.
* Contributors: Joshua Whitley, Rinda Gunjala, Sam Rustan, ShepelIlya, Zach Oakes

2.0.0 (2018-07-05)
------------------
* Adding Melodic build. Fixing allowed_failures.
* Adding roslint and fixing reported issues.
* Initial commit for Github.
* Contributors: Joshua Whitley, Sam Rustan
