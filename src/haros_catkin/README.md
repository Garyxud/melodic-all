# haros_catkin

This package provides a catkin integration for [HAROS](https://github.com/git-afsantos/haros).

Required pip dependencies are installed via [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv).
Info: Currently catkin_virtualenv is only compatible with `catkin build`.

## Example: 

<p><a href="https://ipa-jfh.github.io/haros_catkin_test/#dashboard"> 
  <img align="left" src="https://user-images.githubusercontent.com/1840802/30822997-ab05de10-a1df-11e7-87c5-fc7e0de669ff.png" width="46">
  Result of package analysis
  <br/><br/>
</a></p>

Source code of package: [haros_catkin_test](https://github.com/ipa-jfh/haros_catkin_test)

## Running static code analysis with HAROS:

In the package that should get tested add the following.

Add a test dependency to `package.xml`:

```xml
<test_depend>haros_catkin</test_depend>
```

Add the `haros_report` test to the `CMakeLists.txt`:

```c
if (CATKIN_ENABLE_TESTING)
  find_package(haros_catkin REQUIRED)
  haros_report()
endif()
```


