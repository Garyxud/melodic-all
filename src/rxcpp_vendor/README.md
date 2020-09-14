# rxcpp_vendor


## Overview

This packages [RxCpp](https://github.com/ReactiveX/RxCpp) (version `4.1.0`) as a ROS 1 Catkin package, allowing it to be built as part of a Catkin workspace.

No changes have been made to the sources, they have only been packaged.


## Usage

Dependents should add a `build_depend` (and if needed, a `build_export_depend`) on the `rxcpp_vendor` package.

Headers are installed into `<prefix>/include/rxcpp` (*not* `rxcpp_vendor`), so the normal include paths can be used in source files.


## Roadmap

No future changes to this package are expected, other than perhaps tracking upstream if/when new RxCpp releases are made.

A Bloom 3rd party release may be considered if that is considered less maintenance or deemed more compatible.
