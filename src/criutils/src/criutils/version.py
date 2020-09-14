#! /usr/bin/env python
import resource_retriever
import catkin_pkg.package

manifest = resource_retriever.get_filename('package://criutils/package.xml',
                                                            use_protocol=False)
catkin_package = catkin_pkg.package.parse_package(manifest)

__version__ = catkin_package.version
