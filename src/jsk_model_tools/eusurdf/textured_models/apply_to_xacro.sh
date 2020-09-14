#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

for d in `find $DIR  -maxdepth 1 -mindepth 1 -type d`; do
  rosrun eusurdf urdf_to_xacro.py $d/model.urdf $d/model.urdf.xacro
done
