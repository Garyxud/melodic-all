#!/bin/bash

#rosrun collada_urdf_jsk_patch urdf_to_collada `rospack find nao_description`/urdf/nao_robot.xml nao.dae
#rosrun xacro xacro.py  -o nao_robot_v3_structure.urdf `rospack find nao_description`/urdf/nao_robot_v3_structure.urdf.xacro

## nao v3
#rosrun xacro xacro.py -o nao_robot_v3.urdf `rospack find nao_description`/urdf/nao_robot_v3.urdf.xacro
#rosrun collada_urdf urdf_to_collada nao_robot_v3.urdf nao.dae
#rosrun collada_urdf_jsk_patch urdf_to_collada nao_robot_v3.urdf nao.dae

## nao v4
rosrun xacro xacro.py -o nao_robot_v4.rviz.urdf $(rospack find nao_description)/urdf/nao_robot_v4.urdf.xacro
sed -e 's/\.mesh/.mesh.dae/g' nao_robot_v4.rviz.urdf > nao_robot_v4.dae.urdf
rosrun collada_urdf_jsk_patch urdf_to_collada nao_robot_v4.dae.urdf nao.dae

if [ "$?" != 0 ] ;  then exit ; fi

rosrun euscollada collada2eus nao.dae nao.yaml nao.l
if [ "$?" != 0 ] ;  then exit ; fi

rosrun euslisp irteusgl -e "(progn (load \"nao.l\")(nao)(make-irtviewer)(objects (list *nao*)))"
