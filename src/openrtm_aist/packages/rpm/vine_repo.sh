#!/bin/sh
#
# @file vine_repo
# @brief apt-rpm repository database creation for VineLinux
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

# Base directory of repository
basedir="/exports/pub/repository/pub/Linux/Vine/apt"

# VineLinux versions
versions="3.2 4.0 4.2 5.0"

# VineLinux architectures
archs="x86_64 i386 SRPMS"

for version in $versions; do
    for arch in $archs ; do
	rpmdir=$basedir/$version/$arch

	echo ""
	echo "Creating apt-rpm database under:"
	echo $rpmdir
	genbasedir --progress $rpmdir main
    done
done
