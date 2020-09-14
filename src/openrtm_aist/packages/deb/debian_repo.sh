#!/bin/sh
#
# @file debian_repo
# @brief apt-deb repository database creation for debian
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
cd /exports/pub/repository/pub/Linux/debian

# debian versions
versions="etch sarge lenny"

# debian architectures
archs="binary-i386 binary-amd64"

for version in $versions; do
    for arch in $archs ; do
	debs="dists/$version/main/$arch"
	echo ""
	echo "Creating apt-dev database under:"
	echo $debs
	apt-ftparchive packages $debs | gzip -c9 > $debs/Packages.gz
    done
#   dpkg-scansources $version/main/source | gzip > $version/main/source/Sources.gz
done
