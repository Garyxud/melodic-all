#!/bin/sh
#
# @file port_install.sh
# @brief MacPorts ported OpenRTM-aist installer script
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2010
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#
port_cmd=`which port`
wget_cmd=`which wget`
ports_site=http://www.openrtm.org/pub/MacOSX/macports/
ports_file=ports.tgz
sources_entry=file://`pwd`/ports/
sources_conf=/opt/local/etc/macports/sources.conf

if test "x$port_cmd" = "x"; then
    echo "MacPorts required, but not installed."
    echo "Pleae install MacPorts: http://www.macports.org/"
    exit -1
fi

installed=`port installed |grep OpenRTM-aist`

if test "x$installed" != "x" ; then
    echo "OpenRTM-aist is already installed. exiting."
    exit 0
fi

if test "x$wget_cmd" = "x"; then
    echo "wget command not found. Installing wget."
    echo "# A password may be required for sudo. Input your password."
    sudo port install wget
fi

echo $sources_entry
echo "Getting Portfile from: " $ports_site/$ports_file
wget $ports_site/$ports_file
tar xvzf $ports_file
cd ports
portindex

has_entry=`grep -e "^$sources_entry$" $sources_conf`

if test "x$has_entry" = "x" ; then
    echo "Adding local ports repository entry to sources.conf"
    echo "# A password may be required for sudo. Input your password."
    sudo chmod 666 $sources_conf
    sudo echo $sources_entry >> $sources_conf
    sudo chmod 444 $sources_conf
fi

echo "Installing OpemRTM-aist...it may take several minutes."
echo "# A password may be required for sudo. Input your password."
sudo port install OpenRTM-aist

has_entry=`grep -e "^$sources_entry$" $sources_conf`
if test "x$has_entry" != "x" ; then
    echo "Cleaning local ports repository entry in sources.conf"
    echo "# A password may be required for sudo. Input your password."
    sudo chmod 666 $sources_conf
    sudo grep -v $sources_entry $sources_conf > /tmp/sources.conf
    sudo mv /tmp/sources.conf $sources_conf
    sudo chown root:admin $sources_conf
    sudo chmod 444 $sources_conf
fi

