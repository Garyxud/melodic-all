# -*- m4 -*-
#
# @file   openrtm.m4
# @brief  openrtm m4 macro definitions
# @date   $Date: 2006-11-04 17:03:07 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2006
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: openrtm.m4 775 2008-07-28 16:14:45Z n-ando $
#

dnl ----------------------------------------------------------------------
dnl proc to look for a file in a number of places
dnl
dnl FP_FIND_FILE[file_to_find, path_prefix, path_subdir, (fp_path_var)]
dnl
dnl This macro searches a file in given paths which are the
dnl combination of each path_prefix/path_subdir.
dnl The Found path of the file and the directory name are stored
dnl to the variable named "fp_path_var" and "fp_path_var_dir"
dnl If "fp_path_var" (4th var) is abbreviated, underscored
dnl "file_to_find" variable name is used.
dnl
dnl Example:
dnl FP_FIND_FILE[omniORB4/CORBA.h, /usr /usr/local /opt, include cpp/include]
dnl
dnl search path: /usr/include, /usr/cpp/include
dnl              /usr/local/include, /usr/local/cpp/include
dnl              /opt/include, /opt/cpp/include
dnl file path var: $omniORB4_CORBA_h
dnl dir  path var: $omniORB4_CORBA_h_dir
dnl ----------------------------------------------------------------------
AC_DEFUN([FP_FIND_FILE], [
    if test "x$4" = "x" ; then
	ff_name=[`echo $1 | sed 's/[-.*/ ]/_/g'`]
    else
	ff_name="$4"
    fi
    eval $ff_name=
    eval ${ff_name}_dir=
    ff_file=
    ff_file_dir=
    for ff_dir in $2 ; do
	if test -f $ff_dir/$1 ; then
	    ff_file_dir=$ff_dir
	    ff_file=$ff_dir/$1
	    break
	fi
	for ff_subdir in $3 ; do
	    if test -f $ff_dir/$ff_subdir/$1 ; then
		ff_file_dir=$ff_dir/$ff_subdir
		ff_file=$ff_dir/$ff_subdir/$1
		break
	    fi
	done
	if test "x$ff_file" != "x" ; then
	    break
	fi
    done
    eval ${ff_name}_dir="$ff_file_dir"
    eval $ff_name="$ff_file"
])


dnl----------------------------------------------------------------------
dnl
dnl @func UNIQUE([list])
dnl @brief delete duplicate items from space separated item list
dnl
dnl @param list the list to be unduplicated
dnl @return ${u_list} results are stored to this variable
dnl
dnl----------------------------------------------------------------------
AC_DEFUN([UNIQUE], [
    u_list=
    dup="no"
    for item in $1 ; do
	for i in $u_list ; do
	    if test "x$i" = "x$item" ; then
		dup="yes"
	    fi
	done
	if test "x$dup" = "xno"; then
	    u_list="$u_list $item"
	fi
	dup="no"
    done
])


		
