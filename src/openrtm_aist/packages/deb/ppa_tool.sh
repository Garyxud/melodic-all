#!/bin/sh
#
# @file ppa_tool.sh
# @brief Launchpad/PPA packaging and publising tool for multiple distroseries
# @author Noriaki Ando <n-ando@aist.go.jp>
# @date Fri Jul  6 02:33:15 JST 2012
#

#
# 0. Before publishing your package on the PPA, you should create
#    account on Launchpad, and register your OpenPGP key, and create
#    PPA place.
#
# 1. debian/changelog should have the following text in the first line
#
#    ex. openrtm-aist (1.1.0-1~ppa1lucid) lucid; urgency=low
#        <package name> (X.Y.Z-N~ppaMxxxx) yyyyyy; urgency=zzzzz
#    ~ppa[number] should be added to normal version string of deb package.
#
# 2. Before making dput your package to PPA, it is recommended to make
#    source package with '-s' option and try to build it in the local
#    enviuronment.
#
#    $ ppa_tool.sh -p password -s lucid
#    $ cd ../ ; mkdir ../tmp
#    $ tar -C ../tmp xvzf ../your_package.tar.gz
#    $ cd ../tmp/your_package ; debuild -us -uc
#
# 3. If your source pacakge is built without any error, now you can
#    make dput your source package to your PPA.
#
#    $ cd -
#    $ ppa_tool.sh -p password -t ppa:your_account/ppa_place -a
#    # -a: dput your source to all current supported distro-versions
#

# Global variables:
#
#  DEBUG:
#    Debug flag. some functions print debug messages if some value is set.
#  PASSWORD:
#    Password to unlock gpgkey. This is used when package is signed.
#  USERNAME:
#    User name to specify GPG key to sign source package.
#  GPGKEYID
#    GPG key ID to specify GPG key.
#  VERSION
#    Version number to be inserted to the package name.
#  PPAHOST
#    PPA target host name to be given to dput command.
#  TARGETS
#    A list of distroseries name.
#  SUPPORTED
#    Supported distroseries versions.
#  SUPPORTED_LIST
#    Supported distroseries versions with version numbers. Separated by comma.
#

get_distroseries()
{
    if ! test -f /tmp/meta-release; then
        wget -q -O /tmp/meta-release http://changelogs.ubuntu.com/meta-release
    fi
    SUPPORTED=`awk 'BEGIN{RS="";FS="\n";}{if ($5 == "Supported: 1"){sub("Dist: ",""); sub(" ","",$1); printf("%s ",$1);}}END{printf("\n")}' /tmp/meta-release`
    SUPPORTED_LIST=`awk 'BEGIN{RS="";FS="\n";}{if ($5 == "Supported: 1"){sub("Dist: ",""); sub("Version: ",""); printf("%s\t%s,",$1,$3);}}' /tmp/meta-release`
}

print_short_usage()
{
    get_distroseries
    echo "\nUsage: $(basename $0) [OPTION]... [DISTROSERIES]..."
    echo ""
    echo "Optinos:"
    echo "    -p <password>  password to unlock gpg key"
#    echo "  -u <username>  username for gpg key"
#    echo "  -k <key ID>    gpg key ID"
    echo "    -v <version>   versioned package"
    echo "    -t <ppa host>  target ppa host location"
    echo "    -a             publish for all supported distroseries"
    echo "                   [DISTROSERIES] = $SUPPORTED"
    echo "    -s             only create source package in local"
    echo "    -h             print this help"
    echo ""
}

print_usage()
{
    print_short_usage
    echo "Supported Distroseries:"
    local __IFS=$IFS
    IFS=","          # SUPPORTED_LIST'S delimter is ","
    for dist in $SUPPORTED_LIST; do
        echo "    $dist"
    done
    IFS=$__IFS
    echo ""
    echo "EXAMPLES"
    echo "  Send packages for only LTS to openrtm/releases ppa."
    echo ""
    echo "    $(basename $0) -p hoge -t ppa:openrtm/releases hardy lucid"
    echo ""
    echo "  Send packages for all supported versions to my_ppa. Password is"
    echo "  required to sign for each packages."
    echo ""
    echo "    $(basename $0) -t my_ppa -a"
    echo ""
    echo "  Only creating source package without sending it to ppa host."
    echo ""
    echo "    $(basename $0) -t my_ppa -t "
    echo ""
    exit 0
}

get_opt()
{
    # -p <password>        password to unlock gpg key
    # -u <username>        username for gpg key
    # -k <key ID>          gpg key ID
    # -v <version>         versioned package
    #
    PASSWORD=""
    PPAHOST=""
    VERSION=""
    NO_DPUT=""
    while getopts "p:u:k:t:v:hsa" OPT; do
        case $OPT in
            \?) print_short_usage; exit 1;;
            p) PASSWORD="$OPTARG";;
            u) USERNAME="$OPTARG";;
            k) GPGKEYID="$OPTARG";;
            t) PPAHOST="$OPTARG";;
            v) VERSION="$OPTARG";;
            s) NO_DPUT="True";;
            a) ALLDIST="True";;
            h) print_usage; exit 0;
        esac
    done
    shift $(( $OPTIND - 1))
    TARGETS=$@

    # No distroseries without -a is not allowed
    if test $# -eq 0; then
        if test "x$ALLDIST" = "x"; then
            echo "ERROR: no distroseries name specified."
            print_short_usage
            exit 1
        else
            # Getting supported distroseries
            get_distroseries
            TARGETS=$SUPPORTED
        fi
    fi

    # No -t option without -s option is now allowed
    if test "x$PPAHOST" = "x"; then
        if test "x$NO_DPUT" = "x"; then
            echo "ERROR: Option -t with PPA host is necessary"
            print_short_usage
            exit 1
        fi
    fi
    # DEBUG
    if ! test "x$DEBUG" = "x"; then
        echo "PASSWORD: " $PASSWORD
        echo "USERNAME: " $USERNAME
        echo "GPGKEYID: " $GPGKEYID
        echo "VERSION : " $VERSION
        echo "PPAHOST : " $PPAHOST
        echo "TARGETS : " $TARGETS
    fi
}

get_package_name()
{
    if test -f debian/changelog; then
        PACKAGE_NAME=`head -n 1 debian/changelog | awk '{print $1;}'`
        echo "Pacakge base name is: " $PACKAGE_NAME
    else
        echo "ERROR: debian/changelog not found."
        exit 1
    fi
}

check_debiandir()
{
    # checking debian directory
    if ! test -d debian ; then
        echo "ERROR: debian directory not found"
        echo "       This directory should have debian directory"
        echo "       and control, changelog, etc in it."
        exit -1
    fi
    #  checking control file
    if ! test -f debian/control; then # if control not found,
        for d in $TARGETS; do         # control.<codename> should exist
            if $DEBUG; then
                echo "Checking debian/control.$d"
            fi
            if ! test -f debian/control.$d; then
                echo "ERROR: No control files found."
                echo "       debian dir should have at least one control file"
                echo "       or control.<release name> files."
                exit 1
            fi
        done
    else
        echo "\ndebian/control found."
        for d in $TARGETS; do
            if test -f debian/control.$d; then
                echo "debian/control.$d used for $d"
            else
                echo "debian/control used for $d"
            fi
        done
        echo ""
    fi
    # checking changelog file
    if ! test -f debian/changelog ; then
        echo "ERROR: No changelog file found."
        exit -1
    fi
}

clean_source()
{
    if test -f "Makefile"; then
        make clean
        make distclean
    fi
}

backup_debfiles()
{
    trap reset_debfiles 2
    tar czf /tmp/debian.$PACKAGE_NAME.$$.tgz debian
    echo "Backed up debian to /tmp/debian.$PACKAGE_NAME.$$.tgz"
}


reset_debfiles()
{
    if test -f /tmp/debian.$PACKAGE_NAME.$$.tgz; then
        rm -rf debian
        tar xzf /tmp/debian.$PACKAGE_NAME.$$.tgz
        echo "debian directory has been restored."
    fi
    trap 2
}

modify_changelog()
{
    # modify_changelog <release name>
    if ! test $# -eq 1; then
        echo "Internal ERROR: modify_changelog should have one arg."
        finalize
        exit 1
    fi
    local RELEASE=$1
    if test -f debian/changelog.ppa; then
        cp debian/changelog.ppa debian/changelog
    fi
    sed -i \
        -e "1,1 s/^\(.*~ppa[0-9]\+\)\(.*\)\().*\)$/\1${RELEASE}\3/" \
        -e "1,1 s/^\(.*) \)\(.*\)\(;.*\)$/\1${RELEASE}\3/"      \
        debian/changelog
    echo "debian/changelog modified."
    echo "$(head -n 1 debian/changelog)"
}


modify_control()
{
    # modify_control <release name>
    if ! test $# -eq 1; then
        echo "Internal ERROR: modify_control should have one arg."
        finalize
        exit 1
    fi
    local RELEASE=$1
    if test -f debian/control.${RELEASE}; then
        rm debian/control
        cp debian/control.${RELEASE} debian/control
        echo "Using debian/control.${RELEASE}"
    fi
}

build_and_dput()
{
    if test "x$PASSWORD" = "x"; then
        debuild -i -I -S -sa > /tmp/debuild.$$.log
    else
        export LC_ALL=C
        expect -c "
        set timeout 60
        spawn debuild -i -I -S -sa 
        expect passphrase:\ ; send \"$PASSWORD\r\"
        expect passphrase:\ ; send \"$PASSWORD\r\"
        interact
        " > /tmp/debuild.$$.log
    fi
    # getting changes file name
    ls -al /tmp/debuild.$$.log
    local CHANGES="$(sed -n 's/^.*signfile \(.*\.changes\).*$/\1/p' /tmp/debuild.$$.log)"
    if test "x$NO_DPUT" = "x"; then
        echo -e "\nPublishing to $DPUT_TARGET with ../$CHANGES"
        dput $PPAHOST ../$CHANGES
        rm ../$(basename $CHANGES _source.changes)*
    fi
}

insert_version()
{
    if test "x$VERSION" = "x"; then
        return 0
    fi
    local ver=$VERSION
    local pkg=$PACKAGE_NAME
    echo "Replacing $pkg to $pkg$ver"
    # control files
    sed -i "s/$pkg/$pkg$ver/g" debian/control*
    # changelog
    sed -i "s/$pkg/$pkg$ver/g" debian/changelog
    # rules
    sed -i "s/debian\/$pkg/debian\/$pkg$ver/g" debian/rules
}

finalize()
{
    reset_debfiles
}


#------------------------------------------------------------
# main
#------------------------------------------------------------
# checking and getting options
if test $# -eq 0 ; then
    print_usage
    exit -1
fi
get_opt $@

# getting package name and checking debian directory and files
check_debiandir
get_package_name

# backup debian directory

for RELNAME in $TARGETS; do
    backup_debfiles
    clean_source
    modify_changelog $RELNAME
    modify_control   $RELNAME
    insert_version
    build_and_dput
    reset_debfiles
done

exit 0
