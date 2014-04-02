#!/bin/bash

EUSLISP_PKGDIR=`rospack find euslisp`
EUS2_EXE=`find $EUSLISP_PKGDIR -name eus2`

if [ ! "$EUS2_EXE" ] ; then
    echo -e "\e[1;31mERROR in eus2 is not found\e[m"
    exit 1
fi

function generate-msg-srv {
    local dir=$1;
    echo $dir
    cd $dir; rm -fr msg/eus srv/eus; if [ -e build ]; then cd build; make ROSBUILD_genmsg_eus; fi
}

function check-error {
    if [ "$?" != "0" ] ; then
	echo -e "\e[1;31mERROR in ${pkg_list[$pkg_i]}\e[m"
	err_list[${#err_list[*]}]=${pkg_list[$pkg_i]}
    fi
}
function check-warn {
    if [ "$?" != "0" ] ; then
	echo -e "\e[1;35mWARN in ${pkg_list[$pkg_i]}\e[m"
	warn_list[${#warn_list[*]}]=${pkg_list[$pkg_i]}
    fi
}

function print-usage {
    echo "Usage $0 : [option] [package_name] "
    echo "    (no option)     : generate for all packages that previously build "
    echo "    [package_name]  : generate for the [package_name] package"
    echo "    --all           : generate for all packages in the ROS_PACKAGE_PATH"
    echo "    --shared        : generate for all packages in the ROS_PACKAGE_PATH with ROS_NOBUILD files"
    echo "    --compile       : compile generated message files"
    echo "    --help          : print this message"
}

#trap 'kill -s HUP $$ ' INT TERM
ALL=No
SHARED=No
COMPILE=No
while [ $# -gt 0 ]
do
    case $1 in
	-h|--help)
	    print-usage; exit 0;;
	-s|--shared)
            SHARED=Yes;;
	-a|--all)
            ALL=Yes;;
        -c|--compile)
            COMPILE=Yes;;
	*) break;;
    esac
    shift
done

# profile
rospack profile > /dev/null


if [ "" != "$ROS_HOME" ] ; then
    roshomedir="$ROS_HOME";
else
    roshomedir="$HOME/.ros";
fi

mkdir -p $roshomedir/roseus/$ROS_DISTRO
# listap all packages

if [ "${ALL}" = "Yes" ]; then
    package_list_names=${@:-`rospack list-names`}
elif [ "${SHARED}" = "Yes" ] ;  then
    package_list_names=${@:-`for pkg in \`rospack list | cut -d\\  -f 2\`; do if [ -e $pkg/ROS_NOBUILD ]; then echo \`basename $pkg\`; fi; done`}
else
    package_list_names=${@:-`cd $roshomedir/roseus/$ROS_DISTRO; find ./ -maxdepth 1 -type d -print | sed s%^./%%g`}
fi
for pkg in $package_list_names; do
    fullpath_pkg=`rospack find $pkg`;
    if [ "$fullpath_pkg" ] ; then
	echo "package:$pkg -> $fullpath_pkg"
	pkg_list[${#pkg_list[*]}]=$fullpath_pkg
    fi
done

echo -e "\e[1;32mgenerating... ${#pkg_list[@]} files with ALL=${ALL}, SHARED=${SHARED}, COMPILE=${COMPILE} option\e[m"

# generate msg file
for pkg_i in $(seq 0 $((${#pkg_list[@]} - 1))); do
    pkg=${pkg_list[$pkg_i]}
    echo -e "\e[1;31mgenerating... $pkg_i/${#pkg_list[@]}\e[m"
    pkg_name=`basename $pkg`
    if [ -e $pkg/msg/ ] ; then
	for file in `find $pkg/msg -type f -name "*.msg"`; do
	    echo -e "\e[1;32mgenerating msg... ${file}\e[m"
	    `rospack find geneus`/scripts/genmsg_eus $file;
	    check-error
	done
    fi
    if [ -e $pkg/srv/ ] ; then
	for file in `find $pkg/srv -type f -name "*.srv"`; do
	    echo -e "\e[1;32mgenerating srv... ${file}\e[m"
	    `rospack find geneus`/scripts/gensrv_eus $file;
	    check-error
	done
    fi
    rospack depends $pkg_name > /dev/null || (check-warn) ; ## just for check depends error
    echo -e "\e[1;32mgenerating manifest... ${pkg_name}\e[m"
    `rospack find geneus`/scripts/genmanifest_eus $pkg_name
    check-error
    if [ "${COMPILE}" = "Yes" ]; then
        rosrun roseus roseus "(progn (setq lisp::*error-handler* #'(lambda (&rest args) (print args *error-output*)(exit 1))) (setq ros::*compile-message* t) (ros::load-ros-manifest \"$pkg_name\") (exit 0))"
        check-error
    fi
done

if [ $((${#warn_list[@]})) -gt 0 ] ; then
    echo -e "\e[1;33m[WARNING] occurred while processing $0, missing dependencies?\e[m"
    for warn_i in $(seq 0 $((${#warn_list[@]} - 1))); do
	warn=${warn_list[$warn_i]}
	echo -e "\e[1;33m$warn\e[m"
	rospack depends -q `basename $warn`
    done
fi

if [ $((${#err_list[@]})) -gt 0 ] ; then
    echo -e "\e[1;31m[ERROR] occurred while processing $0\e[m"
    for err_i in $(seq 0 $((${#err_list[@]} - 1))); do
	err=${err_list[$err_i]}
	echo -e "\e[1;31m$err\e[m"
    done
    exit 1
fi


