#!/bin/bash

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
	echo -e "\e[1;33mWARN in ${pkg_list[$pkg_i]}\e[m"
	warn_list[${#warn_list[*]}]=${pkg_list[$pkg_i]}
    fi
}

function print-usage {
    echo "$0 : [option] package_name "
    echo " [option]"
    echo "   --help       : print this message"
}

#trap 'kill -s HUP $$ ' INT TERM
case $1 in
    -h|--help)
	print-usage; exit 0;
esac

# profile
rospack profile > /dev/null

# listap all packages
package_list_names=${@:-`rospack list-names`}
for pkg in $package_list_names; do
    fullpath_pkg=`rospack find $pkg`;
    if [ "$fullpath_pkg" ] ; then
	echo "package:$pkg -> $fullpath_pkg"
	pkg_list[${#pkg_list[*]}]=$fullpath_pkg
    fi
done

if [ "" != "$ROS_HOME" ] ; then
    roshomedir="$ROS_HOME";
else
    roshomedir="$HOME/.ros";
fi

# generate msg file
for pkg_i in $(seq 0 $((${#pkg_list[@]} - 1))); do
    pkg=${pkg_list[$pkg_i]}
    echo -e "\e[1;31mgenerating... $pkg_i/${#pkg_list[@]}\e[m"
    pkg_name=`basename $pkg`
    if [ -e $pkg/msg/ ] ; then
	for file in `find $pkg/msg -type f -name "*.msg"`; do
	    `rospack find roseus`/scripts/genmsg_eus $file;
	    check-error
	done
    fi
    if [ -e $pkg/srv/ ] ; then
	for file in `find $pkg/srv -type f -name "*.srv"`; do
	    `rospack find roseus`/scripts/gensrv_eus $file;
	    check-error
	done
    fi
    rospack depends $pkg_name > /dev/null ; check-warn ; ## just for check depends error
    `rospack find roseus`/scripts/genmanifest_eus $pkg_name
    check-error
done

if [ $((${#warn_list[@]})) -gt 0 ] ; then
    echo -e "\e[1;33m[WARNING] occurred while processing $0, missing dependencies?\e[m"
    for warn_i in $(seq 0 $((${#warn_list[@]} - 1))); do
	warn=${warn_list[$warn_i]}
	echo -e "\e[1;33m$warn\e[m"
	rospack depends -q `basename $warn`
    done
fi

touch ${roshomedir}/roseus/msggenerated

if [ $((${#err_list[@]})) -gt 0 ] ; then
    echo -e "\e[1;31m[ERROR] occurred while processing $0\e[m"
    for err_i in $(seq 0 $((${#err_list[@]} - 1))); do
	err=${err_list[$err_i]}
	echo -e "\e[1;31m$err\e[m"
    done
    exit 1
fi


