#!/bin/bash
# -*- tab-width: 8; -*-

#rosrun geneus gen_eus.py -m libccd -o /home/k-okada/catkin_ws/ws_msggen/devel/share/roseus/ros/libccd

function print-usage {
    echo "Usage $0 : [option] [package_name] "
    echo "    (no option)     : generate for all packages that previously build "
    echo "    [package_name]  : generate for the [package_name] package"
    echo "    --all           : generate for all packages in the ROS_PACKAGE_PATH"
    echo "    --compile       : compile generated message files"
    echo "    --help          : print this message"
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


ALL=No
SHARED=No
COMPILE=No
while [ $# -gt 0 ]
do
    case $1 in
	-h|--help)
	    print-usage; exit 0;;
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


IFS=':' read -ra TMP <<< "$CMAKE_PREFIX_PATH"
export output_dir=${TMP[0]}/share/roseus/ros

if [ "${ALL}" = "Yes" ]; then
    package_list_names=${@:-`rospack list-names`}
else
    package_list_names=${@:-`cd $output_dir; find ./ -maxdepth 1 -type d -print | sed s%^./%%g`}
fi

for pkg in $package_list_names; do
    fullpath_pkg=`rospack find $pkg`;
    if [ "$fullpath_pkg" ] ; then
	echo "package:$pkg -> $fullpath_pkg"
	pkg_list[${#pkg_list[*]}]=$fullpath_pkg
    fi
done

echo -e "\e[1;32mgenerating... ${#pkg_list[@]} files with ALL=${ALL}, COMPILE=${COMPILE} option\e[m"
echo -e "\e[1;32mwriting output to...${output_dir}\e[m"

# generate msg file
function run-pkg()
{
    pkg=$(rospack find $1)
    echo -e "\e[1;31mgenerating... $pkg_i/${#pkg_list[@]}\e[m"
    pkg_name=`basename $pkg`
    pkg_depends=`rospack depends ${pkg_name}`
    pkg_includes="-I$pkg_name:`rospack find $pkg_name`/msg"
    pkg_msg_depends=""
    for pkg_d in $pkg_depends; do
        pkg_d_path=`rospack find $pkg_d`
        if [ -e ${pkg_d_path}/msg -o -e ${pkg_d_path}/srv ]; then
            pkg_msg_depends="$pkg_msg_depends $pkg_d"
            pkg_includes="$pkg_includes -I$pkg_d:${pkg_d_path}/msg"
        fi
    done
    if [ -e $pkg/msg/ ] ; then
	for file in `find $pkg/msg -type f -name "*.msg"`; do
	    echo -e "\e[1;32mgenerating msg... ${file}\e[m"
	    rosrun geneus gen_eus.py -p $pkg_name -o ${output_dir}/${pkg_name}/msg $pkg_includes $file
	    check-error
	done
    fi
    if [ -e $pkg/srv/ ] ; then
	for file in `find $pkg/srv -type f -name "*.srv"`; do
	    echo -e "\e[1;32mgenerating srv... ${file}\e[m"
	    rosrun geneus gen_eus.py -p $pkg_name -o ${output_dir}/${pkg_name}/srv $pkg_includes $file
	    check-error
	done
    fi
    rospack depends $pkg_name > /dev/null || (check-warn) ; ## just for check depends error
    echo -e "\e[1;32mgenerating manifest... ${output_dir}/${pkg_name}/manifest.l\e[m"
    rosrun geneus gen_eus.py -m -o ${output_dir}/${pkg_name} ${pkg_name} ${pkg_msg_depends}
    check-error
    if [ "${COMPILE}" = "Yes" ]; then
        rosrun roseus roseus "(progn (setq lisp::*error-handler* #'(lambda (&rest args) (print args *error-output*)(exit 1))) (setq ros::*compile-message* t) (ros::load-ros-manifest \"$pkg_name\") (exit 0))"
        check-error
    fi
}

export -f run-pkg
export -f check-warn
export -f check-error

echo $package_list_names | xargs -d' ' -n 1 -P $(grep -c processor /proc/cpuinfo) -I % bash -c "run-pkg %"

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
