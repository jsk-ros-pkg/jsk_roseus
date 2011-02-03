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

#trap 'kill -s HUP $$ ' INT TERM

# profile
rospack profile

# listap all packages
package_list_names=${@:-`rospack list-names`}
for pkg in $package_list_names; do
    echo "package:$pkg"
    pkg_list[${#pkg_list[*]}]=`rospack find $pkg`
done

#rm */eus directory
for pkg_i in $(seq 0 $((${#pkg_list[@]} - 1))); do
    pkg=${pkg_list[$pkg_i]}
    if [ -e $pkg/msg ] ; then echo "rm $pkg/msg/eus"; rm -fr $pkg/msg/eus; fi
    if [ -e $pkg/srv ] ; then echo "rm $pkg/srv/eus"; rm -fr $pkg/srv/eus; fi
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
    if [ -e $pkg/msg/ ] ; then
	for file in `find $pkg/msg -type f -name "*.msg"`; do
	    echo $file
	    `rospack find roseus`/scripts/genmsg_eus $file;
	    check-error
	done
    fi
    if [ -e $pkg/srv/ ] ; then
	for file in `find $pkg/srv -type f -name "*.srv"`; do
	    echo $file
	    `rospack find roseus`/scripts/gensrv_eus $file;
	    check-error
	done
    fi
    pkg_name=`basename $pkg`
    if [ ! -e $roshomedir/roseus/$pkg_name ] ; then
	mkdir -p $roshomedir/roseus/$pkg_name;
    fi
    depends=`rospack depends $pkg_name`
    `rospack find roseus`/scripts/genmanifest_eus "$roshomedir/roseus/$pkg_name/_manifest.l" "$depends $pkg_name";
done

if [ $((${#err_list[@]})) -gt 0 ] ; then
    echo -e "\e[1;31mERROR occurred while processing $0\e[m"
    for err_i in $(seq 0 $((${#err_list[@]} - 1))); do
	err=${err_list[$err_i]}
	echo -e "\e[1;31m$err\e[m"
    done
    exit 1
fi





