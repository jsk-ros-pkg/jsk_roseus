#!/bin/bash

function generate-msg-srv {
    local dir=$1;
    echo $dir
    cd $dir; rm -fr msg/eus srv/eus; if [ -e build ]; then cd build; make ROSBUILD_genmsg_eus; fi
}

function check-error {
    if [ "$?" != "0" ] ; then exit -1; fi
}

#trap 'kill -s HUP $$ ' INT TERM

# listap all packages
for pkg in `rospack list-names`; do
    echo "package:$pkg"
    pkg_list[${#pkg_list[*]}]=`rospack find $pkg`
done

#rm */eus directory
for pkg_i in $(seq 0 $((${#pkg_list[@]} - 1))); do
    pkg=${pkg_list[$pkg_i]}
    if [ -e $pkg/msg ] ; then echo "rm $pkg/msg/eus"; rm -fr $pkg/msg/eus; fi
    if [ -e $pkg/srv ] ; then echo "rm $pkg/srv/eus"; rm -fr $pkg/srv/eus; fi
done

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
done



