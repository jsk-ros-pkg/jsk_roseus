#!/bin/bash

# default arguments
SUB_DIRECTORIES="jskeus/eus jskeus"
TARGET_DIRECTORY=`rospack find euslisp`;
OUTPUT=/tmp/create_changelog.rst

function print-usage {
    echo "Usage $0 : (options)"
    echo "    --sub-directories  : sub directories for git source tree (default : \"$SUB_DIRECTORIES\")"
    echo "    --target-directory : target directory including CHANGELOG.rst (default : $TARGET_DIRECTORY)"
    echo "    --output           : output file name (default : $OUTPUT)"
    echo "    --help             : print this message"
}

while [ $# -gt 0 ]
do
    case $1 in
        -h|--help)
            print-usage; exit 0;;
        -s|--sub-directories)
            SUB_DIRECTORIES=$2;shift;;
        -t|--target-directory)
            TARGET_DIRECTORY=$2;shift;;
        -o|--output)
            OUTPUT=$2;shift;;
        *) break;;
    esac
    shift
done

LATEST_DATE=$(grep -m1 .\\.*\\. $TARGET_DIRECTORY/CHANGELOG.rst | cut -d\( -f2 | cut -d\) -f1)
# LATEST_DATE="2014-05-30" # for debug

echo "generate changelog"
echo "  TARGET DIR : $TARGET_DIRECTORY"
echo "  SUB_PACKAGE : $SUB_DIRECTORIES"
echo "  LATEST DATE in CHANGELOG.rst : $LATEST_DATE"
echo "  OUTPUT : $OUTPUT"
rm -f $OUTPUT $OUTPUT.*
for dir in $SUB_DIRECTORIES;
do
    cd $TARGET_DIRECTORY/$dir;
    rep_name=$(git remote -v |head -n1 | cut -d/  -f5|cut -d\  -f1); # Get repository name
    git log --oneline --after=$LATEST_DATE --no-merges --date=short --pretty=format:"* %ad %h ($rep_name) %s" >> $OUTPUT.$$;
done;
sort $OUTPUT.$$ -r > $OUTPUT
