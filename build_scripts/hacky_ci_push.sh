#!/bin/bash
set -x
set -e

if [[ -z "${RBD_BASE}" ]]; then
    echo "RBD_BASE env var not set. Can't use build scripts."
    exit 1
fi

# replace and backup current file in same file tree as base repo

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GENERATOR_DIR="$DIR/../generators"
BUILD_DIR="$DIR/build"

if [ "$1" == "all" ]; then
    echo "Pushing entire build file tree to RBD_BASE"
    # trailing slash for rsync to work
    rsync -av $BUILD_DIR/ $RBD_BASE 
    exit
fi

# for each file in the arg list
for GEN_FILE_PATH in "$@"
do
    if [ ! -f $GEN_FILE_PATH ]; then
        echo "File not generated!"
        exit 1
    else
        TREE_REL_PATH=$(realpath --relative-to="$BUILD_DIR" "$GEN_FILE_PATH")
        CURR_RBD_FILE="$RBD_BASE/$TREE_REL_PATH"
        cp $GEN_FILE_PATH $CURR_RBD_FILE
    fi
done
