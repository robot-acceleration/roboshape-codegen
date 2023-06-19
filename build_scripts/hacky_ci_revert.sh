#!/bin/bash
set -x
set -e

if [[ -z "${RBD_BASE}" ]]; then
    echo "RBD_BASE env var not set. Can't use build scripts."
    exit 1
fi

# revert current copy of RAWFILE in rbd-accelerator to original
# handwritten version from git head

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GENERATOR_DIR="$DIR/../generators"
BUILD_DIR="$DIR/build"

if [ "$1" == "all" ]; then
    echo "Reverting rbd-accelerator to HEAD, undoing all generated files and deleting backups."
    git -C $RBD_BASE reset --hard HEAD
    find $RBD_BASE -name "*.backup" -type f -delete
    exit
fi

# for each file in the arg list
for GEN_FILE_PATH in "$@"
do
    TREE_REL_PATH=$(realpath --relative-to="$BUILD_DIR" "$GEN_FILE_PATH")
    CURR_RBD_FILE="$RBD_BASE/$TREE_REL_PATH"
    if [ ! -f $CURR_RBD_FILE ]; then
        echo "File not found in rbd-accelerator!"
        exit 1
    else
        git -C $RBD_BASE checkout -- $CURR_RBD_FILE
        rm $CURR_RBD_FILE.backup
        echo "Reverted $CURR_RBD_FILE to git head in rbd-accelerator."
    fi
done
