#!/bin/bash
set -x
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GENERATOR_DIR="$DIR/../generators"
BUILD_DIR="$DIR/build"

# for each file in arg list

for PYFILE in "$@"
do
    TREE_REL_PATH=$(realpath --relative-to="$GENERATOR_DIR" "$PYFILE")
    BUILD_DEST_DIR="$( dirname $BUILD_DIR/$TREE_REL_PATH )"

    mkdir -p $BUILD_DEST_DIR

    python3 $PYFILE -d $BUILD_DEST_DIR
done
