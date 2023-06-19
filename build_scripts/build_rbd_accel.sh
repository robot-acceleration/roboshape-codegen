#!/bin/bash
set -x
set -e

if [[ -z "${RBD_BASE}" ]]; then
    echo "RBD_BASE env var not set. Can't use build scripts."
    exit 1
fi

echo "Regenerating build..."
cd $RBD_BASE/PackagedCode
make build.verilator NUM_LINKS=7
