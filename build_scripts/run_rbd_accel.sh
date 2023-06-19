#!/bin/bash
set -x
set -e

if [[ -z "${RBD_BASE}" ]]; then
    echo "RBD_BASE env var not set. Can't use build scripts."
    exit 1
fi

echo "Now running simulation..."
$RBD_BASE/PackagedCode/verilator/bin/ubuntu.exe
