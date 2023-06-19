#!/bin/bash

num_links_sweep=( 7 8 10 12 15 17 20 30 100 )
chain_sweep_urdf_dir=$AUTOMATION_PREFIX/experiments/chain_sweep_urdfs
for num_links in "${num_links_sweep[@]}"
do
    cd $AUTOMATION_PREFIX/build_scripts
    #export URDF_FILE="$chain_sweep_urdf_dir/chain_$num_links.urdf"
    export URDF_FILE="$AUTOMATION_PREFIX/urdfs/iiwa_rbd_accel.urdf"
    make generate
    make push
    cd $AUTOMATION_PREFIX/experiments
    vivado -mode batch -source rbd_accel_synth.tcl -tclargs $num_links
done
