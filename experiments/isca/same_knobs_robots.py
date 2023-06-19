#!/usr/bin/python3
import numpy as np
import matplotlib as mpl
from matplotlib.patches import Patch
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration


robots = [
        "iiwa",
        "HyQ",
        "Baxter",
        "jaco-2",
        "jaco-3",
        "HyQ+arm",
]

urdf_files = [
    "../../urdfs/iiwa_rbd_accel.urdf",
    "../../urdfs/hyq_simple.urdf",
    "../../urdfs/baxter_simple.urdf",
    "../handle_length_sweep/fork_12_6_3_3.urdf",
    "../tine_length_sweep/arm_3_fingers_15_6_3_3_3.urdf",
    "../quadruped_manip_length_sweep/quadruped_manip_19_7_3_3_3_3.urdf"
]

collected_data_filenames = [
    "csv/iiwa_dse_data",
    "csv/hyq_dse_data",
    "csv/baxter_dse_data",
    "csv/jaco2_dse_data",
    "csv/jaco3_dse_data",
    "csv/hyq_manip_dse_data",
]

num_fproc_PEs = 7
num_bproc_PEs = 7
block_size = 7

for i,collected_data_filename in enumerate(collected_data_filenames):
    with open(collected_data_filename+".csv", "r") as csvfile:
        csvreader = csv.reader(csvfile)
        for j,row in enumerate(csvreader):
            if j == 0:
                design_point_latencies = np.array(row).astype(np.float)
            elif j == 1:
                design_point_lut_util = np.array(row).astype(np.float)
            elif j == 2:
                design_point_dsp_util = np.array(row).astype(np.float)
            elif j == 3:
                design_point_num_fproc_PEs = np.array(row).astype(np.int32)
            elif j == 4:
                design_point_num_bproc_PEs = np.array(row).astype(np.int32)
            elif j == 5:
                design_point_block_size = np.array(row).astype(np.int32)
            elif j == 6:
                is_efficient = np.array([x == "True" for x in row])

    urdf_file = urdf_files[i]

    print(urdf_file)

    parser = URDFParser()
    robot = parser.parse(urdf_file)
    fpga_codegen = FPGACodegen(robot)
    dse = DesignSpaceExploration(robot, -1, -1)

    design_point_idx = np.argmin(design_point_latencies)
    print("optimal knob latency:"+str(design_point_latencies[design_point_idx]))
    print("optimal knob LUTs:"+str(design_point_lut_util[design_point_idx]))

    design_point_idx = np.argwhere(np.logical_and(design_point_block_size == block_size, np.logical_and(design_point_num_fproc_PEs == num_fproc_PEs, design_point_num_bproc_PEs == num_bproc_PEs)) == True)[0]

    print("same knob latency:"+str(design_point_latencies[design_point_idx]))
    print("same knob LUTs:"+str(design_point_lut_util[design_point_idx]))
