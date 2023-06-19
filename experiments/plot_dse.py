#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import os

from URDFGenerator import URDFGenerator
from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration

max_DSE_PEs = 10
max_block_size = 10

results_dir = "results_csv/single_robot_dse"

robot_names = [
        "iiwa_rbd_accel",
        "hyq_simple",
        "baxter_simple"
]

for robot_name in robot_names:
    urdf_file = "../urdfs" + "/" + robot_name + ".urdf"
    parser = URDFParser()
    robot = parser.parse(urdf_file)
    fpga_codegen = FPGACodegen(robot)

    print(robot_name)

    dse = DesignSpaceExploration(robot, max_DSE_PEs, max_block_size)

    plot_filename = robot_name + "_dse.pdf"
    robot_csvdir = os.path.join(results_dir, robot_name + "_csv")

    dse.plot_full_dse(plot_filename, robot_csvdir)

    print("----------")

urdf_file = "quadruped_manip_length_sweep/quadruped_manip_19_7_3_3_3_3.urdf"
robot_name = "spot_manipulator"
parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

print(robot_name)

dse = DesignSpaceExploration(robot, max_DSE_PEs, max_block_size)
plot_filename = robot_name + "_dse.pdf"
robot_csvdir = os.path.join(results_dir, robot_name + "_csv")
dse.plot_full_dse(plot_filename, robot_csvdir)

print("----------")

urdf_file = "tine_length_sweep/arm_3_fingers_15_6_3_3_3.urdf"
robot_name = "jaco"
parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

print(robot_name)

dse = DesignSpaceExploration(robot, max_DSE_PEs, max_block_size)
plot_filename = robot_name + "_dse.pdf"
robot_csvdir = os.path.join(results_dir, robot_name + "_csv")
dse.plot_full_dse(plot_filename, robot_csvdir)

print("----------")
