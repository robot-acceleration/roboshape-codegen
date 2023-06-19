#!/usr/bin/python3
import numpy as np
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration

# original ordering: iiwa, hyq, baxter, jaco-2, jaco-3, hyq-manip
# new ordering: iiwa, hyq, jaco-2, baxter, jaco-3, hyq-manip

# original strategies: optimal, [strategies], heuristic

ordering = [0,1,3,2,4,5]

robots = [
        "iiwa",
        "HyQ",
        "Baxter",
        "jaco-2",
        "jaco-3",
        "HyQ-Manip"
]
robots = np.array(robots)[ordering, :]

urdf_files = [
    "../../urdfs/iiwa_rbd_accel.urdf",
    "../../urdfs/hyq_simple.urdf",
    "../../urdfs/baxter_simple.urdf",
    "../handle_length_sweep/fork_12_6_3_3.urdf",
    "../tine_length_sweep/arm_3_fingers_15_6_3_3_3.urdf",
    "../quadruped_manip_length_sweep/quadruped_manip_19_7_3_3_3_3.urdf"
]
urdf_files = np.array(urdf_files)[ordering, :]

strategies = [
        "# of links",
        "Longest chain",
        "Avg leaf depth",
        "Max descendents"
]
resource_val_strategy = [
        [7,12,15,12,15,19],
        [7, 3, 7, 9, 9, 7],
        [7, 3, 5, 9, 9, 4],
        [7, 3, 7,12,15, 7]
]
resource_val_strategy = np.array(resource_val_strategy)[:, ordering]

latency_strategy = [
        [],
        [],
        [],
        [],
        [],
        [],
]

utilization_strategy = [
        [],
        [],
        [],
        [],
        [],
        [],
]

for i,urdf_file in enumerate(urdf_files):
    print(urdf_file)

    parser = URDFParser()
    robot = parser.parse(urdf_file)
    fpga_codegen = FPGACodegen(robot)

    dse = DesignSpaceExploration(robot, -1, -1)

    for j,strategy in enumerate(strategies):
        print(strategy)
        num_fproc_PEs = num_bproc_PEs = block_size = resource_val_strategy[j][i]

        schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, block_size)

        fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
        len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
        bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
        len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
        matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(num_bproc_PEs)
        len_matmul_sched = matmul_sched_table_dict["len_block_minv_sched_per_matrix"]
        total_len_sched = len_fproc_sched*3 + len_bproc_sched + len_matmul_sched*2

        print("len_fproc_sched: "+str(len_fproc_sched)) 
        print("len_bproc_sched: "+str(len_bproc_sched))
        print("len_matmul_sched: "+str(len_matmul_sched))
        print("total: "+str(len_fproc_sched*3+len_bproc_sched+len_matmul_sched*2))

        latency_strategy[j].append(total_len_sched)
        utilization_strategy[j].append(dse.get_lut_usage(num_fproc_PEs, num_bproc_PEs, block_size, len_fproc_sched, robot.get_num_joints()))

        print("----")

    print("optimal (unconstrained):")


    num_fproc_PEs = dse.find_best_num_fproc_PEs()
    num_bproc_PEs = dse.find_best_num_bproc_PEs()
    block_size = robot.get_num_joints()

    print(num_fproc_PEs)
    print(num_bproc_PEs)
    print(block_size)

    schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, block_size)

    fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
    len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
    bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
    len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
    #matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(num_bproc_PEs)
    len_matmul_sched = 1
    total_len_sched = len_fproc_sched*3 + len_bproc_sched + len_matmul_sched*2

    print("len_fproc_sched: "+str(len_fproc_sched)) 
    print("len_bproc_sched: "+str(len_bproc_sched))
    print("len_matmul_sched: "+str(len_matmul_sched))
    print("total: "+str(len_fproc_sched*3+len_bproc_sched+len_matmul_sched*2))

    latency_strategy[4].append(total_len_sched)
    utilization_strategy[4].append(dse.get_lut_usage(num_fproc_PEs, num_bproc_PEs, block_size, len_fproc_sched, robot.get_num_joints()))

    print("----")

    #print("optimal (constrained):")

    #dse = DesignSpaceExploration(robot, -1, -1)

    #num_fproc_PEs, num_bproc_PEs, block_size, lut_usage, dsp_usage = dse.exhaustive_search()

    #print(num_fproc_PEs)
    #print(num_bproc_PEs)
    #print(block_size)
    #print(lut_usage)
    #print(dsp_usage)

    #schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, block_size)

    #fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
    #len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
    #bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
    #len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
    ##matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(num_bproc_PEs)
    #len_matmul_sched = 1
    #total_len_sched = len_fproc_sched*3 + len_bproc_sched + len_matmul_sched*2

    #print("len_fproc_sched: "+str(len_fproc_sched)) 
    #print("len_bproc_sched: "+str(len_bproc_sched))
    #print("len_matmul_sched: "+str(len_matmul_sched))
    #print("total: "+str(len_fproc_sched*3+len_bproc_sched+len_matmul_sched*2))

    #dse.get_lut_usage(num_fproc_PEs, num_bproc_PEs, block_size, len_fproc_sched, robot.get_num_joints(), print_debug=True)
    #dse.get_dsp_usage(num_fproc_PEs, num_bproc_PEs, block_size, print_debug=True)

    #latency_strategy[5].append(total_len_sched)

    #print("----")

### plotting

x = np.arange(len(robots))
width = 0.15

fig, ax = plt.subplots(figsize=(5,3))
rects_0 = ax.bar(x - 2*width, latency_strategy[0], width, label=strategies[0])
rects_1 = ax.bar(x - 1*width, latency_strategy[1], width, label=strategies[1])
rects_2 = ax.bar(x - 0*width, latency_strategy[2], width, label=strategies[2])
rects_3 = ax.bar(x + 1*width, latency_strategy[3], width, label=strategies[3])
rects_4 = ax.bar(x + 2*width, latency_strategy[4], width, label="Optimal DSE")

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4
]
labels = strategies
labels.append("Optimal DSE")

ax.set_ylabel("Latency [cycles]")
ax.set_xticks(x)
ax.set_xticklabels(robots)
ax.legend(handles, labels, ncol=2, loc="upper left", fontsize=9)

fig.tight_layout()

plt.savefig("out/total_latency_unconstrained_baselines.pdf")
plt.close(fig)

##

fig, ax = plt.subplots(figsize=(5,3))
rects_0 = ax.bar(x - 2*width, utilization_strategy[0], width, label=strategies[0])
rects_1 = ax.bar(x - 1*width, utilization_strategy[1], width, label=strategies[1])
rects_2 = ax.bar(x - 0*width, utilization_strategy[2], width, label=strategies[2])
rects_3 = ax.bar(x + 1*width, utilization_strategy[3], width, label=strategies[3])
rects_4 = ax.bar(x + 2*width, utilization_strategy[4], width, label="Optimal DSE")

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4
]
labels = strategies
labels.append("Optimal DSE")

ax.set_ylabel("Resource Utilization [LUTs]")

y_tick_vals = ax.get_yticks()
ax.set_yticklabels(['{x}k'.format(x=int(x//1000)) for x in y_tick_vals])

ax.set_xticks(x)
ax.set_xticklabels(robots)
ax.legend(handles, labels, ncol=2, loc="upper left", fontsize=8)

fig.tight_layout()

plt.savefig("out/utilization_unconstrained_baselines.pdf")
plt.close(fig)
