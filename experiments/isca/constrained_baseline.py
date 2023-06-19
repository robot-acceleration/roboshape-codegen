#!/usr/bin/python3
import numpy as np
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration

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
strategies = [
        "Maximal allocation",
        "Optimal DSE",
]
latency_strategy_vcu118 = [
        [],
        [],
]
lut_utilization_strategy_vcu118 = [
        [],
        [],
]
dsp_utilization_strategy_vcu118 = [
        [],
        [],
]
edp_utilization_strategy_vcu118 = [
        [],
        [],
]

latency_strategy_vc707 = [
        [],
        [],
]
lut_utilization_strategy_vc707 = [
        [],
        [],
]
dsp_utilization_strategy_vc707 = [
        [],
        [],
]
edp_utilization_strategy_vc707 = [
        [],
        [],
]

vcu118_luts = 1182240
vc707_luts = 303600

vcu118_dsps = 6840
vc707_dsps = 2800


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

    parser = URDFParser()
    robot = parser.parse(urdf_files[i])
    fpga_codegen = FPGACodegen(robot)
    dse = DesignSpaceExploration(robot, -1, -1)


    asc_ord_lut_util_indices = np.argsort(design_point_lut_util)
    design_point_lut_util = design_point_lut_util[asc_ord_lut_util_indices]
    design_point_dsp_util = design_point_dsp_util[asc_ord_lut_util_indices]
    design_point_latencies = design_point_latencies[asc_ord_lut_util_indices]
    design_point_num_fproc_PEs = design_point_num_fproc_PEs[asc_ord_lut_util_indices]
    design_point_num_bproc_PEs = design_point_num_bproc_PEs[asc_ord_lut_util_indices]
    design_point_block_size = design_point_block_size[asc_ord_lut_util_indices]
    is_efficient = is_efficient[asc_ord_lut_util_indices]

    # two fpgas
    for i in range(2):
        if i == 0:
            total_luts = vcu118_luts
            total_dsps = vcu118_dsps
        elif i == 1:
            total_luts = vc707_luts
            total_dsps = vc707_dsps

        lut_threshold = 0.83 * total_luts
        dsp_threshold = 0.83 * total_dsps

        indices_below_threshold = np.argwhere(np.logical_and(design_point_lut_util <= lut_threshold, design_point_dsp_util <= dsp_threshold)).squeeze()

        if indices_below_threshold.shape[0] > 0:
            threshold_idx = indices_below_threshold[-1]

            maximal_num_fproc_PEs = design_point_num_fproc_PEs[threshold_idx]
            maximal_num_bproc_PEs = design_point_num_bproc_PEs[threshold_idx]
            maximal_block_size = design_point_block_size[threshold_idx]
            maximal_latency = design_point_latencies[threshold_idx]
            maximal_lut_util = design_point_lut_util[threshold_idx]
            maximal_dsp_util = design_point_dsp_util[threshold_idx]

            if i == 0:
                latency_strategy_vcu118[0].append(maximal_latency)
                lut_utilization_strategy_vcu118[0].append(maximal_lut_util)
                dsp_utilization_strategy_vcu118[0].append(maximal_dsp_util)
            elif i == 1:
                latency_strategy_vc707[0].append(maximal_latency)
                lut_utilization_strategy_vc707[0].append(maximal_lut_util)
                dsp_utilization_strategy_vc707[0].append(maximal_dsp_util)

            constrained_design_pt_idx = indices_below_threshold[np.argmin(design_point_latencies[indices_below_threshold])]

            constrained_num_fproc_PEs = design_point_num_fproc_PEs[constrained_design_pt_idx]
            constrained_num_bproc_PEs = design_point_num_bproc_PEs[constrained_design_pt_idx]
            constrained_block_size = design_point_block_size[constrained_design_pt_idx]
            constrained_latency = design_point_latencies[constrained_design_pt_idx]
            constrained_lut_util = design_point_lut_util[constrained_design_pt_idx]
            constrained_dsp_util = design_point_dsp_util[constrained_design_pt_idx]

            if i == 0:
                latency_strategy_vcu118[1].append(constrained_latency)
                lut_utilization_strategy_vcu118[1].append(constrained_lut_util)
                dsp_utilization_strategy_vcu118[1].append(constrained_dsp_util)
            elif i == 1:
                latency_strategy_vc707[1].append(constrained_latency)
                lut_utilization_strategy_vc707[1].append(constrained_lut_util)
                dsp_utilization_strategy_vc707[1].append(constrained_dsp_util)
        else:
            # no valid design points below threshold
            print("No feasible design points")

            smallest_num_fproc_PEs = design_point_num_fproc_PEs[0]
            smallest_num_bproc_PEs = design_point_num_bproc_PEs[0]
            smallest_block_size = design_point_block_size[0]
            smallest_latency = design_point_latencies[0]
            smallest_lut_util = design_point_lut_util[0]
            smallest_dsp_util = design_point_dsp_util[0]

            # Debug
            print("--- smallest design point debug info ---")
            print(smallest_num_fproc_PEs)
            print(smallest_num_bproc_PEs)
            print(smallest_block_size)
            print(smallest_latency)
            print(smallest_lut_util)
            print(smallest_dsp_util)

            schedule_gen = fpga_codegen.get_schedule_gen(smallest_num_fproc_PEs, smallest_num_bproc_PEs, smallest_block_size)
            fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
            len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
            dse.get_lut_usage(smallest_num_fproc_PEs, smallest_num_bproc_PEs, smallest_block_size, len_fproc_sched, robot.get_num_joints(), print_debug=True, intermediate_muxing=False)
            dse.get_dsp_usage(smallest_num_fproc_PEs, smallest_num_bproc_PEs, smallest_block_size, print_debug=True)
            print("--- debug info end--")
            # Debug end

            if i == 0:
                latency_strategy_vcu118[0].append(0)
                lut_utilization_strategy_vcu118[0].append(0)
                dsp_utilization_strategy_vcu118[0].append(0)
                latency_strategy_vcu118[1].append(0)
                lut_utilization_strategy_vcu118[1].append(0)
                dsp_utilization_strategy_vcu118[1].append(0)
            elif i == 1:
                latency_strategy_vc707[0].append(0)
                lut_utilization_strategy_vc707[0].append(0)
                dsp_utilization_strategy_vc707[0].append(0)
                latency_strategy_vc707[1].append(0)
                lut_utilization_strategy_vc707[1].append(0)
                dsp_utilization_strategy_vc707[1].append(0)

### plotting

robots = [
        "iiwa",
        "HyQ",
        "Baxter",
        "jaco-2",
        "jaco-3",
        "HyQ-Manip"
]

x = np.arange(len(robots))
width = 0.15
offset = 0.05

fig, ax = plt.subplots(figsize=(5,2))
rects_0_vcu118 = ax.bar(x + 0.5*width + offset, latency_strategy_vcu118[0], width, label="VCU118 " + strategies[0])
rects_1_vcu118 = ax.bar(x + 1.5*width + offset, latency_strategy_vcu118[1], width, label="VCU118 " + strategies[1])

rects_0_vc707 = ax.bar(x - 0.5*width - offset, latency_strategy_vc707[1], width, label="VC707 " + strategies[1])
rects_1_vc707 = ax.bar(x - 1.5*width - offset, latency_strategy_vc707[0], width, label="VC707 " + strategies[0])

handles = [
    rects_0_vcu118, rects_1_vcu118,
    rects_0_vc707, rects_1_vc707,
]
labels = ["VCU118 "+strat for strat in strategies] + ["VC707 "+strat for strat in strategies[::-1]]

ax.set_ylabel("Latency [cycles]")
ax.set_xticks(x)
ax.set_xticklabels(robots)
ax.legend(handles, labels, loc="upper left", fontsize=9)

fig.tight_layout()

plt.savefig("out/total_latency_constrained_baseline.pdf")
plt.close(fig)

##

fig, ax = plt.subplots(figsize=(5,2))
rects_0_vcu118 = ax.bar(x + 0.5*width + offset, lut_utilization_strategy_vcu118[0], width, label="VCU118 " + strategies[0])
rects_1_vcu118 = ax.bar(x + 1.5*width + offset, lut_utilization_strategy_vcu118[1], width, label="VCU118 " + strategies[1])

rects_0_vc707 = ax.bar(x - 0.5*width - offset, lut_utilization_strategy_vc707[1], width, label="VC707 " + strategies[1])
rects_1_vc707 = ax.bar(x - 1.5*width - offset, lut_utilization_strategy_vc707[0], width, label="VC707 " + strategies[0])

handles = [
    rects_0_vcu118, rects_1_vcu118,
    rects_0_vc707, rects_1_vc707,
]
labels = ["VCU118 "+strat for strat in strategies] + ["VC707 "+strat for strat in strategies[::-1]]

ax.set_ylabel("Resource Utilization [LUTs]")

y_tick_vals = ax.get_yticks()
ax.set_yticklabels(['{x}k'.format(x=int(x//1000)) for x in y_tick_vals])

ax.set_xticks(x)
ax.set_xticklabels(robots)
ax.legend(handles, labels, loc="upper left", fontsize=8)

fig.tight_layout()

plt.savefig("out/lut_utilization_constrained_baseline.pdf")
plt.close(fig)

##

fig, ax = plt.subplots(figsize=(5,2))
rects_0_vcu118 = ax.bar(x + 0.5*width + offset, dsp_utilization_strategy_vcu118[0], width, label="VCU118 " + strategies[0])
rects_1_vcu118 = ax.bar(x + 1.5*width + offset, dsp_utilization_strategy_vcu118[1], width, label="VCU118 " + strategies[1])

rects_0_vc707 = ax.bar(x - 0.5*width - offset, dsp_utilization_strategy_vc707[1], width, label="VC707 " + strategies[1])
rects_1_vc707 = ax.bar(x - 1.5*width - offset, dsp_utilization_strategy_vc707[0], width, label="VC707 " + strategies[0])

handles = [
    rects_0_vcu118, rects_1_vcu118,
    rects_0_vc707, rects_1_vc707,
]
labels = ["VCU118 "+strat for strat in strategies] + ["VC707 "+strat for strat in strategies[::-1]]

ax.set_ylabel("Resource Utilization [DSPs]")

ax.set_xticks(x)
ax.set_xticklabels(robots)
ax.legend(handles, labels, loc="upper left", fontsize=8)

fig.tight_layout()

plt.savefig("out/dsp_utilization_constrained_baseline.pdf")
plt.close(fig)

### dumping data
print(np.array(latency_strategy_vcu118))
print(np.array(latency_strategy_vc707))
print(np.array(lut_utilization_strategy_vcu118))
print(np.array(lut_utilization_strategy_vc707))
