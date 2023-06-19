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

# original robot_ord: iiwa, hyq, baxter, jaco-2, jaco-3, hyq-manip
# new robot_ord: iiwa, hyq, jaco-2, baxter, jaco-3, hyq-manip

# original strategies: optimal, [strategies]

n_strategies = 6
robot_ord = [0,1,3,2,4,5]
strat_ord = [0,2,1,3,4]

robots = [
        "iiwa",
        "HyQ",
        "Baxter",
        "jaco-2",
        "jaco-3",
        "HyQ+arm",
]
robots = np.array(robots)[robot_ord]

urdf_files = [
    "../../urdfs/iiwa_rbd_accel.urdf",
    "../../urdfs/hyq_simple.urdf",
    "../../urdfs/baxter_simple.urdf",
    "../handle_length_sweep/fork_12_6_3_3.urdf",
    "../tine_length_sweep/arm_3_fingers_15_6_3_3_3.urdf",
    "../quadruped_manip_length_sweep/quadruped_manip_19_7_3_3_3_3.urdf"
]
urdf_files = np.array(urdf_files)[robot_ord]

strategies = [
        "# of links",
        "Longest chain",
        "Avg leaf depth",
        "Max descendents",
        "Heuristic",
]
strategies = np.array(strategies)[strat_ord]

resource_val_strategy = [
        [7,12,15,12,15,19],
        [7, 3, 7, 9, 9, 7],
        [7, 3, 5, 9, 9, 4],
        [7, 3, 7,12,15, 7]
]
resource_val_strategy = np.array(resource_val_strategy)[:, robot_ord]
resource_val_strategy = resource_val_strategy[strat_ord[:-1], :]

latency_strategy = [
        [],
        [],
        [],
        [],
        [],
        [],
]

lut_utilization_strategy = [
        [],
        [],
        [],
        [],
        [],
        [],
]

dsp_utilization_strategy = [
        [],
        [],
        [],
        [],
        [],
        [],
]

total_luts = 1180000
total_dsps = 6840

edp_strategy = [
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

    print("optimal (unconstrained):")

    num_fproc_PEs = dse.find_best_num_fproc_PEs()
    num_bproc_PEs = dse.find_best_num_bproc_PEs()

    print(num_fproc_PEs)
    print(num_bproc_PEs)

    schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, 1)

    fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
    len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
    bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
    len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
    total_len_sched = len_fproc_sched*3 + len_bproc_sched

    print("len_fproc_sched: "+str(len_fproc_sched)) 
    print("len_bproc_sched: "+str(len_bproc_sched))
    print("total: "+str(len_fproc_sched*3+len_bproc_sched))

    lut_util = dse.get_lut_usage_without_mm(num_fproc_PEs, num_bproc_PEs, len_fproc_sched, robot.get_num_joints(), intermediate_muxing=False)
    dsp_util = dse.get_dsp_usage_without_mm(num_fproc_PEs, num_bproc_PEs, robot.get_num_joints())

    latency_strategy[n_strategies-1].append(total_len_sched)
    lut_utilization_strategy[n_strategies-1].append(lut_util)
    dsp_utilization_strategy[n_strategies-1].append(dsp_util)

    edp_strategy[0].append(total_len_sched * (lut_util/total_luts + dsp_util/total_dsps))

    print("----")

    for j,strategy in enumerate(strategies):
        print(strategy)
        if strategy != "Heuristic":
            num_fproc_PEs = num_bproc_PEs = resource_val_strategy[j][i]
        else:
            num_fproc_PEs = resource_val_strategy[2][i] # Longest chain
            num_bproc_PEs = resource_val_strategy[3][i] # Max descendents


        schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, 1)

        fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
        len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
        bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
        len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
        total_len_sched = len_fproc_sched*3 + len_bproc_sched

        print("len_fproc_sched: "+str(len_fproc_sched)) 
        print("len_bproc_sched: "+str(len_bproc_sched))
        print("total: "+str(len_fproc_sched*3+len_bproc_sched))

        lut_util = dse.get_lut_usage_without_mm(num_fproc_PEs, num_bproc_PEs, len_fproc_sched, robot.get_num_joints(), intermediate_muxing=False)
        dsp_util = dse.get_dsp_usage_without_mm(num_fproc_PEs, num_bproc_PEs, robot.get_num_joints())

        latency_strategy[j].append(total_len_sched)
        lut_utilization_strategy[j].append(lut_util)
        dsp_utilization_strategy[j].append(dsp_util)

        edp_strategy[j+1].append(total_len_sched * (lut_util/total_luts + dsp_util/total_dsps))

        print("----")


### plotting

# strategy robot_ord: optimal, [strategies]

strategies = strategies.tolist()

color_indices = np.linspace(0, 1, n_strategies+1)
cmap = mpl.cm.get_cmap("viridis")
colors = [cmap(idx) for idx in color_indices]

# total_latency

x = np.arange(len(robots))
width = 0.1

fig, ax = plt.subplots(figsize=(6,3))
rects_0 = ax.bar(x - 2.5*width, latency_strategy[0], width, color=colors[0], edgecolor="black", linewidth=0.1, label="Optimal DSE baseline")
rects_1 = ax.bar(x - 1.5*width, latency_strategy[1], width, color=colors[1], edgecolor="black", linewidth=0.1, label=strategies[0])
rects_2 = ax.bar(x - 0.5*width, latency_strategy[2], width, color=colors[2], edgecolor="black", linewidth=0.1, label=strategies[1])
rects_3 = ax.bar(x + 0.5*width, latency_strategy[3], width, color=colors[3], edgecolor="black", linewidth=0.1, label=strategies[2])
rects_4 = ax.bar(x + 1.5*width, latency_strategy[4], width, color=colors[4], edgecolor="black", linewidth=0.1, label=strategies[3])
rects_5 = ax.bar(x + 2.5*width, latency_strategy[5], width, color=colors[5], edgecolor="black", linewidth=0.1, label=strategies[4])

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4, rects_5
]
labels = ["Optimal DSE baseline"] + strategies

ax.set_ylabel("Latency [cycles]")
ax.set_xticks(x)
ax.set_xticklabels(robots)
#ax.legend(handles, labels, ncol=2, loc="upper left", fontsize=9)

fig.tight_layout()

plt.savefig("out/total_latency_unconstrained_baselines.pdf")
plt.close(fig)


##

vcu118_luts = 1182240
vc707_luts = 303600

#  lut util

fig, ax = plt.subplots(figsize=(6,3))
rects_0 = ax.bar(x - 2.5*width, lut_utilization_strategy[0], width, color=colors[0], edgecolor="black", linewidth=0.1, label="Optimal DSE baseline")
rects_1 = ax.bar(x - 1.5*width, lut_utilization_strategy[1], width, color=colors[1], edgecolor="black", linewidth=0.1, label=strategies[0])
rects_2 = ax.bar(x - 0.5*width, lut_utilization_strategy[2], width, color=colors[2], edgecolor="black", linewidth=0.1, label=strategies[1])
rects_3 = ax.bar(x + 0.5*width, lut_utilization_strategy[3], width, color=colors[3], edgecolor="black", linewidth=0.1, label=strategies[2])
rects_4 = ax.bar(x + 1.5*width, lut_utilization_strategy[4], width, color=colors[4], edgecolor="black", linewidth=0.1, label=strategies[3])
rects_5 = ax.bar(x + 2.5*width, lut_utilization_strategy[5], width, color=colors[5], edgecolor="black", linewidth=0.1, label=strategies[4])

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4, rects_5
]
labels = ["Optimal DSE baseline"] + strategies

ax.set_ylabel("LUTs")
y_tick_vals = ax.get_yticks()
ax.set_yticklabels(['{x}k'.format(x=int(x//1000)) for x in y_tick_vals])

ax.text(-0.5, vcu118_luts + 100000, "VCU118", fontsize=10)
ax.annotate("VC707", (-0.5, vc707_luts), xytext=(-0.5, vc707_luts + 400000), arrowprops={"arrowstyle":"simple"})

ax.axhline(vcu118_luts, color="black", linestyle="dashed")
ax.axhline(vc707_luts, color="black", linestyle="dashed")

ax.set_xticks(x)
ax.set_xticklabels(robots)
#ax.legend(handles, labels, ncol=2, loc="upper left", fontsize=9)

fig.tight_layout()

plt.savefig("out/lut_utilization_unconstrained_baselines.pdf")
plt.close(fig)

##

#  dsp util

vcu118_dsps = 6840
vc707_dsps = 2800

fig, ax = plt.subplots(figsize=(6,3))
rects_0 = ax.bar(x - 2.5*width, dsp_utilization_strategy[0], width, color=colors[0], edgecolor="black", linewidth=0.1, label="Optimal DSE baseline")
rects_1 = ax.bar(x - 1.5*width, dsp_utilization_strategy[1], width, color=colors[1], edgecolor="black", linewidth=0.1, label=strategies[0])
rects_2 = ax.bar(x - 0.5*width, dsp_utilization_strategy[2], width, color=colors[2], edgecolor="black", linewidth=0.1, label=strategies[1])
rects_3 = ax.bar(x + 0.5*width, dsp_utilization_strategy[3], width, color=colors[3], edgecolor="black", linewidth=0.1, label=strategies[2])
rects_4 = ax.bar(x + 1.5*width, dsp_utilization_strategy[4], width, color=colors[4], edgecolor="black", linewidth=0.1, label=strategies[3])
rects_5 = ax.bar(x + 2.5*width, dsp_utilization_strategy[5], width, color=colors[5], edgecolor="black", linewidth=0.1, label=strategies[4])

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4, rects_5
]
labels = ["Optimal DSE baseline"] + strategies

ax.text(-0.5, vcu118_dsps + 500, "VCU118", fontsize=10)
ax.annotate("VC707", (-0.5, vc707_dsps), xytext=(-0.58, 5700), arrowprops={"arrowstyle":"simple"})

ax.axhline(vcu118_dsps, color="black", linestyle="dashed")
ax.axhline(vc707_dsps, color="black", linestyle="dashed")

ax.set_ylabel("DSPs")
ax.set_xticks(x)
ax.set_xticklabels(robots)
#ax.legend(handles, labels, ncol=2, loc="upper left", fontsize=9)

fig.tight_layout()

plt.savefig("out/dsp_utilization_unconstrained_baselines.pdf")
plt.close(fig)

#  EDP

fig, ax = plt.subplots(figsize=(6,3))
rects_0 = ax.bar(x - 2.5*width, edp_strategy[0], width, color=colors[0], edgecolor="black", linewidth=0.1, label="Optimal DSE baseline")
rects_1 = ax.bar(x - 1.5*width, edp_strategy[1], width, color=colors[1], edgecolor="black", linewidth=0.1, label=strategies[0])
rects_2 = ax.bar(x - 0.5*width, edp_strategy[2], width, color=colors[2], edgecolor="black", linewidth=0.1, label=strategies[1])
rects_3 = ax.bar(x + 0.5*width, edp_strategy[3], width, color=colors[3], edgecolor="black", linewidth=0.1, label=strategies[2])
rects_4 = ax.bar(x + 1.5*width, edp_strategy[4], width, color=colors[4], edgecolor="black", linewidth=0.1, label=strategies[3])
rects_5 = ax.bar(x + 2.5*width, edp_strategy[5], width, color=colors[5], edgecolor="black", linewidth=0.1, label=strategies[4])

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4, rects_5
]
labels = ["Optimal DSE baseline"] + strategies

ax.set_ylabel("EDP")
ax.set_xticks(x)
ax.set_xticklabels(robots)
#ax.legend(handles, labels, ncol=2, loc="upper left", fontsize=9)

fig.tight_layout()

plt.savefig("out/edp_unconstrained_baselines.pdf")
plt.close(fig)

### dumping data

print(np.array(latency_strategy).T)
print(np.array(lut_utilization_strategy).T)
print(np.array(dsp_utilization_strategy).T)
print(np.array(edp_strategy).T)

##

## lut+dsp util
#
#vcu118_luts = 1182240
#vc707_luts = 303600
#
#vcu118_dsps = 6840
#vc707_dsps = 2800
#
#comb_width = 0.07
#
#fig, ax1 = plt.subplots(figsize=(10,2))
#ax2 = ax1.twinx()
#
#rects_0_lut = ax1.bar(x - 5.5*comb_width, lut_utilization_strategy[0], comb_width, label="Optimal DSE baseline")
#rects_1_lut = ax1.bar(x - 3.5*comb_width, lut_utilization_strategy[1], comb_width, label=strategies[0])
#rects_2_lut = ax1.bar(x - 1.5*comb_width, lut_utilization_strategy[2], comb_width, label=strategies[1])
#rects_3_lut = ax1.bar(x + 0.5*comb_width, lut_utilization_strategy[3], comb_width, label=strategies[2])
#rects_4_lut = ax1.bar(x + 2.5*comb_width, lut_utilization_strategy[4], comb_width, label=strategies[3])
#rects_5_lut = ax1.bar(x + 4.5*comb_width, lut_utilization_strategy[5], comb_width, label=strategies[4])
#
#rects_0_dsp = ax2.bar(x - 4.5*comb_width, dsp_utilization_strategy[0], comb_width, label="Optimal DSE baseline", hatch="//", edgecolor="black")
#rects_1_dsp = ax2.bar(x - 2.5*comb_width, dsp_utilization_strategy[1], comb_width, label=strategies[0], hatch="//", edgecolor="black")
#rects_2_dsp = ax2.bar(x - 0.5*comb_width, dsp_utilization_strategy[2], comb_width, label=strategies[1], hatch="//", edgecolor="black")
#rects_3_dsp = ax2.bar(x + 1.5*comb_width, dsp_utilization_strategy[3], comb_width, label=strategies[2], hatch="//", edgecolor="black")
#rects_4_dsp = ax2.bar(x + 3.5*comb_width, dsp_utilization_strategy[4], comb_width, label=strategies[3], hatch="//", edgecolor="black")
#rects_5_dsp = ax2.bar(x + 5.5*comb_width, dsp_utilization_strategy[5], comb_width, label=strategies[4], hatch="//", edgecolor="black")
#
## luts
#ax1.axhline(vcu118_luts, 0, 0.95, color="tab:purple", linestyle="dashed")
#ax1.axhline(vc707_luts, 0, 0.95, color="tab:orange", linestyle="dashed")
## dsps
#ax2.axhline(vcu118_dsps, 0.05, 1, color="tab:purple", linestyle="dotted")
#ax2.axhline(vc707_dsps, 0.05, 1, color="tab:orange", linestyle="dotted")
#
#handles = [
#    rects_0_lut, rects_1_lut, rects_2_lut, rects_3_lut, rects_4_lut, rects_5_lut
#]
#labels = ["Optimal DSE baseline"] + strategies
#
#ax1.set_ylabel("LUTs")
#y_tick_vals = ax1.get_yticks()
#ax1.set_yticklabels(['{x}k'.format(x=int(x//1000)) for x in y_tick_vals])
#
#ax2.set_ylabel("DSPs")
#
#ax1.set_xticks(x)
#ax1.set_xticklabels(robots)
#ax1.legend(handles, labels, ncol=2, loc="upper left", fontsize=9)
#
#fig.tight_layout()
#
#plt.savefig("out/utilization_unconstrained_baselines.pdf")
#plt.close(fig)

##

