#!/usr/bin/python3
import numpy as np
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration

collect_data_or_plot = "plot" # options: collect, plot

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
pareto_curve_filenames = [
    "out/iiwa_pareto_curve",
    "out/hyq_pareto_curve",
    "out/baxter_pareto_curve",
    "out/jaco2_pareto_curve",
    "out/jaco3_pareto_curve",
    "out/hyq_manip_pareto_curve",
]
robots = [
    "iiwa",
    "HyQ",
    "Baxter",
    "Jaco-2",
    "Jaco-3",
    "HyQ+arm"
]

if collect_data_or_plot == "collect":
    for i,urdf_file in enumerate(urdf_files):
        print(urdf_file)
        parser = URDFParser()
        robot = parser.parse(urdf_file)
        fpga_codegen = FPGACodegen(robot)

        dse = DesignSpaceExploration(robot, -1, -1)
        design_point_latencies, design_point_lut_util, design_point_dsp_util, design_point_num_fproc_PEs, design_point_num_bproc_PEs, design_point_block_size, is_efficient = dse.get_pareto_curve_dse(intermediate_muxing=False)
        with open(collected_data_filenames[i]+".csv", "w") as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(design_point_latencies.tolist())
            csvwriter.writerow(design_point_lut_util.tolist())
            csvwriter.writerow(design_point_dsp_util.tolist())
            csvwriter.writerow(design_point_num_fproc_PEs)
            csvwriter.writerow(design_point_num_bproc_PEs)
            csvwriter.writerow(design_point_block_size)
            csvwriter.writerow(is_efficient.tolist())
else:
    gridfig, axs = plt.subplots(2, 3, figsize=(6,2.5))
    axs = axs.flatten()

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

        #parser = URDFParser()
        #robot = parser.parse(urdf_files[i])
        #fpga_codegen = FPGACodegen(robot)

        #sched_occupancy_pareto = []

        ## schedule occupancy for pareto optimal points
        #for pareto_idx in np.nditer(np.argwhere(is_efficient)):
        #    num_fproc_PEs = design_point_num_fproc_PEs[pareto_idx]
        #    num_bproc_PEs = design_point_num_bproc_PEs[pareto_idx]
        #    block_size = design_point_block_size[pareto_idx]

        #    schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, block_size)

        #    fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
        #    bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
        #    matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(num_bproc_PEs)
        #    
        #    len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
        #    len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]

        #    fproc_curr_sched = fproc_sched_table_dict["fproc_curr_sched"]
        #    bproc_curr_sched = bproc_sched_table_dict["bproc_curr_sched"]
        #    
        #    n_nonzeros_fproc_sched = np.count_nonzero(np.array(fproc_curr_sched))
        #    n_nonzeros_bproc_sched = np.count_nonzero(np.array(bproc_curr_sched))

        #    total_sched_ops = (len_fproc_sched*num_fproc_PEs)*3 + (len_bproc_sched*num_bproc_PEs)
        #    total_nonzero_ops = n_nonzeros_fproc_sched*3 + n_nonzeros_bproc_sched

        #    sched_occupancy = total_nonzero_ops / total_sched_ops
        #    sched_occupancy_pareto.append(sched_occupancy)


        ### plotting

        ## grid of pareto curves

        print(design_point_latencies.max())
        print(design_point_lut_util.max())

        design_point_latencies_norm = np.divide(design_point_latencies, design_point_latencies.max())
        design_point_lut_util_norm = np.divide(design_point_lut_util, design_point_lut_util.max())
        
        axs[i].plot(design_point_latencies_norm[is_efficient==False], design_point_lut_util_norm[is_efficient==False], "bo", markersize=0.5)
        axs[i].plot(design_point_latencies_norm[is_efficient==True], design_point_lut_util_norm[is_efficient==True], "rx-", markersize=7)

        axs[i].text(0.5, 0.9, robots[i], horizontalalignment='center', verticalalignment='center', transform=axs[i].transAxes, fontsize=11)

        axs[i].get_xaxis().set_ticks([])
        axs[i].get_yaxis().set_ticks([])

        ## plot design space and pareto curve

        #fig, ax = plt.subplots(figsize=(3,3))

        #ax.plot(design_point_latencies[is_efficient==False], design_point_lut_util[is_efficient==False], "bo", markersize=0.5)
        #ax.plot(design_point_latencies[is_efficient==True], design_point_lut_util[is_efficient==True], "rx-")

        #ax.set_xlabel("Latency [cycles]")
        #ax.set_ylabel("Resource Utilization [LUTs]")

        #y_tick_vals = ax.get_yticks()
        #ax.set_yticklabels(['{x}k'.format(x=int(x//1000)) for x in y_tick_vals])

        #fig.tight_layout()
        #plt.savefig(pareto_curve_filenames[i]+".pdf")
        #plt.close(fig)

        ## plot sched occupancy

        #fig, ax = plt.subplots(figsize=(3,3))

        #ax.plot(design_point_latencies[is_efficient==True], sched_occupancy_pareto, "rx-")

        #ax.set_xlabel("Latency [cycles]")
        #ax.set_ylabel("Normalized schedule occupancy")

        #fig.tight_layout()
        #plt.savefig(pareto_curve_filenames[i]+"_sched_occ"+".pdf")
        #plt.close(fig)

        ## plot fproc, bproc, matmul

        #fig, ax = plt.subplots(figsize=(4,4))
        #fproc_line, = ax.plot(design_point_latencies[is_efficient==True], design_point_num_fproc_PEs[is_efficient==True], "ro-", markersize=8, label="num_fproc_PEs")
        #bproc_line, = ax.plot(design_point_latencies[is_efficient==True], design_point_num_bproc_PEs[is_efficient==True], "yo-", markersize=6, label="num_bproc_PEs")
        #matmul_line, = ax.plot(design_point_latencies[is_efficient==True], design_point_block_size[is_efficient==True], "bo-", markersize=4, label="block_size")

        #plt.legend(handles=[fproc_line, bproc_line, matmul_line])

        #ax.set_xlabel("Latency [cycles]")
        #ax.set_ylabel("Resource count")

        #fig.tight_layout()
        #plt.savefig(pareto_curve_filenames[i]+"_resource_count"+".pdf")
        #plt.close(fig)

    plt.figtext(0.5, 0.08, "x-axis → Latency [cycles]", horizontalalignment='center', verticalalignment='center', fontsize=10)
    plt.figtext(0.1, 0.5, "y-axis → Resource util [LUTs]", rotation=90, horizontalalignment='center', verticalalignment='center', fontsize=9)

    gridfig.subplots_adjust(wspace=0.03, hspace=0.03)
    #gridfig.tight_layout()
    plt.savefig("out/pareto_grid.png")
    plt.close(gridfig)
