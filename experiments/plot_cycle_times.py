#!/usr/bin/python3
import numpy as np
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

from URDFGenerator import URDFGenerator
from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration

#results_dir = "results_csv/multi_robot_lowest_latency_ASIC"
results_dir = "results_csv/multi_robot_lowest_latency_VCU118"

do_chain_length_sweep = True
do_handle_length_sweep = True
do_torso_length_sweep = True
do_quadruped_length_sweep = True
do_quadruped_manip_length_sweep = True
do_tine_length_sweep = True

# for unbounded ASIC numbers, set both to -1
max_DSE_PEs = 7
max_block_size = 10

def plot_num_PEs(single_compute_time_list, num_fproc_PEs_list, num_bproc_PEs_list, block_size_list, swept_values, plot_filename, csvdir):
    fig = plt.figure(figsize=(3,4))

    xlim = max(num_fproc_PEs_list+num_bproc_PEs_list+block_size_list)+1
    ylim = int(ceil(max(single_compute_time_list) / 50) * 50)

    plt.xlim([0, xlim])
    plt.ylim([0, ylim])

    plt.xticks(range(1, 11, 3), fontsize=9)
    plt.yticks(fontsize=8)

    fproc_line, = plt.plot(num_fproc_PEs_list, single_compute_time_list, "ro-", markersize=10, label="# fproc PEs")
    bproc_line, = plt.plot(num_bproc_PEs_list, single_compute_time_list, "yo-", markersize=7, label="# bproc PEs")
    block_size_line, = plt.plot(block_size_list, single_compute_time_list, "bo-", markersize=4, label="Block size")

    plt.legend(handles=[fproc_line, bproc_line, block_size_line], loc="upper left", fontsize=8)

    point_labels = ["A","B","C","D","E","F","G","H","I","J","K","L"]

    for pt_idx in range(len(swept_values)):
        plt.annotate(
                #point_labels[pt_idx], 
                str(swept_values[pt_idx]),
                (num_fproc_PEs_list[pt_idx]-0.7, single_compute_time_list[pt_idx]+1),
                fontsize=9
        )
        if num_bproc_PEs_list[pt_idx] != num_fproc_PEs_list[pt_idx]:
            plt.annotate(
                    #point_labels[pt_idx], 
                    str(swept_values[pt_idx]),
                    (num_bproc_PEs_list[pt_idx]-0.6, single_compute_time_list[pt_idx]+1),
                    fontsize=9
            )
        if block_size_list[pt_idx] != num_fproc_PEs_list[pt_idx] and block_size_list[pt_idx] != num_bproc_PEs_list[pt_idx]:
            plt.annotate(
                    #point_labels[pt_idx], 
                    str(swept_values[pt_idx]),
                    (block_size_list[pt_idx]-0.5, single_compute_time_list[pt_idx]+1),
                    fontsize=9
            )

    plot_filepath = os.path.join(csvdir, plot_filename)
    plt.savefig(plot_filepath)
    
    # csv writing
    excel_plot_zip = zip(swept_values, num_fproc_PEs_list, num_bproc_PEs_list, block_size_list, single_compute_time_list)
    csv_path = os.path.join(csvdir, "line_plot_data.csv")
    with open(csv_path, "w", newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)
        csvwriter.writerow(["swept_value", "num_fproc_PEs", "num_bproc_PEs", "block_size", "single_compute_time_cycles"])
        for data_pt in excel_plot_zip:
            csvwriter.writerow(data_pt)


def get_single_compute_cycle_time(urdf_file, csvdir, sweep_param):
    parser = URDFParser()
    robot = parser.parse(urdf_file)
    fpga_codegen = FPGACodegen(robot)

    dse = DesignSpaceExploration(robot, max_DSE_PEs, max_block_size)

    num_fproc_PEs = dse.find_best_num_fproc_PEs()
    num_bproc_PEs = dse.find_best_num_bproc_PEs()
    block_size = dse.find_best_block_size(num_bproc_PEs)

    schedule_gen = fpga_codegen.get_schedule_gen(num_fproc_PEs, num_bproc_PEs, block_size)

    fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
    len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
    bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
    len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
    matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(num_bproc_PEs)
    len_matmul_sched = matmul_sched_table_dict["len_block_minv_sched_per_matrix"]

    print(urdf_file)

    print("len_fproc_sched: "+str(len_fproc_sched)) 
    schedule_gen.print_fmted_schedules(fproc_sched_table_dict)
    print("len_bproc_sched: "+str(len_bproc_sched))
    schedule_gen.print_fmted_schedules(bproc_sched_table_dict)

    sweep_param_csvdir = os.path.join(csvdir, str(sweep_param))
    schedule_gen.write_schedule_table_csv(fproc_sched_table_dict, sweep_param_csvdir)
    schedule_gen.write_schedule_table_csv(bproc_sched_table_dict, sweep_param_csvdir)
    schedule_gen.write_matmul_schedule_table_csv(matmul_sched_table_dict, sweep_param_csvdir)

    print("len_matmul_sched, block_size: "+str(len_matmul_sched)+", "+str(block_size))

    return num_fproc_PEs, num_bproc_PEs, block_size, (len_fproc_sched*3 + len_bproc_sched + len_matmul_sched*2)

if do_chain_length_sweep:
    # chain 1-12 links
    chain_length_sweep = list(range(1,13))
    chain_length_sweep_cycle_time_list = []
    chain_length_sweep_num_fproc_PEs_list = []
    chain_length_sweep_num_bproc_PEs_list = []
    chain_length_sweep_block_size_list = []

    chain_length_urdf_dir = "chain_length_sweep"
    chain_length_csvdir = os.path.join(results_dir, "chain_length_sweep_csv")
    for chain_length in chain_length_sweep:
        urdf_file = chain_length_urdf_dir + "/" + "chain_" + str(chain_length) + ".urdf"
        num_fproc_PEs, num_bproc_PEs, block_size, single_compute_time = get_single_compute_cycle_time(urdf_file, chain_length_csvdir, chain_length)

        chain_length_sweep_num_fproc_PEs_list.append(num_fproc_PEs)
        chain_length_sweep_num_bproc_PEs_list.append(num_bproc_PEs)
        chain_length_sweep_block_size_list.append(block_size)
        chain_length_sweep_cycle_time_list.append(single_compute_time)

    plot_num_PEs(
            chain_length_sweep_cycle_time_list,
            chain_length_sweep_num_fproc_PEs_list,
            chain_length_sweep_num_bproc_PEs_list,
            chain_length_sweep_block_size_list,
            chain_length_sweep,
            "chain_length_sweep.pdf",
            chain_length_csvdir
    )

if do_handle_length_sweep:
    # fork handle 1-6 links, tine length 3
    handle_length_sweep = list(range(1,7))
    handle_length_sweep_cycle_time_list = []
    handle_length_sweep_num_fproc_PEs_list = []
    handle_length_sweep_num_bproc_PEs_list = []
    handle_length_sweep_block_size_list = []

    handle_length_urdf_dir = "handle_length_sweep"
    handle_length_csvdir = os.path.join(results_dir, "handle_length_sweep_csv")
    for handle_length in handle_length_sweep:
        urdf_file = handle_length_urdf_dir + "/" + "fork_" + str(handle_length+6) + "_" + str(handle_length) + "_3_3" + ".urdf"
        num_fproc_PEs, num_bproc_PEs, block_size, single_compute_time = get_single_compute_cycle_time(urdf_file, handle_length_csvdir, handle_length)

        handle_length_sweep_num_fproc_PEs_list.append(num_fproc_PEs)
        handle_length_sweep_num_bproc_PEs_list.append(num_bproc_PEs)
        handle_length_sweep_block_size_list.append(block_size)
        handle_length_sweep_cycle_time_list.append(single_compute_time)

    plot_num_PEs(
            handle_length_sweep_cycle_time_list,
            handle_length_sweep_num_fproc_PEs_list,
            handle_length_sweep_num_bproc_PEs_list,
            handle_length_sweep_block_size_list,
            handle_length_sweep,
            "handle_length_sweep.pdf",
            handle_length_csvdir
    )

if do_torso_length_sweep:
    # torso limb length 1-10 links
    torso_length_sweep = list(range(1,11))
    torso_length_sweep_cycle_time_list = []
    torso_length_sweep_num_fproc_PEs_list = []
    torso_length_sweep_num_bproc_PEs_list = []
    torso_length_sweep_block_size_list = []

    torso_length_urdf_dir = "torso_length_sweep"
    torso_length_csvdir = os.path.join(results_dir, "torso_length_sweep_csv")
    for torso_length in torso_length_sweep:
        urdf_file = torso_length_urdf_dir + "/" + "torso_" + str(1+torso_length*2) + "_1_{tl}_{tl}.urdf".format(tl=torso_length)
        num_fproc_PEs, num_bproc_PEs, block_size, single_compute_time = get_single_compute_cycle_time(urdf_file, torso_length_csvdir, torso_length)

        torso_length_sweep_num_fproc_PEs_list.append(num_fproc_PEs)
        torso_length_sweep_num_bproc_PEs_list.append(num_bproc_PEs)
        torso_length_sweep_block_size_list.append(block_size)
        torso_length_sweep_cycle_time_list.append(single_compute_time)

    plot_num_PEs(
            torso_length_sweep_cycle_time_list,
            torso_length_sweep_num_fproc_PEs_list,
            torso_length_sweep_num_bproc_PEs_list,
            torso_length_sweep_block_size_list,
            torso_length_sweep,
            "torso_length_sweep.pdf",
            torso_length_csvdir
    )

if do_quadruped_length_sweep:
    # quadruped limb length 1-10 links
    quadruped_length_sweep = list(range(1,11))
    quadruped_length_sweep_cycle_time_list = []
    quadruped_length_sweep_num_fproc_PEs_list = []
    quadruped_length_sweep_num_bproc_PEs_list = []
    quadruped_length_sweep_block_size_list = []

    quadruped_length_urdf_dir = "quadruped_length_sweep"
    quadruped_length_csvdir = os.path.join(results_dir, "quadruped_length_sweep_csv")
    for quadruped_length in quadruped_length_sweep:
        urdf_file = quadruped_length_urdf_dir + "/" + "quadruped_" + str(quadruped_length*4) + "_{ql}_{ql}_{ql}_{ql}.urdf".format(ql=quadruped_length)
        num_fproc_PEs, num_bproc_PEs, block_size, single_compute_time = get_single_compute_cycle_time(urdf_file, quadruped_length_csvdir, quadruped_length)

        quadruped_length_sweep_num_fproc_PEs_list.append(num_fproc_PEs)
        quadruped_length_sweep_num_bproc_PEs_list.append(num_bproc_PEs)
        quadruped_length_sweep_block_size_list.append(block_size)
        quadruped_length_sweep_cycle_time_list.append(single_compute_time)

    plot_num_PEs(
            quadruped_length_sweep_cycle_time_list,
            quadruped_length_sweep_num_fproc_PEs_list,
            quadruped_length_sweep_num_bproc_PEs_list,
            quadruped_length_sweep_block_size_list,
            quadruped_length_sweep,
            "quadruped_length_sweep.pdf",
            quadruped_length_csvdir
    )

if do_quadruped_manip_length_sweep:
    # quadruped_manip limb length 1-10 links
    quadruped_manip_length_sweep = list(range(1,11))
    quadruped_manip_length_sweep_cycle_time_list = []
    quadruped_manip_length_sweep_num_fproc_PEs_list = []
    quadruped_manip_length_sweep_num_bproc_PEs_list = []
    quadruped_manip_length_sweep_block_size_list = []

    quadruped_manip_length_urdf_dir = "quadruped_manip_length_sweep"
    quadruped_manip_length_csvdir = os.path.join(results_dir, "quadruped_manip_length_sweep_csv")
    for quadruped_manip_length in quadruped_manip_length_sweep:
        urdf_file = quadruped_manip_length_urdf_dir + "/" + "quadruped_manip_" + str(7+quadruped_manip_length*4) + "_7_{ql}_{ql}_{ql}_{ql}.urdf".format(ql=quadruped_manip_length)
        num_fproc_PEs, num_bproc_PEs, block_size, single_compute_time = get_single_compute_cycle_time(urdf_file, quadruped_manip_length_csvdir, quadruped_manip_length)

        quadruped_manip_length_sweep_num_fproc_PEs_list.append(num_fproc_PEs)
        quadruped_manip_length_sweep_num_bproc_PEs_list.append(num_bproc_PEs)
        quadruped_manip_length_sweep_block_size_list.append(block_size)
        quadruped_manip_length_sweep_cycle_time_list.append(single_compute_time)

    plot_num_PEs(
            quadruped_manip_length_sweep_cycle_time_list,
            quadruped_manip_length_sweep_num_fproc_PEs_list,
            quadruped_manip_length_sweep_num_bproc_PEs_list,
            quadruped_manip_length_sweep_block_size_list,
            quadruped_manip_length_sweep,
            "quadruped_manip_length_sweep.pdf",
            quadruped_manip_length_csvdir
    )

if do_tine_length_sweep:
    # tine length 1-5 links
    tine_length_sweep = list(range(1,5))
    tine_length_sweep_cycle_time_list = []
    tine_length_sweep_num_fproc_PEs_list = []
    tine_length_sweep_num_bproc_PEs_list = []
    tine_length_sweep_block_size_list = []

    tine_length_urdf_dir = "tine_length_sweep"
    tine_length_csvdir = os.path.join(results_dir, "tine_length_sweep_csv")
    for tine_length in tine_length_sweep:
        urdf_file = tine_length_urdf_dir + "/" + "arm_3_fingers_" + str(6+tine_length*3) + "_6_{tl}_{tl}_{tl}.urdf".format(tl=tine_length)
        num_fproc_PEs, num_bproc_PEs, block_size, single_compute_time = get_single_compute_cycle_time(urdf_file, tine_length_csvdir, tine_length)

        tine_length_sweep_num_fproc_PEs_list.append(num_fproc_PEs)
        tine_length_sweep_num_bproc_PEs_list.append(num_bproc_PEs)
        tine_length_sweep_block_size_list.append(block_size)
        tine_length_sweep_cycle_time_list.append(single_compute_time)

    plot_num_PEs(
            tine_length_sweep_cycle_time_list,
            tine_length_sweep_num_fproc_PEs_list,
            tine_length_sweep_num_bproc_PEs_list,
            tine_length_sweep_block_size_list,
            tine_length_sweep,
            "tine_length_sweep.pdf",
            tine_length_csvdir
    )
