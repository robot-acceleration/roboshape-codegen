#!/usr/bin/python3
import numpy as np
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

results_dir = "results_csv/multi_robot_lowest_latency_VCU118"

def plot_sweep_line_plot(sweep_dir, plot_filename):
    line_plot_csv = os.path.join(sweep_dir, "line_plot_data.csv")

    if "chain" in plot_filename:
        figsize = (5,3)
        lbls = [2,4,6,8,10,12]
    elif "handle" in plot_filename:
        figsize = (4,3)
        lbls = [2,4,6]
    elif "tine" in plot_filename:
        figsize = (3,3)
        lbls = [1,2,3,4]
    elif "quadruped_length" in plot_filename:
        figsize = (5,3)
        lbls = [2,4,6,8,10]
    elif "quadruped_manip" in plot_filename:
        figsize = (5,3)
        lbls = [2,4,6,8,10]
    elif "torso" in plot_filename:
        figsize = (5,3)
        lbls = [2,4,6,8,10]

    fig, axs = plt.subplots(nrows=2, gridspec_kw={'height_ratios': [1,2]}, sharex="all", figsize=figsize)

    fields = []
    line_plot_data = {}

    with open(line_plot_csv) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=",")
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                for field in row:
                    fields.append(field)
                    line_plot_data[field] = []
            else:
                for i,field in enumerate(fields):
                    line_plot_data[field].append(int(row[i]))
            line_count += 1

    swept_values = line_plot_data["swept_value"]
    single_compute_time_list = line_plot_data["single_compute_time_cycles"]
    num_fproc_PEs_list = line_plot_data["num_fproc_PEs"]
    num_bproc_PEs_list = line_plot_data["num_bproc_PEs"]
    block_size_list = line_plot_data["block_size"]

    single_compute_line, = axs[0].plot(swept_values, single_compute_time_list, "go-", markersize=4, label="Total latency")
    #single_compute_bar = axs[0].bar(swept_values, single_compute_time_list, 0.35, color="g", label="Total latency")
    fproc_line, = axs[1].plot(swept_values, num_fproc_PEs_list, "ro-", markersize=10, label="# fproc PEs")
    bproc_line, = axs[1].plot(swept_values, num_bproc_PEs_list, "yo-", markersize=7, label="# bproc PEs")
    block_size_line, = axs[1].plot(swept_values, block_size_list, "bo-", markersize=4, label="Block size")

    axs[1].set_xlim(left=0.5)
    plt.xticks(lbls, [str(l) for l in lbls])

    plt.legend(handles=[single_compute_line, fproc_line, bproc_line, block_size_line], loc="upper left", fontsize=8)

    plot_filepath = os.path.join(sweep_dir, plot_filename)
    plt.savefig(plot_filepath)

sweep_dirs = [
        "chain_length_sweep_csv",
        "quadruped_length_sweep_csv",
        "torso_length_sweep_csv",
        "handle_length_sweep_csv",
        "tine_length_sweep_csv",
        "quadruped_manip_length_sweep_csv",
]
plot_filenames = [
        "chain_length_sweep",
        "quadruped_length_sweep",
        "torso_length_sweep",
        "handle_length_sweep",
        "tine_length_sweep",
        "quadruped_manip_length_sweep",
]

for i,sweep_dir in enumerate(sweep_dirs):
    sweep_dir_path = os.path.join(results_dir, sweep_dir)
    plot_sweep_line_plot(sweep_dir_path, plot_filenames[i]+".pdf")
