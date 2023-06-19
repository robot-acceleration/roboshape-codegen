from FPGACodegen import FPGACodegen
import numpy as np
import matplotlib.pyplot as plt
from math import ceil
import os
import csv

class DesignSpaceExploration:
    def __init__(self, robot, max_DSE_PEs, max_block_size):
        self.robot = robot
        self.fpga_codegen = FPGACodegen(robot)

        if max_DSE_PEs == -1:
            self.max_PEs = self.robot.get_num_joints()
        else:
            self.max_PEs = max_DSE_PEs
        if max_block_size == -1:
            self.max_block_size = self.robot.get_num_joints()
        else:
            self.max_block_size = max_block_size

    def find_best_num_fproc_PEs(self):
        shortest_sched_len = 10000
        shortest_sched_len_PEs = 10000

        # we can only fit up to 7 PEs
        for num_fproc_PEs in range(1,self.max_PEs+1):
            # block size doesn't matter for this so set to 1
            schedule_gen = self.fpga_codegen.get_schedule_gen(num_fproc_PEs, 1, 1)

            fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
            len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
            if len_fproc_sched < shortest_sched_len:
                shortest_sched_len = len_fproc_sched
                shortest_sched_len_PEs = num_fproc_PEs

        return shortest_sched_len_PEs

    def find_best_num_bproc_PEs(self):
        shortest_sched_len = 10000
        shortest_sched_len_PEs = 10000

        # we can only fit up to 7 PEs
        for num_bproc_PEs in range(1,self.max_PEs+1):
            # block size doesn't matter for this so set to 1
            schedule_gen = self.fpga_codegen.get_schedule_gen(1, num_bproc_PEs, 1)

            bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
            len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
            if len_bproc_sched < shortest_sched_len:
                shortest_sched_len = len_bproc_sched
                shortest_sched_len_PEs = num_bproc_PEs

        return shortest_sched_len_PEs

    def find_best_block_size(self, num_bproc_PEs):
        # shortest matmul sched
        shortest_matmul_len = 10000
        best_block_size = -1
        for block_size in range(1, self.max_block_size+1):
            schedule_gen = self.fpga_codegen.get_schedule_gen(1, num_bproc_PEs, block_size)
            matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(num_bproc_PEs)
            len_matmul_sched = matmul_sched_table_dict["len_block_minv_sched_per_matrix"]
            if len_matmul_sched < shortest_matmul_len:
                best_block_size = block_size
                shortest_matmul_len = len_matmul_sched
        return best_block_size

    def plot_full_dse(self, plot_filename, csvdir_path):
        len_fproc_scheds = []
        # we can only fit up to 7 PEs
        for num_fproc_PEs in range(1,self.max_PEs+1):
            # block size doesn't matter for this so set to 1
            schedule_gen = self.fpga_codegen.get_schedule_gen(num_fproc_PEs, 1, 1)

            fproc_sched_table_dict = schedule_gen.get_fproc_schedule_tables()
            len_fproc_sched = fproc_sched_table_dict["len_fproc_sched"]
            len_fproc_scheds.append(len_fproc_sched*3)

            print("num_fproc_PEs: "+str(num_fproc_PEs))
            print("len_fproc_sched: "+str(len_fproc_sched)) 
            schedule_gen.print_fmted_schedules(fproc_sched_table_dict)

            PE_csvdir = os.path.join(csvdir_path, str(num_fproc_PEs))
            schedule_gen.write_schedule_table_csv(fproc_sched_table_dict, PE_csvdir)

        len_bproc_scheds = []
        # we can only fit up to 7 PEs
        for num_bproc_PEs in range(1,self.max_PEs+1):
            # block size doesn't matter for this so set to 1
            schedule_gen = self.fpga_codegen.get_schedule_gen(1, num_bproc_PEs, 1)

            bproc_sched_table_dict = schedule_gen.get_bproc_schedule_tables()
            len_bproc_sched = bproc_sched_table_dict["len_bproc_sched"]
            len_bproc_scheds.append(len_bproc_sched)

            print("num_bproc_PEs: "+str(num_bproc_PEs))
            print("len_bproc_sched: "+str(len_bproc_sched)) 
            schedule_gen.print_fmted_schedules(bproc_sched_table_dict)

            PE_csvdir = os.path.join(csvdir_path, str(num_bproc_PEs))
            schedule_gen.write_schedule_table_csv(bproc_sched_table_dict, PE_csvdir)

        # using best num_bproc_PEs for matmul
        matmul_sched_lens = []
        best_num_bproc_PEs = self.find_best_num_bproc_PEs()
        for block_size in range(1, self.max_block_size+1):
            schedule_gen = self.fpga_codegen.get_schedule_gen(1, best_num_bproc_PEs, block_size)
            matmul_sched_table_dict = schedule_gen.get_matmul_schedule_tables(best_num_bproc_PEs)
            len_matmul_sched = matmul_sched_table_dict["len_block_minv_sched_per_matrix"]
            matmul_sched_lens.append(len_matmul_sched*2)

            print("len_matmul_sched, block_size: "+str(len_matmul_sched)+", "+str(block_size))

            if len_matmul_sched <= 30:
                schedule_gen.print_matmul_schedules(matmul_sched_table_dict)

            resource_csvdir = os.path.join(csvdir_path, str(block_size))
            schedule_gen.write_matmul_schedule_table_csv(matmul_sched_table_dict, resource_csvdir)

        ### plotting

        fig = plt.figure(figsize=(6,5))

        #ylim = int(ceil(max(len_fproc_scheds+len_bproc_scheds+matmul_sched_lens) / 50) * 50)
        if self.robot.get_num_joints() >= 15:
            ylim = 200
        else:
            ylim = 100

        plt.xlim([0, max(self.max_PEs, self.max_block_size)+1])
        plt.ylim([0, ylim])

        fproc_line, = plt.plot(list(range(1,self.max_PEs+1)), len_fproc_scheds, "ro-", markersize=15, label="# of cycles fproc")
        bproc_line, = plt.plot(list(range(1,self.max_PEs+1)), len_bproc_scheds, "yo-", markersize=10, label="# of cycles bproc")
        matmul_line, = plt.plot(list(range(1,self.max_block_size+1)), matmul_sched_lens, "bo-", markersize=5, label="# of cycles matmul")

        plt.legend(handles=[fproc_line, bproc_line, matmul_line])

        plot_filepath = os.path.join(csvdir_path, plot_filename)
        plt.savefig(plot_filepath)

        # csv writing, radhika: got lazy, zip will break if max_num_PEs != max_block_size
        excel_plot_zip = zip(list(range(1, self.max_PEs+1)), len_fproc_scheds, len_bproc_scheds, matmul_sched_lens)
        csv_path = os.path.join(csvdir_path, "line_plot_data.csv")
        with open(csv_path, "w", newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow(["compute_resource_alloc", "num_cycles_fproc", "num_cycles_bproc", "num_cycles_matmul"])
            for data_pt in excel_plot_zip:
                csvwriter.writerow(data_pt)
