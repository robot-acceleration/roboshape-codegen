from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import BluespecWriter, FileManager
from rbd_config import dim_list, urdf_file, num_PEs, block_size
from DataStructSizes import *
from math import log2, floor

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()

#-------- File management -------------------

fm = FileManager("GradientPipelineBprocDtauUpdateUnrolling.bsv")
output_file_path = fm.get_output_file_path()
gp_file = open(output_file_path, "w")

bw = BluespecWriter(gp_file, dim_list, num_links)

#---------------------------------------------

bluespec_interface = fpga_codegen.get_bluespec_interface(num_PEs, block_size)
sched_table_dict = bluespec_interface.get_bproc_schedule_tables()

len_bproc_sched = sched_table_dict["len_bproc_sched"]
rnea_bproc_curr_sched = sched_table_dict["rnea_bproc_curr_sched"]
rnea_bproc_par_sched = sched_table_dict["rnea_bproc_par_sched"]
bproc_curr_sched = sched_table_dict["bproc_curr_sched"]
bproc_par_sched = sched_table_dict["bproc_par_sched"]
bproc_derv_sched = sched_table_dict["bproc_derv_sched"]

branch_links = bluespec_interface.get_branch_parent_lids()

#---------------------------------------------

num_links_decr = num_links-1
num_links_incr = num_links+1
# num_links+2 is for the "world" link, used in backward pass to generate updates
# for leaf links
max_link_ref = num_links+1
max_link_counter_bitwidth = int(floor(log2(max_link_ref)))+1

num_PEs_incr = num_PEs+1

num_branches = len(branch_links)
num_branches_incr = num_branches+1

#---------------------------------------------

for idx_backward_stream in range(len_bproc_sched):
    if (idx_backward_stream == 0):
        bw.writeLine("if (idx_backward_stream == {idx_backward_stream}) begin".format(idx_backward_stream=idx_backward_stream))
    else:
        bw.writeLine("else if (idx_backward_stream == {idx_backward_stream}) begin".format(idx_backward_stream=idx_backward_stream))
    for j in range(num_PEs):
        link_out_curr_PE = bproc_curr_sched[idx_backward_stream][j]
        if link_out_curr_PE == 0 or link_out_curr_PE == num_links+1:
            # nop
            continue

        bw.writeLine("    new_dtaudq[{link_out_curr_PE}][{link_out_derv_PE}] = dtau_curr_out_dqPE{PE_id};".format(
                # schedule link ids are 1-indexed, we subtract one to update matrix
                link_out_curr_PE=bproc_curr_sched[idx_backward_stream][j]-1,
                link_out_derv_PE=bproc_derv_sched[idx_backward_stream][j]-1,
                PE_id=j+1
        ))
        bw.writeLine("    new_dtaudqd[{link_out_curr_PE}][{link_out_derv_PE}] = dtau_curr_out_dqdPE{PE_id};".format(
                # schedule link ids are 1-indexed, we subtract one to update matrix
                link_out_curr_PE=bproc_curr_sched[idx_backward_stream][j]-1,
                link_out_derv_PE=bproc_derv_sched[idx_backward_stream][j]-1,
                PE_id=j+1
        ))
    bw.writeLine("end")
