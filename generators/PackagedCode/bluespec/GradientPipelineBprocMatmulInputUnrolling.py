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

fm = FileManager("GradientPipelineBprocMatmulInputUnrolling.bsv")
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

matmul_sched_table_dict = bluespec_interface.get_matmul_schedule_tables()
len_block_minv_sched_per_matrix = matmul_sched_table_dict["len_block_minv_sched_per_matrix"]
n_tiles = matmul_sched_table_dict["n_tiles"]
lh_tile_sched = matmul_sched_table_dict["lh_tile_sched"]
rh_tile_sched = matmul_sched_table_dict["rh_tile_sched"]
output_tile_incr_sched = matmul_sched_table_dict["output_tile_incr_sched"]
rh_tile_col_sched = matmul_sched_table_dict["rh_tile_col_sched"]

lh_tile_row_start_sched = matmul_sched_table_dict["lh_tile_row_start_sched"]
lh_tile_col_start_sched = matmul_sched_table_dict["lh_tile_col_start_sched"]
rh_tile_row_start_sched = matmul_sched_table_dict["rh_tile_row_start_sched"]
rh_tile_col_start_sched = matmul_sched_table_dict["rh_tile_col_start_sched"]
output_tile_row_start_sched = matmul_sched_table_dict["output_tile_row_start_sched"]
output_tile_col_start_sched = matmul_sched_table_dict["output_tile_col_start_sched"]

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

for curr_PE in range(num_PEs):
    for block_c in range(block_size):
        for block_r in range(block_size):
            bw.writeLine("Bit#(32) minv_block_in_R{block_r}_C{block_c}_dqdPE{curr_PE} = 0;".format(
                block_r=block_r+1, block_c=block_c+1, curr_PE=curr_PE+1
            ))

bw.writeLine("")

for curr_PE in range(num_PEs):
    for block_r in range(block_size):
        bw.writeLine("Bit#(32) dtau_vec_in_R{block_r}_dqdPE{curr_PE} = 0;".format(
            block_r=block_r+1, curr_PE=curr_PE+1
        ))

bw.writeLine("")

bw.writeLine("case (idx_block_minv_feed_dtau)")
bw.writeLine("")

for sched_idx in range(len_block_minv_sched_per_matrix):
    bw.writeLine("    {sched_idx}: begin".format(sched_idx=sched_idx))

    for curr_PE in range(num_PEs):
        for block_c in range(block_size):
            for block_r in range(block_size):
                minv_r = lh_tile_row_start_sched[sched_idx][curr_PE] + block_r
                minv_c = lh_tile_col_start_sched[sched_idx][curr_PE] + block_c
                bw.writeLine("        minv_block_in_R{block_r}_C{block_c}_dqdPE{curr_PE} = minv[{minv_r}][{minv_c}];".format(
                        block_r=block_r+1, block_c=block_c+1, curr_PE=curr_PE+1,
                        minv_r=minv_r, minv_c=minv_c
                ))
            bw.writeLine("")

    for curr_PE in range(num_PEs):
        for block_r in range(block_size):
            dtau_r = rh_tile_row_start_sched[sched_idx][curr_PE] + block_r
            dtau_c = rh_tile_col_start_sched[sched_idx][curr_PE] + rh_tile_col_sched[sched_idx][curr_PE]
            if rh_tile_col_sched[sched_idx][curr_PE] < block_size:
                bw.writeLine("        dtau_vec_in_R{block_r}_dqdPE{curr_PE} = dtau_mat_curr[{dtau_r}][{dtau_c}];".format(
                        block_r=block_r+1, curr_PE=curr_PE+1,
                        dtau_r=dtau_r, dtau_c=dtau_c
                ))
            else:
                bw.writeLine("        dtau_vec_in_R{block_r}_dqdPE{curr_PE} = 0;".format(
                        block_r=block_r+1, curr_PE=curr_PE+1,
                ))
        bw.writeLine("")

    bw.writeLine("    end")
    bw.writeLine("")

bw.writeLine("endcase")
bw.writeLine("")

for curr_PE in range(num_PEs):
    for block_c in range(block_size):
        for block_r in range(block_size):
            bw.writeLine("bproc.minv_block_in_R{block_r}_C{block_c}_dqdPE{curr_PE}(minv_block_in_R{block_r}_C{block_c}_dqdPE{curr_PE});".format(
                    block_r=block_r+1, block_c=block_c+1, 
                    curr_PE=curr_PE+1
            ))
        bw.writeLine("")

    for block_r in range(block_size):
        bw.writeLine("bproc.dtau_vec_in_R{block_r}_dqdPE{curr_PE}(dtau_vec_in_R{block_r}_dqdPE{curr_PE});".format(
                block_r=block_r+1, curr_PE=curr_PE+1
        ))
    bw.writeLine("")
