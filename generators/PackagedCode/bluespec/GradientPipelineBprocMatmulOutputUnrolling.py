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

fm = FileManager("GradientPipelineBprocMatmulOutputUnrolling.bsv")
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
    for block_id in range(block_size):
        bw.writeLine("Bit#(32) new_minv_dtau_R{block_id}_dqdPE{curr_PE} = 0;".format(block_id=block_id+1, curr_PE=curr_PE+1))
    bw.writeLine("")

bw.writeLine("")

bw.writeLine("case (idx_block_minv_stream_dtau)")
bw.writeLine("    // new_minv_dtau[r][c] = new_minv_dtau[r][c] + update;")
bw.writeLine("")

for sched_idx in range(len_block_minv_sched_per_matrix):
    bw.writeLine("    {sched_idx}: begin".format(sched_idx=sched_idx))

    for curr_PE in range(num_PEs):
        for block_id in range(block_size):
            r = output_tile_row_start_sched[sched_idx][curr_PE] + block_id
            # output_tile_col_sched is the same as rh_tile_col_sched
            c = output_tile_col_start_sched[sched_idx][curr_PE] + rh_tile_col_sched[sched_idx][curr_PE]
            if rh_tile_col_sched[sched_idx][curr_PE] < block_size:
                bw.writeLine("        new_minv_dtau_R{block_id}_dqdPE{curr_PE} = new_minv_dtau[{r}][{c}];".format(
                    r=r, c=c, block_id=block_id+1, curr_PE=curr_PE+1)
                )
        bw.writeLine("")

    bw.writeLine("    end")
    bw.writeLine("")

bw.writeLine("endcase")

bw.writeLine("")

for curr_PE in range(num_PEs):
    for block_id in range(block_size):
        bw.writeLine("new_minv_dtau_R{block_id}_dqdPE{curr_PE} = new_minv_dtau_R{block_id}_dqdPE{curr_PE} + minv_vec_out_R{block_id}_dqdPE{curr_PE};".format(
            block_id=block_id+1, curr_PE=curr_PE+1))

bw.writeLine("")

bw.writeLine("case (idx_block_minv_stream_dtau)")
bw.writeLine("    // new_minv_dtau[r][c] = new_minv_dtau[r][c] + update;")
bw.writeLine("")

for sched_idx in range(len_block_minv_sched_per_matrix):
    bw.writeLine("    {sched_idx}: begin".format(sched_idx=sched_idx))

    for curr_PE in range(num_PEs):
        for block_id in range(block_size):
            r = output_tile_row_start_sched[sched_idx][curr_PE] + block_id
            c = output_tile_col_start_sched[sched_idx][curr_PE] + rh_tile_col_sched[sched_idx][curr_PE]
            # accumulating matmul output vector into right place in the final big matrix
            if rh_tile_col_sched[sched_idx][curr_PE] < block_size:
                bw.writeLine("        new_minv_dtau[{r}][{c}] = new_minv_dtau_R{block_id}_dqdPE{curr_PE};".format(
                    r=r, c=c, block_id=block_id+1, curr_PE=curr_PE+1)
                )
        bw.writeLine("")

    bw.writeLine("    end")
    bw.writeLine("")

bw.writeLine("endcase")

bw.writeLine("")
