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

fm = FileManager("GradientPipelineFprocIntermediateMuxUnrolling.bsv")
output_file_path = fm.get_output_file_path()
gp_file = open(output_file_path, "w")

bw = BluespecWriter(gp_file, dim_list, num_links)

#---------------------------------------------

bluespec_interface = fpga_codegen.get_bluespec_interface(num_PEs, block_size)
sched_table_dict = bluespec_interface.get_fproc_schedule_tables()

len_fproc_sched = sched_table_dict["len_fproc_sched"]
rnea_fproc_curr_sched = sched_table_dict["rnea_fproc_curr_sched"]
rnea_fproc_par_sched = sched_table_dict["rnea_fproc_par_sched"]
fproc_curr_sched = sched_table_dict["fproc_curr_sched"]
fproc_par_sched = sched_table_dict["fproc_par_sched"]
fproc_derv_sched = sched_table_dict["fproc_derv_sched"]

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

bw.writeLine("case (idx_forward_stream)")
bw.writeLine("    // curr is 1-indexed")
bw.writeLine("    // dim, derv are 0-indexed")
bw.writeLine("    // new_f_inter_acc[curr][dim]")
bw.writeLine("    // slotting into right places in dfi{dq,dqd} matrix of regs")
bw.writeLine("    // new_dfidq_inter_acc[curr][dim][derv]")
bw.writeLine("")

for sched_idx in range(len_fproc_sched):
    link_out_curr_rnea = rnea_fproc_curr_sched[sched_idx]

    bw.writeLine("    {sched_idx}: begin".format(sched_idx=sched_idx))

    bw.writeLine("        new_f_inter_acc[{link_out_curr_rnea}][0] = f_curr_vec_out_AX_rnea;".format(link_out_curr_rnea=link_out_curr_rnea))
    bw.writeLine("        new_f_inter_acc[{link_out_curr_rnea}][1] = f_curr_vec_out_AY_rnea;".format(link_out_curr_rnea=link_out_curr_rnea))
    bw.writeLine("        new_f_inter_acc[{link_out_curr_rnea}][2] = f_curr_vec_out_AZ_rnea;".format(link_out_curr_rnea=link_out_curr_rnea))
    bw.writeLine("        new_f_inter_acc[{link_out_curr_rnea}][3] = f_curr_vec_out_LX_rnea;".format(link_out_curr_rnea=link_out_curr_rnea))
    bw.writeLine("        new_f_inter_acc[{link_out_curr_rnea}][4] = f_curr_vec_out_LY_rnea;".format(link_out_curr_rnea=link_out_curr_rnea))
    bw.writeLine("        new_f_inter_acc[{link_out_curr_rnea}][5] = f_curr_vec_out_LZ_rnea;".format(link_out_curr_rnea=link_out_curr_rnea))

    bw.writeLine("")
    bw.writeLine("        // dfidq")
    bw.writeLine("")

    for dim_id,dim in enumerate(dim_list):
        for PE in range(num_PEs):
            link_out_curr_PE = fproc_curr_sched[sched_idx][PE]
            link_out_derv_PE = fproc_derv_sched[sched_idx][PE]

            bw.writeLine("        new_dfidq_inter_acc[{curr}][{dim_id}][{derv}] = dfdq_curr_vec_out_{dim}_dqPE{PE_id};".format(
                curr=link_out_curr_PE,
                dim_id=dim_id,
                derv=link_out_derv_PE-1, # 0-indexed here, 1-indexed in sched table
                dim=dim,
                PE_id=PE+1
            ))
        bw.writeLine("")

    bw.writeLine("        // dfidqd")
    bw.writeLine("")

    for dim_id,dim in enumerate(dim_list):
        for PE in range(num_PEs):
            link_out_curr_PE = fproc_curr_sched[sched_idx][PE]
            link_out_derv_PE = fproc_derv_sched[sched_idx][PE]

            bw.writeLine("        new_dfidqd_inter_acc[{curr}][{dim_id}][{derv}] = dfdqd_curr_vec_out_{dim}_dqdPE{PE_id};".format(
                curr=link_out_curr_PE,
                dim_id=dim_id,
                derv=link_out_derv_PE-1, # 0-indexed here, 1-indexed in sched table
                dim=dim,
                PE_id=PE+1
            ))
    bw.writeLine("    end")
    bw.writeLine("")

bw.writeLine("endcase")
