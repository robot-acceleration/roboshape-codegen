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

fm = FileManager("GradientPipelineFprocDvDaFlush.bsv")
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
max_link_incr = max_link_ref+1
max_link_counter_bitwidth = int(floor(log2(max_link_ref)))+1
num_links_bitwidth = int(floor(log2(num_links)))+1

num_PEs_incr = num_PEs+1

num_branches = len(branch_links)
num_branches_incr = num_branches+1

#---------------------------------------------

last_index_of_chain_per_PE = {}
for PE_id in range(num_PEs):
    # 1-indexed
    last_index_of_chain_per_PE[PE_id+1] = []

is_multiple_chains = False

# tracking when does each chain end per PE 
for c in range(num_PEs):
    curr_chain_derv = fproc_derv_sched[0][c]
    for r in range(len_fproc_sched):
        new_chain_derv = fproc_derv_sched[r][c]
        if new_chain_derv == curr_chain_derv:
            continue
        else:
            # we track the last index of each chain (so the index _before_ the new
            # chain starts) because we'd like to flush
            # the PE registers after the last index of the chain is processed
            last_index_of_chain_per_PE[c+1].append(r-1)
            curr_chain_derv = new_chain_derv
            is_multiple_chains = True

if not is_multiple_chains:
    # keep tracking state for each PE, no flushing needed
    bw.writeLine("// prevs for each PE")
    bw.writeLine("if (link_out_curr_PE != 0) begin")
    bw.writeLine("    dvda_prev[i] <= dvda_PE;")
    bw.writeLine("end")
else:
    bw.writeLine("if (link_out_curr_PE != 0) begin")
    if_stmt = "    if ("
    for PE_id in range(1, num_PEs+1):
        number_of_chains_per_PE = len(last_index_of_chain_per_PE[PE_id])
        if number_of_chains_per_PE == 0:
            continue
        if_stmt += "(i == {PE_id} && (".format(PE_id=PE_id)
        for li,last_index in enumerate(last_index_of_chain_per_PE[PE_id]):
            if_stmt += "idx_forward_stream == " + str(last_index)
            if li < number_of_chains_per_PE-1:
                if_stmt += "||"
            else:
                if_stmt += ")"
        if_stmt += ")"

        # do we have more flushing to do in next PEs?
        more_flushing = False
        print("PE_id: " + str(PE_id))
        for next_PE in range(PE_id+1, num_PEs+1):
            print("next_PE: " + str(next_PE))
            number_of_chains_next_PE = len(last_index_of_chain_per_PE[next_PE])
            if number_of_chains_next_PE:
                more_flushing = True
                break
        if more_flushing:
            if_stmt += "||"
        else:
            if_stmt += ") begin"
    bw.writeLine(if_stmt)
    bw.writeLine("        dvda_prev[i] <= DvDaIntermediate {dadqd: unpack(0), dvdqd: unpack(0), dadq: unpack(0), dvdq: unpack(0)};")
    bw.writeLine("    end")

    bw.writeLine("    else begin")
    bw.writeLine("        dvda_prev[i] <= dvda_PE;")
    bw.writeLine("    end")

    bw.writeLine("end")
