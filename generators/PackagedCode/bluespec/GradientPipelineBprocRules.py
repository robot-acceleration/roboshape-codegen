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

fm = FileManager("GradientPipelineBprocRules.bsv")
output_file_path = fm.get_output_file_path()
gp_file = open(output_file_path, "w")

bw = BluespecWriter(gp_file, dim_list, num_links, block_size)

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
# num_links+1 is for the "world" link, used in backward pass to generate updates
# for leaf links
max_link_ref = num_links+1
max_link_counter_bitwidth = int(floor(log2(max_link_ref)))+1

num_PEs_incr = num_PEs+1

num_branches = len(branch_links)
num_branches_incr = num_branches+1

n_tiles_bitwidth = int(floor(log2(n_tiles)))+1
block_size_bitwidth = int(floor(log2(block_size)))+1


#---------------------------------------------

bw.writeLine("Reg#(Bit#(32)) idx_backward_feed <- mkReg(0);")
bw.writeLine("Reg#(Bit#(32)) idx_backward_stream <- mkReg(0);")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("Reg#(Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Bit#(32)))) dtaudq <- mkReg(unpack(0));")
bw.writeLineWithFormatSpecReplace("Reg#(Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Bit#(32)))) dtaudqd <- mkReg(unpack(0));")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("Reg#(Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Bit#(32)))) minv_dtaudq <- mkReg(unpack(0));")
bw.writeLineWithFormatSpecReplace("Reg#(Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Bit#(32)))) minv_dtaudqd <- mkReg(unpack(0));")
bw.writeLine("")
bw.writeLine("Reg#(Bit#(32)) offset_out <- mkReg(0);")
bw.writeLine("Ehr#(2,Bool) flip <- mkEhr(True);")
bw.writeLine("Ehr#(2,Bool) mul_mat <-mkEhr(False);")
bw.writeLine("Reg#(Bool) keep_backward <-mkReg(True);")
bw.writeLine("")
bw.writeLine("// True is feed, False is stream")
bw.writeLine("Ehr#(2,Bool) block_minv_phase <- mkEhr(True);")
bw.writeLine("")
bw.writeLine("rule restart_bw (!mul_mat[0] && !keep_backward);")
bw.writeLine("    keep_backward <= True;")
bw.writeLine("endrule")
bw.writeLine("")
bw.writeLine("//*******************************/")
bw.writeLine("//   SCHEDULE TABLES BPROC       /")
bw.writeLine("//*******************************/")
bw.writeLine("")

###
bw.writeLine("Bit#(32) len_bproc_sched = {len_bproc_sched};".format(len_bproc_sched=len_bproc_sched))
bw.writeLine("")

bw.writeLine("Vector#({len_bproc_sched}, Bit#({max_link_counter_bitwidth})) rnea_bproc_curr_sched = ".format(
    len_bproc_sched=len_bproc_sched,
    max_link_counter_bitwidth=max_link_counter_bitwidth
))
bw.writeLine(bw.get_row_vec_bluespec("    ", rnea_bproc_curr_sched) + ";")
bw.writeLine("")

bw.writeLine("Vector#({len_bproc_sched}, Bit#({max_link_counter_bitwidth})) rnea_bproc_par_sched = ".format(
    len_bproc_sched=len_bproc_sched,
    max_link_counter_bitwidth=max_link_counter_bitwidth
))
bw.writeLine(bw.get_row_vec_bluespec("    ", rnea_bproc_par_sched) + ";")
bw.writeLine("")

bw.writeLine("Vector#({len_bproc_sched}, Vector#({num_PEs}, Bit#({max_link_counter_bitwidth}))) bproc_curr_sched = vec(".format(
    len_bproc_sched=len_bproc_sched,
    num_PEs=num_PEs,
    max_link_counter_bitwidth=max_link_counter_bitwidth
))
for i,row in enumerate(bproc_curr_sched):
    ending = "" if i == len_bproc_sched-1 else ","
    bw.writeLine(bw.get_row_vec_bluespec("    ", row) + ending)
bw.writeLine(");")

bw.writeLine("Vector#({len_bproc_sched}, Vector#({num_PEs}, Bit#({max_link_counter_bitwidth}))) bproc_par_sched = vec(".format(
    len_bproc_sched=len_bproc_sched,
    num_PEs=num_PEs,
    max_link_counter_bitwidth=max_link_counter_bitwidth
))
for i,row in enumerate(bproc_par_sched):
    ending = "" if i == len_bproc_sched-1 else ","
    bw.writeLine(bw.get_row_vec_bluespec("    ", row) + ending)
bw.writeLine(");")
bw.writeLine("")

bw.writeLine("Vector#({len_bproc_sched}, Vector#({num_PEs}, Bit#({max_link_counter_bitwidth}))) bproc_derv_sched = vec(".format(
    len_bproc_sched=len_bproc_sched,
    num_PEs=num_PEs,
    max_link_counter_bitwidth=max_link_counter_bitwidth
))
for i,row in enumerate(bproc_derv_sched):
    ending = "" if i == len_bproc_sched-1 else ","
    bw.writeLine(bw.get_row_vec_bluespec("    ", row) + ending)
bw.writeLine(");")
bw.writeLine("")
###

bw.writeLine("// s/8/NUM_LINKS+2 because we use up to link 8 and is 1-indexed")
bw.writeLineWithFormatSpecReplace("Vector#({max_link_ref},Reg#(FUpdIntermediate)) f_upd_acc <- replicateM(mkReg(unpack(0)));")
bw.writeLine("")
bw.writeLine("// for bproc ext branching")
bw.writeLine("// s/8/NUM_PES+1")
bw.writeLine("Vector#({num_PEs_incr},Reg#(DfUpdIntermediate)) df_upd_prev <- replicateM(mkReg(unpack(0)));".format(num_PEs_incr=num_PEs_incr))
bw.writeLine("// s/1/NUM_BRANCHES+1")
bw.writeLine("// s/8/NUM_PES+1")
bw.writeLine("Vector#({num_PEs_incr}, Vector#({num_branches_incr},Reg#(DfUpdIntermediate))) df_upd_branch <- replicateM(replicateM(mkReg(unpack(0))));".format(num_PEs_incr=num_PEs_incr, num_branches_incr=num_branches_incr))
bw.writeLine("")

###
bw.writeLine("function Bit#({max_link_counter_bitwidth}) branchTableBproc(Bit#({max_link_counter_bitwidth}) branch_link);".format(max_link_counter_bitwidth=max_link_counter_bitwidth))
bw.writeLine("    case (branch_link)")

for i,branch_link in enumerate(branch_links):
    bw.writeLine("        {branch_link}: return {lookup_index};".format(
        branch_link=branch_link,
        lookup_index=i+1
    ))

bw.writeLine("        default: return 0;")
bw.writeLine("    endcase")
bw.writeLine("endfunction")
bw.writeLine("")
###


bw.writeLineWithFormatSpecReplace("function DfUpdIntermediate selectDfUpdBranch(Bit#({max_link_counter_bitwidth}) curr_link, Bit#({max_link_counter_bitwidth}) curr_PE);")
###
if num_branches > 0:
    if_stmt = "    if ("
    for i,branch_link in enumerate(branch_links):
        if_stmt += "curr_link == " + str(branch_link)
        if i < num_branches-1:
            if_stmt += " || "
    if_stmt += ") begin"

    bw.writeLine(if_stmt)
    bw.writeLine("        return df_upd_branch[curr_PE][branchTableBproc(curr_link)];")
    bw.writeLine("    end")
if num_branches > 0:
    bw.writeLine("    else begin")
    bw.writeLine("        return df_upd_prev[curr_PE];")
    bw.writeLine("    end")
else:
    bw.writeLine("    return df_upd_prev[curr_PE];")
bw.writeLine("endfunction")
###
bw.writeLine("")
bw.writeLine("//*******************************/")

bw.writeLine("//******** BLOCK MINV MULTIPLICATION SCHEDULE ******/")
bw.writeLine("")
bw.writeLine("// We represent the matmul as lh_matrix * rh_matrix = output_matrix")
bw.writeLine("// in our case, lh_matrix is minv, rh_matrix is dtau{dq,dqd}")
bw.writeLine("// output_matrix is minv_dtau{dq,dqd}")
bw.writeLine("")
bw.writeLine("// We multiply lh_tile * rh_tile and increment output_tile by the")
bw.writeLine("// resulting block output.")
bw.writeLine("")
bw.writeLine("// Each PE does: matrix * col-vector = col-vector")
bw.writeLine("// The lh_tile * rh_tile matmul happens column-wise, where we have each PE")
bw.writeLine("// processing one column of rh_tile, and outputing one column of output_tile.")
bw.writeLine("// So:")
bw.writeLine("// PE1: lh_tile * rh_tile[][0] = output_tile[][0]")
bw.writeLine("// PE2: lh_tile * rh_tile[][1] = output_tile[][1]")
bw.writeLine("// ...")
bw.writeLine("// PE7: lh_tile * rh_tile[][6] = output_tile[][6]")
bw.writeLine("")

###
bw.writeLine("Bit#({n_tiles_bitwidth}) n_tiles = {n_tiles};".format(n_tiles_bitwidth=n_tiles_bitwidth, n_tiles=n_tiles))
bw.writeLine("// n_rows_tile = n_cols_tile = 7")
bw.writeLine("Bit#({block_size_bitwidth}) tile_size = {block_size};".format(block_size_bitwidth=block_size_bitwidth, block_size=block_size))
bw.writeLine("")
bw.writeLine("// s/Bit#(3)/number of times we need to iterate over tiles")
bw.writeLine("Bit#(32) len_block_minv_sched_per_matrix = {len_block_minv_sched_per_matrix};".format(len_block_minv_sched_per_matrix=len_block_minv_sched_per_matrix))
bw.writeLine("// since we go both dtaudq and dtaudqd, so twice of entire schedule")
bw.writeLine("// length")
bw.writeLine("Bit#(32) len_block_minv_sched_total = len_block_minv_sched_per_matrix * 2;")
bw.writeLine("")
bw.writeLine("Reg#(Bit#(32)) idx_block_minv_feed <- mkReg(0);")
bw.writeLine("Reg#(Bit#(32)) idx_block_minv_stream <- mkReg(0);")
bw.writeLine("")
bw.writeLine("// we have the same block schedule for dtaudq and dtaudqd")
bw.writeLine("")

###
bw.writeLine("// {lh,rh,output}_tile_{incr}_sched just kept to easily read tile sched")
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(lh_tile_sched, "lh_tile_sched",
        len_block_minv_sched_per_matrix, num_PEs, n_tiles_bitwidth
    )
)

bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(rh_tile_sched, "rh_tile_sched",
        len_block_minv_sched_per_matrix, num_PEs, n_tiles_bitwidth
    )
)

bw.writeLine("// which tile on the output matrix should the result be added to")
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(output_tile_incr_sched, "output_tile_incr_sched",
        len_block_minv_sched_per_matrix, num_PEs, n_tiles_bitwidth
    )
)
###

###
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(lh_tile_row_start_sched, "lh_tile_row_start_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)

bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(rh_tile_row_start_sched, "rh_tile_row_start_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)

bw.writeLine("// which tile on the output matrix should the result be added to")
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(output_tile_row_start_sched, "output_tile_row_start_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)
###

###
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(lh_tile_col_start_sched, "lh_tile_col_start_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)

bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(rh_tile_col_start_sched, "rh_tile_col_start_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)

bw.writeLine("// which tile on the output matrix should the result be added to")
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(output_tile_col_start_sched, "output_tile_col_start_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)
###

bw.writeLine("")
bw.writeLine("// s/Bit#(4)/number of cols per tile")
bw.writeLine("// this serves as both the input rh_tile col schedule, and the output_tile col")
bw.writeLine("// we slot the output col into.")
bw.writeLine(
    bw.get_matmul_sched_table_full_decl_str_bluespec(rh_tile_col_sched, "rh_tile_col_sched",
        len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth
    )
)
###

bw.writeLine("")
bw.writeLine("//*******************************/")
bw.writeLine("")

bw.writeLine("rule feedBproc (flip[0] && !mul_mat[0] && keep_backward && idx_backward_feed < len_bproc_sched);")
bw.writeLine("    $display(\"=========== BPROC FEED for idx \", idx_backward_feed, \" =============\");")
bw.writeLine("")
bw.writeLine("    flip[0] <= !flip[0];")
bw.writeLine("")
bw.writeLine("    if (idx_backward_feed == len_bproc_sched-1) begin")
bw.writeLine("        idx_backward_feed <= 0;")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        idx_backward_feed <= idx_backward_feed + 1;")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("    let intermediate = intermediate_values.first();")
bw.writeLine("    let trigo = trigo_values.first();")
bw.writeLine("")
bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    // Getting link ids for different components of the schedule")

bw.writeLine("    let link_in_curr_rnea = rnea_bproc_curr_sched[idx_backward_feed];")
bw.writeLine("    let link_in_par_rnea = rnea_bproc_par_sched[idx_backward_feed];")

for i in range(num_PEs):
    bw.writeLine("    Bit#({max_link_counter_bitwidth}) link_in_curr_PE{PE_id} = 0;".format(
            max_link_counter_bitwidth=max_link_counter_bitwidth, PE_id=i+1)
    )
    bw.writeLine("    Bit#({max_link_counter_bitwidth}) link_in_par_PE{PE_id} = 0;".format(
            max_link_counter_bitwidth=max_link_counter_bitwidth, PE_id=i+1)
    )
    bw.writeLine("    Bit#({max_link_counter_bitwidth}) link_in_derv_PE{PE_id} = 0;".format(
            max_link_counter_bitwidth=max_link_counter_bitwidth, PE_id=i+1)
    )
bw.writeLine("")

bw.writeLine("`include \"GradientPipelineBprocSchedUnrolling.bsv\"")

bw.writeLine("")
bw.writeLine("    // grabbing the input values are 1-indexed")
bw.writeLine("    let input_curr_rnea = intermediate[link_in_curr_rnea];")
bw.writeLine("    let input_par_rnea = intermediate[link_in_par_rnea];")

for i in range(num_PEs):
    bw.writeLine("    let input_curr_PE{PE_id} = intermediate[link_in_curr_PE{PE_id}];".format(PE_id=i+1))
bw.writeLine("")

for i in range(num_PEs):
    bw.writeLine("    let input_par_PE{PE_id} = intermediate[link_in_par_PE{PE_id}];".format(PE_id=i+1))
bw.writeLine("")

bw.writeLine("    // external branches")
for i in range(num_PEs):
    bw.writeLine("    let df_upd_PE{PE_id} = selectDfUpdBranch(link_in_curr_PE{PE_id},{PE_id});".format(PE_id=i+1))
bw.writeLine("")
###

bw.writeLine("    //********************************************")
bw.writeLine("")
bw.writeLine("    //-----------------------------------")
bw.writeLine("")
bw.writeLine("    bproc.get_data();")
bw.writeLine("")
bw.writeLine("    //------- RNEA INPUTS ---------")
bw.writeLine("")
bw.writeLine("    bproc.link_in_rnea(link_in_curr_rnea);")
bw.writeLine("    bproc.sinq_val_in_rnea(trigo[link_in_curr_rnea].sinq);")
bw.writeLine("    bproc.cosq_val_in_rnea(trigo[link_in_curr_rnea].cosq);")
bw.writeLine("    bproc.f_prev_vec_in_AX_rnea(intermediate[link_in_par_rnea].f[0]);")
bw.writeLine("    bproc.f_prev_vec_in_AY_rnea(intermediate[link_in_par_rnea].f[1]);")
bw.writeLine("    bproc.f_prev_vec_in_AZ_rnea(intermediate[link_in_par_rnea].f[2]);")
bw.writeLine("    bproc.f_prev_vec_in_LX_rnea(intermediate[link_in_par_rnea].f[3]);")
bw.writeLine("    bproc.f_prev_vec_in_LY_rnea(intermediate[link_in_par_rnea].f[4]);")
bw.writeLine("    bproc.f_prev_vec_in_LZ_rnea(intermediate[link_in_par_rnea].f[5]);")
bw.writeLine("    bproc.f_upd_curr_vec_in_AX_rnea(f_upd_acc[link_in_curr_rnea].f_upd[0]);")
bw.writeLine("    bproc.f_upd_curr_vec_in_AY_rnea(f_upd_acc[link_in_curr_rnea].f_upd[1]);")
bw.writeLine("    bproc.f_upd_curr_vec_in_AZ_rnea(f_upd_acc[link_in_curr_rnea].f_upd[2]);")
bw.writeLine("    bproc.f_upd_curr_vec_in_LX_rnea(f_upd_acc[link_in_curr_rnea].f_upd[3]);")
bw.writeLine("    bproc.f_upd_curr_vec_in_LY_rnea(f_upd_acc[link_in_curr_rnea].f_upd[4]);")
bw.writeLine("    bproc.f_upd_curr_vec_in_LZ_rnea(f_upd_acc[link_in_curr_rnea].f_upd[5]);")
bw.writeLine("")
bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    //------- DQ INPUTS -----------")
bw.writeLine("")


for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    bproc.link_in_dqPE{PE_id}(link_in_curr_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    bproc.derv_in_dqPE{PE_id}(link_in_derv_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    bproc.sinq_val_in_dqPE{PE_id}(trigo[link_in_curr_PE{PE_id}].sinq);".format(PE_id=PE_id))
    bw.writeLine("    bproc.cosq_val_in_dqPE{PE_id}(trigo[link_in_curr_PE{PE_id}].cosq);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_prev_vec_in_AX_dqPE{PE_id}(input_par_PE{PE_id}.dfidq[0][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_prev_vec_in_AY_dqPE{PE_id}(input_par_PE{PE_id}.dfidq[1][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_prev_vec_in_AZ_dqPE{PE_id}(input_par_PE{PE_id}.dfidq[2][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_prev_vec_in_LX_dqPE{PE_id}(input_par_PE{PE_id}.dfidq[3][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_prev_vec_in_LY_dqPE{PE_id}(input_par_PE{PE_id}.dfidq[4][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_prev_vec_in_LZ_dqPE{PE_id}(input_par_PE{PE_id}.dfidq[5][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.f_upd_curr_vec_in_AX_dqPE{PE_id}(f_upd_acc[link_in_curr_PE{PE_id}].f_upd[0]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.f_upd_curr_vec_in_AY_dqPE{PE_id}(f_upd_acc[link_in_curr_PE{PE_id}].f_upd[1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.f_upd_curr_vec_in_AZ_dqPE{PE_id}(f_upd_acc[link_in_curr_PE{PE_id}].f_upd[2]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.f_upd_curr_vec_in_LX_dqPE{PE_id}(f_upd_acc[link_in_curr_PE{PE_id}].f_upd[3]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.f_upd_curr_vec_in_LY_dqPE{PE_id}(f_upd_acc[link_in_curr_PE{PE_id}].f_upd[4]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.f_upd_curr_vec_in_LZ_dqPE{PE_id}(f_upd_acc[link_in_curr_PE{PE_id}].f_upd[5]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_upd_curr_vec_in_AX_dqPE{PE_id}(df_upd_PE{PE_id}.dfdq_upd[0]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_upd_curr_vec_in_AY_dqPE{PE_id}(df_upd_PE{PE_id}.dfdq_upd[1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_upd_curr_vec_in_AZ_dqPE{PE_id}(df_upd_PE{PE_id}.dfdq_upd[2]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_upd_curr_vec_in_LX_dqPE{PE_id}(df_upd_PE{PE_id}.dfdq_upd[3]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_upd_curr_vec_in_LY_dqPE{PE_id}(df_upd_PE{PE_id}.dfdq_upd[4]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdq_upd_curr_vec_in_LZ_dqPE{PE_id}(df_upd_PE{PE_id}.dfdq_upd[5]);".format(PE_id=PE_id))
    bw.writeLine("")

bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    //------- DQD INPUTS -----------")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    bproc.link_in_dqdPE{PE_id}(link_in_curr_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    bproc.derv_in_dqdPE{PE_id}(link_in_derv_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    bproc.sinq_val_in_dqdPE{PE_id}(trigo[link_in_curr_PE{PE_id}].sinq);".format(PE_id=PE_id))
    bw.writeLine("    bproc.cosq_val_in_dqdPE{PE_id}(trigo[link_in_curr_PE{PE_id}].cosq);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_prev_vec_in_AX_dqdPE{PE_id}(input_par_PE{PE_id}.dfidqd[0][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_prev_vec_in_AY_dqdPE{PE_id}(input_par_PE{PE_id}.dfidqd[1][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_prev_vec_in_AZ_dqdPE{PE_id}(input_par_PE{PE_id}.dfidqd[2][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_prev_vec_in_LX_dqdPE{PE_id}(input_par_PE{PE_id}.dfidqd[3][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_prev_vec_in_LY_dqdPE{PE_id}(input_par_PE{PE_id}.dfidqd[4][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_prev_vec_in_LZ_dqdPE{PE_id}(input_par_PE{PE_id}.dfidqd[5][link_in_derv_PE{PE_id}-1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_upd_curr_vec_in_AX_dqdPE{PE_id}(df_upd_PE{PE_id}.dfdqd_upd[0]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_upd_curr_vec_in_AY_dqdPE{PE_id}(df_upd_PE{PE_id}.dfdqd_upd[1]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_upd_curr_vec_in_AZ_dqdPE{PE_id}(df_upd_PE{PE_id}.dfdqd_upd[2]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_upd_curr_vec_in_LX_dqdPE{PE_id}(df_upd_PE{PE_id}.dfdqd_upd[3]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_upd_curr_vec_in_LY_dqdPE{PE_id}(df_upd_PE{PE_id}.dfdqd_upd[4]);".format(PE_id=PE_id))
    bw.writeLine("    bproc.dfdqd_upd_curr_vec_in_LZ_dqdPE{PE_id}(df_upd_PE{PE_id}.dfdqd_upd[5]);".format(PE_id=PE_id))
    bw.writeLine("")

bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    $display(\"=========== BPROC FEED for idx \", idx_backward_feed, \" END =============\");")
bw.writeLine("")
bw.writeLine("endrule")


bw.writeLine("")
bw.writeLine("// this is because we grab matmul results when we reach the end of the schedule")
bw.writeLine("rule streamBproc (!flip[1] && unpack(bproc.output_ready()) && !mul_mat[1] && keep_backward && idx_backward_stream < len_bproc_sched);")
bw.writeLine("")
bw.writeLine("    $display(\"=========== BPROC STREAM for idx \", idx_backward_stream, \" =============\");")
bw.writeLine("")
bw.writeLine("    if (idx_backward_stream == len_bproc_sched-1) begin")
bw.writeLine("        idx_backward_stream <= 0;")
bw.writeLine("    end")
bw.writeLine("    else begin          ")
bw.writeLine("        idx_backward_stream <= idx_backward_stream + 1;")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("    flip[1] <= !flip[1];")
bw.writeLine("")
bw.writeLine("    //****************************************")
bw.writeLine("")

bw.writeLine("    let link_out_curr_rnea = rnea_bproc_curr_sched[idx_backward_stream];")
bw.writeLine("    let link_out_par_rnea = rnea_bproc_par_sched[idx_backward_stream];")

bw.writeLine("")
bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    //********* DTAUDQ OUTPUT REDIRECTION ********/")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dtau_curr_out_dqPE{PE_id} = bproc.dtau_curr_out_dqPE{PE_id}();".format(PE_id=PE_id))

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dtau_curr_out_dqdPE{PE_id} = bproc.dtau_curr_out_dqdPE{PE_id}();".format(PE_id=PE_id))

bw.writeLine("")
bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    //********* RNEA OUTPUT REDIRECTION ********/")
bw.writeLine("")
bw.writeLine("    let f_upd_prev_vec_out_AX_rnea = bproc.f_upd_prev_vec_out_AX_rnea();")
bw.writeLine("    let f_upd_prev_vec_out_AY_rnea = bproc.f_upd_prev_vec_out_AY_rnea();")
bw.writeLine("    let f_upd_prev_vec_out_AZ_rnea = bproc.f_upd_prev_vec_out_AZ_rnea();")
bw.writeLine("    let f_upd_prev_vec_out_LX_rnea = bproc.f_upd_prev_vec_out_LX_rnea();")
bw.writeLine("    let f_upd_prev_vec_out_LY_rnea = bproc.f_upd_prev_vec_out_LY_rnea();")
bw.writeLine("    let f_upd_prev_vec_out_LZ_rnea = bproc.f_upd_prev_vec_out_LZ_rnea();")
bw.writeLine("")
bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    //********* DQPE OUTPUT REDIRECTION ********/")
bw.writeLine("")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dfdq_upd_prev_vec_out_AX_dqPE{PE_id} = bproc.dfdq_upd_prev_vec_out_AX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_upd_prev_vec_out_AY_dqPE{PE_id} = bproc.dfdq_upd_prev_vec_out_AY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_upd_prev_vec_out_AZ_dqPE{PE_id} = bproc.dfdq_upd_prev_vec_out_AZ_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_upd_prev_vec_out_LX_dqPE{PE_id} = bproc.dfdq_upd_prev_vec_out_LX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_upd_prev_vec_out_LY_dqPE{PE_id} = bproc.dfdq_upd_prev_vec_out_LY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_upd_prev_vec_out_LZ_dqPE{PE_id} = bproc.dfdq_upd_prev_vec_out_LZ_dqPE{PE_id}();".format(PE_id=PE_id))

bw.writeLine("    ")
bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    //********* DQDPE OUTPUT REDIRECTION ********/")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dfdqd_upd_prev_vec_out_AX_dqdPE{PE_id} = bproc.dfdqd_upd_prev_vec_out_AX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_upd_prev_vec_out_AY_dqdPE{PE_id} = bproc.dfdqd_upd_prev_vec_out_AY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_upd_prev_vec_out_AZ_dqdPE{PE_id} = bproc.dfdqd_upd_prev_vec_out_AZ_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_upd_prev_vec_out_LX_dqdPE{PE_id} = bproc.dfdqd_upd_prev_vec_out_LX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_upd_prev_vec_out_LY_dqdPE{PE_id} = bproc.dfdqd_upd_prev_vec_out_LY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_upd_prev_vec_out_LZ_dqdPE{PE_id} = bproc.dfdqd_upd_prev_vec_out_LZ_dqdPE{PE_id}();".format(PE_id=PE_id))

bw.writeLine("")
bw.writeLine("//    Vector#(7, Bit#(32)) dtaudq_row = vec(dtau_curr_out_dqPE1, dtau_curr_out_dqPE2, dtau_curr_out_dqPE3, dtau_curr_out_dqPE4, dtau_curr_out_dqPE5, dtau_curr_out_dqPE6, dtau_curr_out_dqPE7);")
bw.writeLine("//    Vector#(7, Bit#(32)) dtaudqd_row = vec(dtau_curr_out_dqdPE1, dtau_curr_out_dqdPE2, dtau_curr_out_dqdPE3, dtau_curr_out_dqdPE4, dtau_curr_out_dqdPE5, dtau_curr_out_dqdPE6, dtau_curr_out_dqdPE7);")
bw.writeLine("//    $display(\"row out idx stream: \", fshow(idx_backward_stream));")
bw.writeLine("//    $display(\"dtaudq_row: \", fshow(dtaudq_row));")
bw.writeLine("//    $display(\"dtaudqd_row: \", fshow(dtaudqd_row));")
bw.writeLine("")
bw.writeLine("    let new_dtaudq = dtaudq;")
bw.writeLine("    let new_dtaudqd = dtaudqd;")
bw.writeLine("    ")

bw.writeLine("`include \"GradientPipelineBprocDtauUpdateUnrolling.bsv\"")

bw.writeLine("")
bw.writeLine("    dtaudq <= new_dtaudq;")
bw.writeLine("    dtaudqd <= new_dtaudqd;")
bw.writeLine("")
bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    Vector#(6,Bit#(32)) vec_f_upd_rnea = vec(f_upd_prev_vec_out_AX_rnea,f_upd_prev_vec_out_AY_rnea,f_upd_prev_vec_out_AZ_rnea,f_upd_prev_vec_out_LX_rnea,f_upd_prev_vec_out_LY_rnea,f_upd_prev_vec_out_LZ_rnea);")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    Vector#(6,Bit#(32)) vec_dfdq_upd_PE{PE_id} = vec(dfdq_upd_prev_vec_out_AX_dqPE{PE_id},dfdq_upd_prev_vec_out_AY_dqPE{PE_id},dfdq_upd_prev_vec_out_AZ_dqPE{PE_id},dfdq_upd_prev_vec_out_LX_dqPE{PE_id},dfdq_upd_prev_vec_out_LY_dqPE{PE_id},dfdq_upd_prev_vec_out_LZ_dqPE{PE_id});".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    Vector#(6,Bit#(32)) vec_dfdqd_upd_PE{PE_id} = vec(dfdqd_upd_prev_vec_out_AX_dqdPE{PE_id},dfdqd_upd_prev_vec_out_AY_dqdPE{PE_id},dfdqd_upd_prev_vec_out_AZ_dqdPE{PE_id},dfdqd_upd_prev_vec_out_LX_dqdPE{PE_id},dfdqd_upd_prev_vec_out_LY_dqdPE{PE_id},dfdqd_upd_prev_vec_out_LZ_dqdPE{PE_id});".format(PE_id=PE_id))
bw.writeLine("")

bw.writeLine("    //*******************************************")
bw.writeLine("")
bw.writeLine("    FUpdIntermediate f_upd_rnea = FUpdIntermediate {f_upd: vec_f_upd_rnea};")
bw.writeLine("    FUpdIntermediate f_upd_add;")
bw.writeLine("    let f_upd_reg = f_upd_acc[link_out_par_rnea];")
bw.writeLine("    f_upd_add.f_upd[0] = f_upd_reg.f_upd[0] + f_upd_rnea.f_upd[0];")
bw.writeLine("    f_upd_add.f_upd[1] = f_upd_reg.f_upd[1] + f_upd_rnea.f_upd[1];")
bw.writeLine("    f_upd_add.f_upd[2] = f_upd_reg.f_upd[2] + f_upd_rnea.f_upd[2];")
bw.writeLine("    f_upd_add.f_upd[3] = f_upd_reg.f_upd[3] + f_upd_rnea.f_upd[3];")
bw.writeLine("    f_upd_add.f_upd[4] = f_upd_reg.f_upd[4] + f_upd_rnea.f_upd[4];")
bw.writeLine("    f_upd_add.f_upd[5] = f_upd_reg.f_upd[5] + f_upd_rnea.f_upd[5];")
bw.writeLine("")
bw.writeLine("    if (link_out_par_rnea != 0) begin")
bw.writeLine("        //// branching case")
###
if num_branches > 0:
    if_stmt = "        if ("
    for i,branch_link in enumerate(branch_links):
        if_stmt += "link_out_par_rnea == " + str(branch_link)
        if i < num_branches-1:
            if_stmt += " || "
    if_stmt += ") begin"

    bw.writeLine(if_stmt)
    bw.writeLine("            f_upd_acc[link_out_par_rnea] <= f_upd_add;")
    bw.writeLine("        end")
if num_branches > 0:
    bw.writeLine("        else begin")
    bw.writeLine("            f_upd_acc[link_out_par_rnea] <= f_upd_rnea;")
    bw.writeLine("        end")
else:
    bw.writeLine("        f_upd_acc[link_out_par_rnea] <= f_upd_rnea;")
bw.writeLine("    end")
###
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    DfUpdIntermediate df_upd_PE{PE_id} = DfUpdIntermediate {{dfdqd_upd: unpack(pack(vec_dfdqd_upd_PE{PE_id})), dfdq_upd: unpack(pack(vec_dfdq_upd_PE{PE_id}))}};".format(PE_id=PE_id))

bw.writeLine("")
bw.writeLine("    for (int i=1; i<={num_PEs}; i=i+1) begin".format(num_PEs=num_PEs))
bw.writeLine("        // this is same calculation as link_out_par_PEx, just wanted to for-loopify it")
bw.writeLine("        let link_out_curr_PE = bproc_curr_sched[idx_backward_stream][i-1];")
bw.writeLine("        let link_out_par_PE = bproc_par_sched[idx_backward_stream][i-1];")
bw.writeLine("        DfUpdIntermediate df_upd_PE;")
bw.writeLine("        case(i)")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("            {PE_id}: df_upd_PE = df_upd_PE{PE_id}; ".format(PE_id=PE_id))

bw.writeLine("        endcase")
bw.writeLine("")
bw.writeLine("`include \"GradientPipelineBprocDfUpdFlush.bsv\"")
bw.writeLine("")
bw.writeLine("        let df_upd_reg = df_upd_branch[i][branchTableBproc(link_out_par_PE)];")
bw.writeLine("        DfUpdIntermediate df_upd_acc;")
bw.writeLine("        df_upd_acc.dfdq_upd[0] = df_upd_reg.dfdq_upd[0] + df_upd_PE.dfdq_upd[0];")
bw.writeLine("        df_upd_acc.dfdq_upd[1] = df_upd_reg.dfdq_upd[1] + df_upd_PE.dfdq_upd[1];")
bw.writeLine("        df_upd_acc.dfdq_upd[2] = df_upd_reg.dfdq_upd[2] + df_upd_PE.dfdq_upd[2];")
bw.writeLine("        df_upd_acc.dfdq_upd[3] = df_upd_reg.dfdq_upd[3] + df_upd_PE.dfdq_upd[3];")
bw.writeLine("        df_upd_acc.dfdq_upd[4] = df_upd_reg.dfdq_upd[4] + df_upd_PE.dfdq_upd[4];")
bw.writeLine("        df_upd_acc.dfdq_upd[5] = df_upd_reg.dfdq_upd[5] + df_upd_PE.dfdq_upd[5];")
bw.writeLine("        df_upd_acc.dfdqd_upd[0] = df_upd_reg.dfdqd_upd[0] + df_upd_PE.dfdqd_upd[0];")
bw.writeLine("        df_upd_acc.dfdqd_upd[1] = df_upd_reg.dfdqd_upd[1] + df_upd_PE.dfdqd_upd[1];")
bw.writeLine("        df_upd_acc.dfdqd_upd[2] = df_upd_reg.dfdqd_upd[2] + df_upd_PE.dfdqd_upd[2];")
bw.writeLine("        df_upd_acc.dfdqd_upd[3] = df_upd_reg.dfdqd_upd[3] + df_upd_PE.dfdqd_upd[3];")
bw.writeLine("        df_upd_acc.dfdqd_upd[4] = df_upd_reg.dfdqd_upd[4] + df_upd_PE.dfdqd_upd[4];")
bw.writeLine("        df_upd_acc.dfdqd_upd[5] = df_upd_reg.dfdqd_upd[5] + df_upd_PE.dfdqd_upd[5];")
bw.writeLine("")
bw.writeLine("        $display(\"df_upd_prev PE\", fshow(i), \" dfdqd_upd: \", fshow(df_upd_PE.dfdqd_upd));")
bw.writeLine("        //// branching case")

###
if num_branches > 0:
    if_stmt = "        if ("
    for i,branch_link in enumerate(branch_links):
        if_stmt += "link_out_par_PE == " + str(branch_link)
        if i < num_branches-1:
            if_stmt += " || "
    if_stmt += ") begin"

    bw.writeLine(if_stmt)
    bw.writeLine("            df_upd_branch[i][branchTableBproc(link_out_par_PE)] <= df_upd_acc;")
    bw.writeLine("            $display(\"branch acc\", fshow(i), \" dfdqd_upd: \", fshow(df_upd_acc.dfdqd_upd));")
    bw.writeLine("        end")
###
bw.writeLine("    end")

bw.writeLine("")
bw.writeLine("    if (idx_backward_stream == len_bproc_sched-1) begin")
bw.writeLine("        $display(\"feeding first minv tile from streamBproc\");")
bw.writeLine("        mul_mat[1] <= !mul_mat[1];")
bw.writeLine("        keep_backward <= False;")
bw.writeLine("        block_minv_phase[0] <= !block_minv_phase[0];")
bw.writeLine("        idx_block_minv_feed <= idx_block_minv_feed + 1;")
bw.writeLine("")
bw.writeLine("        // PRINT MINV")
bw.writeLine("        $display(\"MINV INPUT:\");")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLine("            $display(fshow(readVReg(minv[i])));")
bw.writeLine("        end")
bw.writeLine("        // PRINT DTAUDQ")
bw.writeLine("        $display(\"DTAUDQ INPUT:\");")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLine("            $display(fshow(new_dtaudq[i]));")
bw.writeLine("        end")
bw.writeLine("        // PRINT DTAUDQD")
bw.writeLine("        $display(\"DTAUDQD INPUT:\");")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLine("            $display(fshow(new_dtaudqd[i]));")
bw.writeLine("        end")
bw.writeLine("")
bw.writeLine("        bproc.get_data_minv();")
bw.writeLine("")
bw.writeLine("")

bw.writeLine("`include \"GradientPipelineBprocMatmulFirstInputUnrolling.bsv\"");

bw.writeLine("    end")
bw.writeLine("")

bw.writeLine("    $display(\"=========== BPROC STREAM for idx \", idx_backward_stream, \" END =============\");")
bw.writeLine("")
bw.writeLine("endrule")
bw.writeLine("")
bw.writeLine("rule feedBlockMinv (mul_mat[0] && block_minv_phase[0] && idx_block_minv_feed < len_block_minv_sched_total);")
bw.writeLine("    $display(\"ENTERING FEED BLOCK MINV\");")
bw.writeLine("")
bw.writeLine("    block_minv_phase[0] <= !block_minv_phase[0];")
bw.writeLine("")
bw.writeLine("    if (idx_block_minv_feed == len_block_minv_sched_total-1) begin")
bw.writeLine("        $display(\"Fed last minv tile\");")
bw.writeLine("        idx_block_minv_feed <= 0;")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        idx_block_minv_feed <= idx_block_minv_feed + 1;")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("    Bit#(32) idx_block_minv_feed_dtau;")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("    Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Bit#(32))) dtau_mat_curr;")
bw.writeLine("")
bw.writeLine("    if (idx_block_minv_feed < len_block_minv_sched_per_matrix) begin")
bw.writeLine("        // DTAUDQ")
bw.writeLine("        idx_block_minv_feed_dtau = idx_block_minv_feed;")
bw.writeLine("        dtau_mat_curr = dtaudq;")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        // DTAUDQD")
bw.writeLine("        idx_block_minv_feed_dtau = idx_block_minv_feed - len_block_minv_sched_per_matrix;")
bw.writeLine("        dtau_mat_curr = dtaudqd;")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("    $display(\"MINV BLOCK IN\");")
bw.writeLine("//    for (Bit#(4) i=0; i<7; i=i+1) begin")
bw.writeLine("//        $display(fshow(minv[lh_tile_row_start+i][rh_tile_col_start+0]), \" \", fshow(minv[lh_tile_row_start+i][rh_tile_col_start+1]), \" \", fshow(minv[lh_tile_row_start+i][rh_tile_col_start+2]), \" \", fshow(minv[lh_tile_row_start+i][rh_tile_col_start+3]), \" \", fshow(minv[lh_tile_row_start+i][rh_tile_col_start+4]), \" \", fshow(minv[lh_tile_row_start+i][rh_tile_col_start+5]), \" \", fshow(minv[lh_tile_row_start+i][rh_tile_col_start+6]));")
bw.writeLine("//    end")

bw.writeLine("")
bw.writeLine("    // MINV INPUT")
bw.writeLine("    bproc.get_data_minv();")
bw.writeLine("")
bw.writeLine("`include \"GradientPipelineBprocMatmulInputUnrolling.bsv\"");
bw.writeLine("")
bw.writeLine("endrule")

bw.writeLine("")
bw.writeLine("rule streamBlockMinv (mul_mat[1] && unpack(bproc.output_ready_minv()) && !block_minv_phase[1] && idx_block_minv_stream < len_block_minv_sched_total);")
bw.writeLine("    $display(\"ENTERING STREAM BLOCK MINV\");")
bw.writeLine("")
bw.writeLine("    block_minv_phase[1] <= !block_minv_phase[1];")
bw.writeLine("")
bw.writeLine("    if (idx_block_minv_stream == len_block_minv_sched_total-1) begin")
bw.writeLine("        intermediate_values.deq(); //finished processing that block")
bw.writeLine("        trigo_values.deq(); //finished processing that block")
bw.writeLine("")
bw.writeLine("        $display(\"completed entire matmul\");")
bw.writeLine("        mul_mat[1] <= !mul_mat[1];")
bw.writeLine("        idx_block_minv_stream <= 0;")
bw.writeLine("")
bw.writeLine("        //// branching case")

for i in range(num_branches):
    for p in range(num_PEs):
        PE_id = p+1
        bw.writeLine("        df_upd_branch[{PE_id}][{branch_i_incr}] <= DfUpdIntermediate {{dfdqd_upd: unpack(0), dfdq_upd: unpack(0)}};".format(
                PE_id=PE_id,
                branch_i_incr=i+1
        ))
bw.writeLine("")

for p in range(num_PEs):
    PE_id = p+1
    bw.writeLine("        df_upd_prev[{PE_id}] <= DfUpdIntermediate {{dfdqd_upd: unpack(0), dfdq_upd: unpack(0)}};".format(
            PE_id=PE_id
    ))
bw.writeLine("")

for i in range(max_link_ref):
    bw.writeLine("        f_upd_acc[{i}] <= FUpdIntermediate {{f_upd: unpack(0)}};".format(i=i))
bw.writeLine("    end")

bw.writeLine("    else begin")
bw.writeLine("        idx_block_minv_stream <= idx_block_minv_stream + 1;")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("    Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Bit#(32))) new_minv_dtau;")
bw.writeLine("")
bw.writeLine("    Bit#(32) idx_block_minv_stream_dtau;")
bw.writeLine("    if (idx_block_minv_stream < len_block_minv_sched_per_matrix) begin")
bw.writeLine("        idx_block_minv_stream_dtau = idx_block_minv_stream;")
bw.writeLine("        new_minv_dtau = minv_dtaudq;")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        idx_block_minv_stream_dtau = idx_block_minv_stream - len_block_minv_sched_per_matrix;")
bw.writeLine("        new_minv_dtau = minv_dtaudqd;")
bw.writeLine("    end")

bw.writeLine("")

for p in range(num_PEs):
    PE_id = p+1
    for i in range(block_size):
        bw.writeLine("    let minv_vec_out_R{r}_dqdPE{PE_id} = bproc.minv_vec_out_R{r}_dqdPE{PE_id}();".format(PE_id=PE_id, r=i+1))
    bw.writeLine("")

bw.writeLine("//    $display(\"MINV VEC OUT BLOCK\");")
bw.writeLine("//    $display(fshow(minv_vec_out_R1_dqdPE1), \" \", fshow(minv_vec_out_R1_dqdPE2), \" \", fshow(minv_vec_out_R1_dqdPE3), \" \", fshow(minv_vec_out_R1_dqdPE4), \" \", fshow(minv_vec_out_R1_dqdPE5), \" \", fshow(minv_vec_out_R1_dqdPE6), \" \", fshow(minv_vec_out_R1_dqdPE7));")
bw.writeLine("//    $display(fshow(minv_vec_out_R2_dqdPE1), \" \", fshow(minv_vec_out_R2_dqdPE2), \" \", fshow(minv_vec_out_R2_dqdPE3), \" \", fshow(minv_vec_out_R2_dqdPE4), \" \", fshow(minv_vec_out_R2_dqdPE5), \" \", fshow(minv_vec_out_R2_dqdPE6), \" \", fshow(minv_vec_out_R2_dqdPE7));")
bw.writeLine("//    $display(fshow(minv_vec_out_R3_dqdPE1), \" \", fshow(minv_vec_out_R3_dqdPE2), \" \", fshow(minv_vec_out_R3_dqdPE3), \" \", fshow(minv_vec_out_R3_dqdPE4), \" \", fshow(minv_vec_out_R3_dqdPE5), \" \", fshow(minv_vec_out_R3_dqdPE6), \" \", fshow(minv_vec_out_R3_dqdPE7));")
bw.writeLine("//    $display(fshow(minv_vec_out_R4_dqdPE1), \" \", fshow(minv_vec_out_R4_dqdPE2), \" \", fshow(minv_vec_out_R4_dqdPE3), \" \", fshow(minv_vec_out_R4_dqdPE4), \" \", fshow(minv_vec_out_R4_dqdPE5), \" \", fshow(minv_vec_out_R4_dqdPE6), \" \", fshow(minv_vec_out_R4_dqdPE7));")
bw.writeLine("//    $display(fshow(minv_vec_out_R5_dqdPE1), \" \", fshow(minv_vec_out_R5_dqdPE2), \" \", fshow(minv_vec_out_R5_dqdPE3), \" \", fshow(minv_vec_out_R5_dqdPE4), \" \", fshow(minv_vec_out_R5_dqdPE5), \" \", fshow(minv_vec_out_R5_dqdPE6), \" \", fshow(minv_vec_out_R5_dqdPE7));")
bw.writeLine("//    $display(fshow(minv_vec_out_R6_dqdPE1), \" \", fshow(minv_vec_out_R6_dqdPE2), \" \", fshow(minv_vec_out_R6_dqdPE3), \" \", fshow(minv_vec_out_R6_dqdPE4), \" \", fshow(minv_vec_out_R6_dqdPE5), \" \", fshow(minv_vec_out_R6_dqdPE6), \" \", fshow(minv_vec_out_R6_dqdPE7));")
bw.writeLine("//    $display(fshow(minv_vec_out_R7_dqdPE1), \" \", fshow(minv_vec_out_R7_dqdPE2), \" \", fshow(minv_vec_out_R7_dqdPE3), \" \", fshow(minv_vec_out_R7_dqdPE4), \" \", fshow(minv_vec_out_R7_dqdPE5), \" \", fshow(minv_vec_out_R7_dqdPE6), \" \", fshow(minv_vec_out_R7_dqdPE7));")
bw.writeLine("")

bw.writeLine("`include \"GradientPipelineBprocMatmulOutputUnrolling.bsv\"")

bw.writeLine("")
bw.writeLine("    if (idx_block_minv_stream == len_block_minv_sched_total-1) begin")
bw.writeLine("        let new_minv_dtaudq = minv_dtaudq;")
bw.writeLine("        let new_minv_dtaudqd = new_minv_dtau;")
bw.writeLine("")
bw.writeLine("        // flush minv_dtaudq matrices for next knot point")
bw.writeLine("        minv_dtaudq <= unpack(0);")
bw.writeLine("        minv_dtaudqd <= unpack(0);")
bw.writeLine("")
bw.writeLine("        // PRINT MINV_DTAUDQ")
bw.writeLine("        $display(\"MINV_DTAUDQ OUTPUT:\");")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLine("            $display(fshow(new_minv_dtaudq[i]));")
bw.writeLine("        end")
bw.writeLine("        // PRINT MINV_DTAUDQD")
bw.writeLine("        $display(\"MINV_DTAUDQD OUTPUT:\");")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLine("            $display(fshow(new_minv_dtaudqd[i]));")
bw.writeLine("        end")
bw.writeLine("")

n_entries_matrices = num_links*num_links * 2
bw.writeLine("        Vector#({n_entries_matrices}, Bit#(32)) result_vec;".format(n_entries_matrices=n_entries_matrices))

bw.writeLine("")
bw.writeLine("        // see earlier versions to see how result_vec is packed")
bw.writeLine("        //result_vec = vec(")
bw.writeLine("       //   minv_dtaudq_out_R7_C1, minv_dtaudq_out_R7_C2, minv_dtaudq_out_R7_C3, minv_dtaudq_out_R7_C4, minv_dtaudq_out_R7_C5, minv_dtaudq_out_R7_C6, minv_dtaudq_out_R7_C7,")
bw.writeLine("        //   minv_dtaudqd_out_R7_C1, minv_dtaudqd_out_R7_C2, minv_dtaudqd_out_R7_C3, minv_dtaudqd_out_R7_C4, minv_dtaudqd_out_R7_C5, minv_dtaudqd_out_R7_C6, minv_dtaudqd_out_R7_C7,")
bw.writeLine("        //   minv_dtaudq_out_R6_C1, minv_dtaudq_out_R6_C2, minv_dtaudq_out_R6_C3, minv_dtaudq_out_R6_C4, minv_dtaudq_out_R6_C5, minv_dtaudq_out_R6_C6, minv_dtaudq_out_R6_C7,")
bw.writeLine("        //   minv_dtaudqd_out_R6_C1, minv_dtaudqd_out_R6_C2, minv_dtaudqd_out_R6_C3, minv_dtaudqd_out_R6_C4, minv_dtaudqd_out_R6_C5, minv_dtaudqd_out_R6_C6, minv_dtaudqd_out_R6_C7,")
bw.writeLine("        // ...")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLineWithFormatSpecReplace("            for (int j=0; j<{num_links}; j=j+1) begin")
bw.writeLineWithFormatSpecReplace("                result_vec[i*2*{num_links} + j] = new_minv_dtaudq[{num_links}-i-1][j];")
bw.writeLine("            end")
bw.writeLineWithFormatSpecReplace("            for (int j=0; j<{num_links}; j=j+1) begin")
bw.writeLineWithFormatSpecReplace("                result_vec[i*2*{num_links} + j+{num_links}] = new_minv_dtaudqd[{num_links}-i-1][j];")
bw.writeLine("            end")
bw.writeLine("        end")
bw.writeLine("")
bw.writeLine("        count_interm[0] <= count_interm[0] - 1;")
bw.writeLine("        out_data.enq(pack(result_vec));")
bw.writeLine("")
bw.writeLine("        $display(\"========== MATMUL RESULTS END ==========\");")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        if (idx_block_minv_stream < len_block_minv_sched_per_matrix) begin")
bw.writeLine("            minv_dtaudq <= new_minv_dtau;")
bw.writeLine("        end")
bw.writeLine("        else begin")
bw.writeLine("            minv_dtaudqd <= new_minv_dtau;")
bw.writeLine("        end")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("endrule")
