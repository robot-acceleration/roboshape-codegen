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

fm = FileManager("GradientPipelineFprocRules.bsv")
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


bw.writeLine("// parameterize by len link sched")
bw.writeLine("Reg#(Bit#(32)) idx_forward_feed <- mkReg(0);")
bw.writeLine("Reg#(Bit#(32)) idx_forward_stream <- mkReg(0);")
bw.writeLine("")
bw.writeLine("Reg#(Bool) keepgoing <- mkReg(True);")
bw.writeLine("Ehr#(2,Bit#(8)) count_interm <- mkEhr(0);")
bw.writeLine("")
bw.writeLine("//*******************************/")
bw.writeLine("//   SCHEDULE TABLES FPROC       /")
bw.writeLine("//*******************************/")
bw.writeLine("")

###
bw.writeLine("Bit#(32) len_fproc_sched = {len_fproc_sched};".format(len_fproc_sched=len_fproc_sched))
bw.writeLine("")

bw.writeLine("Vector#({len_fproc_sched}, Bit#({num_links_bitwidth})) rnea_fproc_curr_sched = ".format(
    len_fproc_sched=len_fproc_sched,
    num_links_bitwidth=num_links_bitwidth
))
bw.writeLine(bw.get_row_vec_bluespec("    ", rnea_fproc_curr_sched) + ";")
bw.writeLine("")

bw.writeLine("Vector#({len_fproc_sched}, Bit#({num_links_bitwidth})) rnea_fproc_par_sched = ".format(
    len_fproc_sched=len_fproc_sched,
    num_links_bitwidth=num_links_bitwidth
))
bw.writeLine(bw.get_row_vec_bluespec("    ", rnea_fproc_par_sched) + ";")
bw.writeLine("")

bw.writeLine("Vector#({len_fproc_sched}, Vector#({num_PEs}, Bit#({num_links_bitwidth}))) fproc_curr_sched = vec(".format(
    len_fproc_sched=len_fproc_sched,
    num_PEs=num_PEs,
    num_links_bitwidth=num_links_bitwidth
))
for i,row in enumerate(fproc_curr_sched):
    ending = "" if i == len_fproc_sched-1 else ","
    bw.writeLine(bw.get_row_vec_bluespec("    ", row) + ending)
bw.writeLine(");")

bw.writeLine("Vector#({len_fproc_sched}, Vector#({num_PEs}, Bit#({num_links_bitwidth}))) fproc_par_sched = vec(".format(
    len_fproc_sched=len_fproc_sched,
    num_PEs=num_PEs,
    num_links_bitwidth=num_links_bitwidth
))
for i,row in enumerate(fproc_par_sched):
    ending = "" if i == len_fproc_sched-1 else ","
    bw.writeLine(bw.get_row_vec_bluespec("    ", row) + ending)
bw.writeLine(");")
bw.writeLine("")

bw.writeLine("Vector#({len_fproc_sched}, Vector#({num_PEs}, Bit#({num_links_bitwidth}))) fproc_derv_sched = vec(".format(
    len_fproc_sched=len_fproc_sched,
    num_PEs=num_PEs,
    num_links_bitwidth=num_links_bitwidth
))
for i,row in enumerate(fproc_derv_sched):
    ending = "" if i == len_fproc_sched-1 else ","
    bw.writeLine(bw.get_row_vec_bluespec("    ", row) + ending)
bw.writeLine(");")
bw.writeLine("")
###

bw.writeLine("// for fproc ext branching")
bw.writeLine("// both 1-indexed")
bw.writeLine("// s/8/NUM_PES+1")
bw.writeLine("Vector#({num_PEs_incr},Reg#(DvDaIntermediate)) dvda_prev <- replicateM(mkReg(unpack(0)));".format(num_PEs_incr=num_PEs_incr))
bw.writeLine("// s/1/NUM_BRANCHES+1")
bw.writeLine("// s/8/NUM_PES+1")
bw.writeLine("Vector#({num_PEs_incr},Vector#({num_branches_incr},Reg#(DvDaIntermediate))) dvda_branch <- replicateM(replicateM(mkReg(unpack(0))));".format(
        num_PEs_incr=num_PEs_incr,
        num_branches_incr=num_branches_incr
))
bw.writeLine("")

###
bw.writeLine("// returns the index of dvda_branch/upd_branch where the dvda/upd for that link was stored")
bw.writeLine("// for later use")
bw.writeLine("function Bit#({num_links_bitwidth}) branchTableFproc(Bit#({num_links_bitwidth}) branch_link);".format(num_links_bitwidth=num_links_bitwidth))
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

bw.writeLine("function DvDaIntermediate selectDvDaBranch(Bit#({num_links_bitwidth}) curr_link, Bit#({num_links_bitwidth}) par_link, Bit#({num_links_bitwidth}) curr_PE);".format(num_links_bitwidth=num_links_bitwidth))
bw.writeLine("    if (par_link != curr_link-1) begin")
bw.writeLine("        return dvda_branch[curr_PE][branchTableFproc(par_link)];")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        return dvda_prev[curr_PE];")
bw.writeLine("    end")
bw.writeLine("endfunction")
bw.writeLine("")
###

bw.writeLine("//*******************************/")
bw.writeLine("")
bw.writeLine("// There cannot be backpressure.")
bw.writeLine("rule feedFproc( !phase[1] && idx_forward_feed < len_fproc_sched);")
bw.writeLine("    $display(\"=========== FPROC FEED for idx \", idx_forward_feed, \" =============\");")
bw.writeLine("")
bw.writeLine("    phase[1] <= !phase[1];")
bw.writeLine("    let complete_input = inputs.first();")
bw.writeLine("    $display(\"Feed forward at idx: \", idx_forward_feed);")
bw.writeLine("    //$display(\"Feed forward data: \", fshow(complete_input[idx_forward_feed]));")
bw.writeLine("    $display(\"tic feed: %d\", cyc_count);")
bw.writeLine("")
bw.writeLine("    // Getting link ids for different components of the schedule")
bw.writeLine("    let link_in_curr_rnea = rnea_fproc_curr_sched[idx_forward_feed];")
bw.writeLine("    let link_in_par_rnea = rnea_fproc_par_sched[idx_forward_feed];")

###

#for i in range(num_PEs):
#    bw.writeLine("    let link_in_curr_PE{PE_id} = fproc_curr_sched[idx_forward_feed][{i}];".format(PE_id=i+1, i=i))
#
#for i in range(num_PEs):
#    bw.writeLine("    let link_in_par_PE{PE_id} = fproc_par_sched[idx_forward_feed][{i}];".format(PE_id=i+1, i=i))
#
#for i in range(num_PEs):
#    bw.writeLine("    let link_in_derv_PE{PE_id} = fproc_derv_sched[idx_forward_feed][{i}];".format(PE_id=i+1, i=i))
for i in range(num_PEs):
    bw.writeLine("    Bit#({num_links_bitwidth}) link_in_curr_PE{PE_id} = 0;".format(
            num_links_bitwidth=num_links_bitwidth, PE_id=i+1)
    )
for i in range(num_PEs):
    bw.writeLine("    Bit#({num_links_bitwidth}) link_in_par_PE{PE_id} = 0;".format(
            num_links_bitwidth=num_links_bitwidth, PE_id=i+1)
    )
for i in range(num_PEs):
    bw.writeLine("    Bit#({num_links_bitwidth}) link_in_derv_PE{PE_id} = 0;".format(
            num_links_bitwidth=num_links_bitwidth, PE_id=i+1)
    )

bw.writeLine("")
bw.writeLine("`include \"GradientPipelineFprocSchedUnrolling.bsv\"")

bw.writeLine("    ")
bw.writeLine("    // grabbing the input values are 1-indexed")
bw.writeLine("    let input_curr_rnea = complete_input[link_in_curr_rnea];")
bw.writeLine("    let input_par_rnea = complete_input[link_in_par_rnea];")

for i in range(num_PEs):
    bw.writeLine("    let input_curr_PE{PE_id} = complete_input[link_in_curr_PE{PE_id}];".format(PE_id=i+1))
bw.writeLine("")

bw.writeLine("    // external branches")
for i in range(num_PEs):
    bw.writeLine("    let dvda_PE{PE_id} = selectDvDaBranch(link_in_curr_PE{PE_id},link_in_par_PE{PE_id},{PE_id});".format(PE_id=i+1))
bw.writeLine("")

###
bw.writeLine("")
bw.writeLine("    //-----------------------------------")
bw.writeLine("")
bw.writeLine("    fproc.get_data();")
bw.writeLine("")
bw.writeLine("    //------- RNEA INPUTS ---------")
bw.writeLine("")
bw.writeLine("    fproc.link_in_rnea(link_in_curr_rnea);")
bw.writeLine("    fproc.sinq_val_in_rnea(input_curr_rnea.sinq);")
bw.writeLine("    fproc.cosq_val_in_rnea(input_curr_rnea.cosq);")
bw.writeLine("    fproc.qd_val_in_rnea(input_curr_rnea.qd);")
bw.writeLine("    fproc.qdd_val_in_rnea(input_curr_rnea.qdd);")
bw.writeLine("    fproc.v_prev_vec_in_AX_rnea(rnea_acc[link_in_par_rnea].v[0]);")
bw.writeLine("    fproc.v_prev_vec_in_AY_rnea(rnea_acc[link_in_par_rnea].v[1]);")
bw.writeLine("    fproc.v_prev_vec_in_AZ_rnea(rnea_acc[link_in_par_rnea].v[2]);")
bw.writeLine("    fproc.v_prev_vec_in_LX_rnea(rnea_acc[link_in_par_rnea].v[3]);")
bw.writeLine("    fproc.v_prev_vec_in_LY_rnea(rnea_acc[link_in_par_rnea].v[4]);")
bw.writeLine("    fproc.v_prev_vec_in_LZ_rnea(rnea_acc[link_in_par_rnea].v[5]);")
bw.writeLine("    fproc.a_prev_vec_in_AX_rnea(rnea_acc[link_in_par_rnea].a[0]);")
bw.writeLine("    fproc.a_prev_vec_in_AY_rnea(rnea_acc[link_in_par_rnea].a[1]);")
bw.writeLine("    fproc.a_prev_vec_in_AZ_rnea(rnea_acc[link_in_par_rnea].a[2]);")
bw.writeLine("    fproc.a_prev_vec_in_LX_rnea(rnea_acc[link_in_par_rnea].a[3]);")
bw.writeLine("    fproc.a_prev_vec_in_LY_rnea(rnea_acc[link_in_par_rnea].a[4]);")
bw.writeLine("    fproc.a_prev_vec_in_LZ_rnea(rnea_acc[link_in_par_rnea].a[5]);")
bw.writeLine("")
bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    //------- DQ INPUTS -----------")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    fproc.link_in_dqPE{PE_id}(link_in_curr_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    fproc.derv_in_dqPE{PE_id}(link_in_derv_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    fproc.sinq_val_in_dqPE{PE_id}(input_curr_PE{PE_id}.sinq);".format(PE_id=PE_id))
    bw.writeLine("    fproc.cosq_val_in_dqPE{PE_id}(input_curr_PE{PE_id}.cosq);".format(PE_id=PE_id))
    bw.writeLine("    fproc.qd_val_in_dqPE{PE_id}(input_curr_PE{PE_id}.qd);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_AX_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_AY_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_AZ_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_LX_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_LY_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_LZ_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_AX_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_AY_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_AZ_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_LX_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_LY_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_LZ_dqPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_prev_vec_in_AX_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].v[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_prev_vec_in_AY_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].v[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_prev_vec_in_AZ_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].v[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_prev_vec_in_LX_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].v[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_prev_vec_in_LY_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].v[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_prev_vec_in_LZ_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].v[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_prev_vec_in_AX_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].a[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_prev_vec_in_AY_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].a[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_prev_vec_in_AZ_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].a[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_prev_vec_in_LX_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].a[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_prev_vec_in_LY_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].a[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_prev_vec_in_LZ_dqPE{PE_id}(rnea_acc[link_in_par_PE{PE_id}].a[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdq_prev_vec_in_AX_dqPE{PE_id}(dvda_PE{PE_id}.dvdq[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdq_prev_vec_in_AY_dqPE{PE_id}(dvda_PE{PE_id}.dvdq[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdq_prev_vec_in_AZ_dqPE{PE_id}(dvda_PE{PE_id}.dvdq[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdq_prev_vec_in_LX_dqPE{PE_id}(dvda_PE{PE_id}.dvdq[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdq_prev_vec_in_LY_dqPE{PE_id}(dvda_PE{PE_id}.dvdq[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdq_prev_vec_in_LZ_dqPE{PE_id}(dvda_PE{PE_id}.dvdq[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadq_prev_vec_in_AX_dqPE{PE_id}(dvda_PE{PE_id}.dadq[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadq_prev_vec_in_AY_dqPE{PE_id}(dvda_PE{PE_id}.dadq[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadq_prev_vec_in_AZ_dqPE{PE_id}(dvda_PE{PE_id}.dadq[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadq_prev_vec_in_LX_dqPE{PE_id}(dvda_PE{PE_id}.dadq[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadq_prev_vec_in_LY_dqPE{PE_id}(dvda_PE{PE_id}.dadq[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadq_prev_vec_in_LZ_dqPE{PE_id}(dvda_PE{PE_id}.dadq[5]);".format(PE_id=PE_id))
    bw.writeLine("")

bw.writeLine("")
bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    //------- DQD INPUTS ----------")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    fproc.link_in_dqdPE{PE_id}(link_in_curr_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    fproc.derv_in_dqdPE{PE_id}(link_in_derv_PE{PE_id});".format(PE_id=PE_id))
    bw.writeLine("    fproc.sinq_val_in_dqdPE{PE_id}(input_curr_PE{PE_id}.sinq);".format(PE_id=PE_id))
    bw.writeLine("    fproc.cosq_val_in_dqdPE{PE_id}(input_curr_PE{PE_id}.cosq);".format(PE_id=PE_id))
    bw.writeLine("    fproc.qd_val_in_dqdPE{PE_id}(input_curr_PE{PE_id}.qd);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_AX_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_AY_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_AZ_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_LX_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_LY_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.v_curr_vec_in_LZ_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].v[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_AX_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_AY_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_AZ_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_LX_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_LY_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.a_curr_vec_in_LZ_dqdPE{PE_id}(rnea_acc[link_in_curr_PE{PE_id}].a[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdqd_prev_vec_in_AX_dqdPE{PE_id}(dvda_PE{PE_id}.dvdqd[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdqd_prev_vec_in_AY_dqdPE{PE_id}(dvda_PE{PE_id}.dvdqd[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdqd_prev_vec_in_AZ_dqdPE{PE_id}(dvda_PE{PE_id}.dvdqd[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdqd_prev_vec_in_LX_dqdPE{PE_id}(dvda_PE{PE_id}.dvdqd[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdqd_prev_vec_in_LY_dqdPE{PE_id}(dvda_PE{PE_id}.dvdqd[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dvdqd_prev_vec_in_LZ_dqdPE{PE_id}(dvda_PE{PE_id}.dvdqd[5]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadqd_prev_vec_in_AX_dqdPE{PE_id}(dvda_PE{PE_id}.dadqd[0]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadqd_prev_vec_in_AY_dqdPE{PE_id}(dvda_PE{PE_id}.dadqd[1]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadqd_prev_vec_in_AZ_dqdPE{PE_id}(dvda_PE{PE_id}.dadqd[2]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadqd_prev_vec_in_LX_dqdPE{PE_id}(dvda_PE{PE_id}.dadqd[3]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadqd_prev_vec_in_LY_dqdPE{PE_id}(dvda_PE{PE_id}.dadqd[4]);".format(PE_id=PE_id))
    bw.writeLine("    fproc.dadqd_prev_vec_in_LZ_dqdPE{PE_id}(dvda_PE{PE_id}.dadqd[5]);".format(PE_id=PE_id))
    bw.writeLine("")

bw.writeLine("")
bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    // trigo_acc is 1-indexed")
bw.writeLine("    trigo_acc[link_in_curr_rnea] <= Trigo{")
bw.writeLine("                    sinq: input_curr_rnea.sinq,")
bw.writeLine("                    cosq: input_curr_rnea.cosq,")
bw.writeLine("                    minv: input_curr_rnea.minv")
bw.writeLine("    };")
bw.writeLine("")
bw.writeLine("    //-----------------------------")
bw.writeLine("")
bw.writeLine("    if (idx_forward_feed == len_fproc_sched-1) begin")
bw.writeLine("        keepgoing <= False;")
bw.writeLine("        inputs.deq();")
bw.writeLine("        count_interm[1] <= count_interm[1] + 1;")
bw.writeLine("        Vector#({max_link_incr}, Trigo) embed; // 1-indexed + we need zeros as input for 8th link".format(max_link_incr=max_link_incr))
bw.writeLine("        // s/7/num_links")
bw.writeLine("        embed[0] = unpack(0);")
bw.writeLine("        for (int i = 1; i <= {num_links}; i = i + 1) begin".format(num_links=num_links))
bw.writeLine("            embed[i] = trigo_acc[i]; // trigo_acc is 1-indexed")
bw.writeLine("        end")
bw.writeLine("        embed[{max_link_ref}] = unpack(0);".format(max_link_ref=max_link_ref))
bw.writeLine("        trigo_values.enq(embed);")
bw.writeLine("")
bw.writeLine("        // just for minv")
bw.writeLineWithFormatSpecReplace("        for (int i=0; i<{num_links}; i=i+1) begin")
bw.writeLineWithFormatSpecReplace("            for (int j=0; j<{num_links}; j=j+1) begin")
bw.writeLine("                minv[i][j] <= trigo_acc[i+1].minv[j];")
bw.writeLine("            end")
bw.writeLine("        end")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        idx_forward_feed <= idx_forward_feed + 1;")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("    $display(\"=========== FPROC FEED for idx \", idx_forward_feed, \" END =============\");")
bw.writeLine("")
bw.writeLine("endrule")

bw.writeLine("")
bw.writeLine("rule restartPipe (!keepgoing && count_interm[1] < 3);")
bw.writeLine("    keepgoing <= True;")
bw.writeLine("    if (idx_forward_feed == len_fproc_sched-1) begin")
bw.writeLine("        idx_forward_feed <= 0;")
bw.writeLine("    end")
bw.writeLine("endrule")
bw.writeLine("")
bw.writeLine("// s/8/len_sched")
bw.writeLine("rule streamFproc( phase[0] && unpack(fproc.output_ready()) && idx_forward_stream < len_fproc_sched);")
bw.writeLine("    $display(\"=========== FPROC STREAM for idx \", idx_forward_stream, \" =============\");")
bw.writeLine("")
bw.writeLine("    phase[0] <= !phase[0];")
bw.writeLine("    $display(\"tic stream: %d\", cyc_count);")
bw.writeLine("")
bw.writeLine("    //******** RNEA OUTPUT REDIRECTION *******/")
bw.writeLine("")
bw.writeLine("    let link_out_curr_rnea = rnea_fproc_curr_sched[idx_forward_stream];")

bw.writeLine("    //$display(\"curr link outs: rnea, PE1, PE2...PE7: \", link_out_curr_rnea, link_out_curr_PE1,link_out_curr_PE2,link_out_curr_PE3,link_out_curr_PE4,link_out_curr_PE5,link_out_curr_PE6,link_out_curr_PE7);")
bw.writeLine("")
bw.writeLine("    let v_curr_vec_out_AX_rnea = fproc.v_curr_vec_out_AX_rnea();")
bw.writeLine("    let v_curr_vec_out_AY_rnea = fproc.v_curr_vec_out_AY_rnea();")
bw.writeLine("    let v_curr_vec_out_AZ_rnea = fproc.v_curr_vec_out_AZ_rnea();")
bw.writeLine("    let v_curr_vec_out_LX_rnea = fproc.v_curr_vec_out_LX_rnea();")
bw.writeLine("    let v_curr_vec_out_LY_rnea = fproc.v_curr_vec_out_LY_rnea();")
bw.writeLine("    let v_curr_vec_out_LZ_rnea = fproc.v_curr_vec_out_LZ_rnea();")
bw.writeLine("    let a_curr_vec_out_AX_rnea = fproc.a_curr_vec_out_AX_rnea();")
bw.writeLine("    let a_curr_vec_out_AY_rnea = fproc.a_curr_vec_out_AY_rnea();")
bw.writeLine("    let a_curr_vec_out_AZ_rnea = fproc.a_curr_vec_out_AZ_rnea();")
bw.writeLine("    let a_curr_vec_out_LX_rnea = fproc.a_curr_vec_out_LX_rnea();")
bw.writeLine("    let a_curr_vec_out_LY_rnea = fproc.a_curr_vec_out_LY_rnea();")
bw.writeLine("    let a_curr_vec_out_LZ_rnea = fproc.a_curr_vec_out_LZ_rnea();")
bw.writeLine("    let f_curr_vec_out_AX_rnea = fproc.f_curr_vec_out_AX_rnea();")
bw.writeLine("    let f_curr_vec_out_AY_rnea = fproc.f_curr_vec_out_AY_rnea();")
bw.writeLine("    let f_curr_vec_out_AZ_rnea = fproc.f_curr_vec_out_AZ_rnea();")
bw.writeLine("    let f_curr_vec_out_LX_rnea = fproc.f_curr_vec_out_LX_rnea();")
bw.writeLine("    let f_curr_vec_out_LY_rnea = fproc.f_curr_vec_out_LY_rnea();")
bw.writeLine("    let f_curr_vec_out_LZ_rnea = fproc.f_curr_vec_out_LZ_rnea();")
bw.writeLine("")
bw.writeLine("    //***************************************/")
bw.writeLine("")
bw.writeLine("    //********* DQPE OUTPUT REDIRECTION ********/")
bw.writeLine("    ")

###
for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dfdq_curr_vec_out_AX_dqPE{PE_id} = fproc.dfdq_curr_vec_out_AX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_curr_vec_out_AY_dqPE{PE_id} = fproc.dfdq_curr_vec_out_AY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_curr_vec_out_AZ_dqPE{PE_id} = fproc.dfdq_curr_vec_out_AZ_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_curr_vec_out_LX_dqPE{PE_id} = fproc.dfdq_curr_vec_out_LX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_curr_vec_out_LY_dqPE{PE_id} = fproc.dfdq_curr_vec_out_LY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdq_curr_vec_out_LZ_dqPE{PE_id} = fproc.dfdq_curr_vec_out_LZ_dqPE{PE_id}();".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dvdq_curr_vec_out_AX_dqPE{PE_id} = fproc.dvdq_curr_vec_out_AX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdq_curr_vec_out_AY_dqPE{PE_id} = fproc.dvdq_curr_vec_out_AY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdq_curr_vec_out_AZ_dqPE{PE_id} = fproc.dvdq_curr_vec_out_AZ_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdq_curr_vec_out_LX_dqPE{PE_id} = fproc.dvdq_curr_vec_out_LX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdq_curr_vec_out_LY_dqPE{PE_id} = fproc.dvdq_curr_vec_out_LY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdq_curr_vec_out_LZ_dqPE{PE_id} = fproc.dvdq_curr_vec_out_LZ_dqPE{PE_id}();".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dadq_curr_vec_out_AX_dqPE{PE_id} = fproc.dadq_curr_vec_out_AX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadq_curr_vec_out_AY_dqPE{PE_id} = fproc.dadq_curr_vec_out_AY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadq_curr_vec_out_AZ_dqPE{PE_id} = fproc.dadq_curr_vec_out_AZ_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadq_curr_vec_out_LX_dqPE{PE_id} = fproc.dadq_curr_vec_out_LX_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadq_curr_vec_out_LY_dqPE{PE_id} = fproc.dadq_curr_vec_out_LY_dqPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadq_curr_vec_out_LZ_dqPE{PE_id} = fproc.dadq_curr_vec_out_LZ_dqPE{PE_id}();".format(PE_id=PE_id))
bw.writeLine("")
###

bw.writeLine("")
bw.writeLine("    //***************************************/")
bw.writeLine("")
bw.writeLine("    //********* DQDPE OUTPUT REDIRECTION ********/")
bw.writeLine("")

###
for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dfdqd_curr_vec_out_AX_dqdPE{PE_id} = fproc.dfdqd_curr_vec_out_AX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_curr_vec_out_AY_dqdPE{PE_id} = fproc.dfdqd_curr_vec_out_AY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_curr_vec_out_AZ_dqdPE{PE_id} = fproc.dfdqd_curr_vec_out_AZ_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_curr_vec_out_LX_dqdPE{PE_id} = fproc.dfdqd_curr_vec_out_LX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_curr_vec_out_LY_dqdPE{PE_id} = fproc.dfdqd_curr_vec_out_LY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dfdqd_curr_vec_out_LZ_dqdPE{PE_id} = fproc.dfdqd_curr_vec_out_LZ_dqdPE{PE_id}();".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dvdqd_curr_vec_out_AX_dqdPE{PE_id} = fproc.dvdqd_curr_vec_out_AX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdqd_curr_vec_out_AY_dqdPE{PE_id} = fproc.dvdqd_curr_vec_out_AY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdqd_curr_vec_out_AZ_dqdPE{PE_id} = fproc.dvdqd_curr_vec_out_AZ_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdqd_curr_vec_out_LX_dqdPE{PE_id} = fproc.dvdqd_curr_vec_out_LX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdqd_curr_vec_out_LY_dqdPE{PE_id} = fproc.dvdqd_curr_vec_out_LY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dvdqd_curr_vec_out_LZ_dqdPE{PE_id} = fproc.dvdqd_curr_vec_out_LZ_dqdPE{PE_id}();".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    let dadqd_curr_vec_out_AX_dqdPE{PE_id} = fproc.dadqd_curr_vec_out_AX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadqd_curr_vec_out_AY_dqdPE{PE_id} = fproc.dadqd_curr_vec_out_AY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadqd_curr_vec_out_AZ_dqdPE{PE_id} = fproc.dadqd_curr_vec_out_AZ_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadqd_curr_vec_out_LX_dqdPE{PE_id} = fproc.dadqd_curr_vec_out_LX_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadqd_curr_vec_out_LY_dqdPE{PE_id} = fproc.dadqd_curr_vec_out_LY_dqdPE{PE_id}();".format(PE_id=PE_id))
    bw.writeLine("    let dadqd_curr_vec_out_LZ_dqdPE{PE_id} = fproc.dadqd_curr_vec_out_LZ_dqdPE{PE_id}();".format(PE_id=PE_id))
bw.writeLine("")
###

bw.writeLine("")
bw.writeLine("    //***********************************")
bw.writeLine("")
bw.writeLine("    let new_f_inter_acc = f_inter_acc;")
bw.writeLine("    let new_dfidq_inter_acc = dfidq_inter_acc;")
bw.writeLine("    let new_dfidqd_inter_acc = dfidqd_inter_acc;")
bw.writeLine("")

bw.writeLine("`include \"GradientPipelineFprocIntermediateMuxUnrolling.bsv\"")

bw.writeLine("")
bw.writeLine("    //**********************************")
bw.writeLine("")
bw.writeLine("    Vector#(6,Bit#(32)) vec_f = vec(f_curr_vec_out_AX_rnea, f_curr_vec_out_AY_rnea, f_curr_vec_out_AZ_rnea, f_curr_vec_out_LX_rnea, f_curr_vec_out_LY_rnea, f_curr_vec_out_LZ_rnea);")
bw.writeLine("    Vector#(6,Bit#(32)) vec_a = vec(a_curr_vec_out_AX_rnea, a_curr_vec_out_AY_rnea, a_curr_vec_out_AZ_rnea, a_curr_vec_out_LX_rnea, a_curr_vec_out_LY_rnea, a_curr_vec_out_LZ_rnea);")
bw.writeLine("    Vector#(6,Bit#(32)) vec_v = vec(v_curr_vec_out_AX_rnea, v_curr_vec_out_AY_rnea, v_curr_vec_out_AZ_rnea, v_curr_vec_out_LX_rnea, v_curr_vec_out_LY_rnea, v_curr_vec_out_LZ_rnea);")
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    Vector#(6,Bit#(32)) vec_dvdq_PE{PE_id} = vec(dvdq_curr_vec_out_AX_dqPE{PE_id},dvdq_curr_vec_out_AY_dqPE{PE_id},dvdq_curr_vec_out_AZ_dqPE{PE_id},dvdq_curr_vec_out_LX_dqPE{PE_id},dvdq_curr_vec_out_LY_dqPE{PE_id},dvdq_curr_vec_out_LZ_dqPE{PE_id});".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    Vector#(6,Bit#(32)) vec_dadq_PE{PE_id} = vec(dadq_curr_vec_out_AX_dqPE{PE_id},dadq_curr_vec_out_AY_dqPE{PE_id},dadq_curr_vec_out_AZ_dqPE{PE_id},dadq_curr_vec_out_LX_dqPE{PE_id},dadq_curr_vec_out_LY_dqPE{PE_id},dadq_curr_vec_out_LZ_dqPE{PE_id});".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    Vector#(6,Bit#(32)) vec_dvdqd_PE{PE_id} = vec(dvdqd_curr_vec_out_AX_dqdPE{PE_id},dvdqd_curr_vec_out_AY_dqdPE{PE_id},dvdqd_curr_vec_out_AZ_dqdPE{PE_id},dvdqd_curr_vec_out_LX_dqdPE{PE_id},dvdqd_curr_vec_out_LY_dqdPE{PE_id},dvdqd_curr_vec_out_LZ_dqdPE{PE_id});".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    Vector#(6,Bit#(32)) vec_dadqd_PE{PE_id} = vec(dadqd_curr_vec_out_AX_dqdPE{PE_id},dadqd_curr_vec_out_AY_dqdPE{PE_id},dadqd_curr_vec_out_AZ_dqdPE{PE_id},dadqd_curr_vec_out_LX_dqdPE{PE_id},dadqd_curr_vec_out_LY_dqdPE{PE_id},dadqd_curr_vec_out_LZ_dqdPE{PE_id});".format(PE_id=PE_id))
bw.writeLine("")

for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("    DvDaIntermediate dvda_PE{PE_id} = DvDaIntermediate {{dadqd: unpack(pack(vec_dadqd_PE{PE_id})), dvdqd: unpack(pack(vec_dvdqd_PE{PE_id})), dadq: unpack(pack(vec_dadq_PE{PE_id})), dvdq: unpack(pack(vec_dvdq_PE{PE_id}))}};".format(PE_id=PE_id))
bw.writeLine("")

bw.writeLine("    // s/8/LEN_SCHED-1")
bw.writeLine("    if (idx_forward_stream == len_fproc_sched-1) begin")
bw.writeLine("        for (int i=0; i<={num_PEs}; i=i+1) begin".format(num_PEs=num_PEs))
bw.writeLine("            dvda_prev[i] <= DvDaIntermediate {dadqd: unpack(0), dvdqd: unpack(0), dadq: unpack(0), dvdq: unpack(0)};")
bw.writeLine("        end")
bw.writeLine("    end")
bw.writeLine("    else begin")
bw.writeLine("        if (link_out_curr_rnea != 0) begin")
bw.writeLine("            rnea_acc[link_out_curr_rnea] <= RNEAIntermediate {v : unpack(pack(vec_v)), a : unpack(pack(vec_a)), f: unpack(pack(vec_f))};")
bw.writeLine("        end")
bw.writeLine("")
bw.writeLine("        for (int i=1; i<={num_PEs}; i=i+1) begin".format(num_PEs=num_PEs))
bw.writeLine("            // this is same calculation as link_out_curr_PEx, just wanted to for-loopify it")
bw.writeLine("            let link_out_curr_PE = fproc_curr_sched[idx_forward_stream][i-1];")
bw.writeLine("            DvDaIntermediate dvda_PE;")
bw.writeLine("            case(i)")

###
for i in range(num_PEs):
    PE_id = i+1
    bw.writeLine("                {PE_id}: dvda_PE = dvda_PE{PE_id};".format(PE_id=PE_id))
###
bw.writeLine("            endcase")

bw.writeLine("")

bw.writeLine("`include \"GradientPipelineFprocDvDaFlush.bsv\"")

bw.writeLine("")
bw.writeLine("            // branch regs for each PE")
bw.writeLine("            //// branching case")

###
if num_branches > 0:
    if_stmt = "            if ("
    for i,branch_link in enumerate(branch_links):
        if_stmt += "link_out_curr_PE == " + str(branch_link)
        if i < num_branches-1:
            if_stmt += " || "
    if_stmt += ") begin"
    bw.writeLine(if_stmt)

    bw.writeLine("                dvda_branch[i][branchTableFproc(link_out_curr_PE)] <= dvda_PE;")
###
    bw.writeLine("            end")

bw.writeLine("        end")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("    //for (int i=1; i<={num_links}; i=i+1)")
bw.writeLine("    //    $display(\"dvda_prev[\", fshow(i), \"]: \", fshow(dvda_prev[i]));")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("    //for (int i=1; i<={num_links}; i=i+1)")
bw.writeLine("    //    $display(\"rnea_acc[\", fshow(i), \"]: \", fshow(rnea_acc[i]));")
bw.writeLine("")
bw.writeLine("    if (idx_forward_stream == len_fproc_sched-1) begin")
bw.writeLine("        $display(\"rnea for all links\");")
bw.writeLineWithFormatSpecReplace("        for (int lid=1; lid<={num_links}; lid=lid+1) begin")
bw.writeLine("            $display(fshow(new_f_inter_acc[lid]));")
bw.writeLine("        end")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("        for (int lid=1; lid<={num_links}; lid=lid+1) begin")
bw.writeLine("            $display(\"dfdq for link \", fshow(lid));")
bw.writeLine("            for (int i=0; i<6; i=i+1) begin")
bw.writeLine("                $display(fshow(new_dfidq_inter_acc[lid][i]));")
bw.writeLine("            end")
bw.writeLine("        end")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("        for (int lid=1; lid<={num_links}; lid=lid+1) begin")
bw.writeLine("            $display(\"dfdqd for link \", fshow(lid));")
bw.writeLine("            for (int i=0; i<6; i=i+1) begin")
bw.writeLine("                $display(fshow(new_dfidqd_inter_acc[lid][i]));")
bw.writeLine("            end")
bw.writeLine("        end")
bw.writeLine("    end")
bw.writeLine("")
bw.writeLine("    //**********************************")
bw.writeLine("")
bw.writeLine("    $display(\"Idx\", fshow(idx_forward_stream));")
bw.writeLine("")
bw.writeLine("    f_inter_acc <= new_f_inter_acc;")
bw.writeLine("    dfidq_inter_acc <= new_dfidq_inter_acc;")
bw.writeLine("    dfidqd_inter_acc <= new_dfidqd_inter_acc;")
bw.writeLine("")
bw.writeLine("    //**************************")
bw.writeLine("        ")
bw.writeLine("")
bw.writeLine("    let newid = idx_forward_stream + 1;")
bw.writeLine("    // s/7/len_sched-1")
bw.writeLine("    if (idx_forward_stream == len_fproc_sched-1) begin")
bw.writeLine("        // s/9/num_links+2 because bproc takes num_links+1 as input and embed is 1-indexed")
bw.writeLine("        Vector#({max_link_incr}, Intermediate2) embed;".format(max_link_incr=max_link_incr))
bw.writeLine("        embed[0] = unpack(0);")
bw.writeLineWithFormatSpecReplace("        for (int i = 1; i <= {num_links}; i = i + 1) begin")
bw.writeLine("            // embed,inter_acc are 1-indexed")
bw.writeLine("            embed[i] = Intermediate2 {dfidqd : unpack(pack(new_dfidqd_inter_acc[i])), dfidq : unpack(pack(new_dfidq_inter_acc[i])), f: unpack(pack(new_f_inter_acc[i]))};")
bw.writeLine("        end")
bw.writeLine("        embed[{max_link_ref}] = unpack(0);".format(max_link_ref=max_link_ref))
bw.writeLine("        intermediate_values.enq(embed);")
bw.writeLine("        newid = 0;")
bw.writeLine("    end")
bw.writeLine("    idx_forward_stream <= newid; ")
bw.writeLine("")
bw.writeLine("    $display(\"=========== FPROC STREAM for idx \", idx_forward_stream, \" END =============\");")
bw.writeLine("endrule")

