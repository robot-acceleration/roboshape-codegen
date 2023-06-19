from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import BluespecWriter, FileManager
from rbd_config import dim_list, urdf_file, num_PEs, block_size

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()

#-------- File management -------------------

fm = FileManager("BviIfcBProc_rev2.bsv")
output_file_path = fm.get_output_file_path()
bvi_ifc_file = open(output_file_path, "w")

bw = BluespecWriter(bvi_ifc_file, dim_list, num_links)

#---------------------------------------------

bproc = fpga_codegen.get_bproc(num_PEs, block_size)

#---------------------------------------------

bw.writeLine("import Clocks::*;")
bw.writeLine("(* always_ready, always_enabled *)")
bw.writeLine("interface BProc;")
bw.writeLine("    (* always_ready *)")
bw.writeLine("    method Action get_data();")
bw.writeLine("    method Action get_data_minv();")
bw.writeLine("")

###
# TODO: deprecated in rev3 with block minv multiply
#minv_prev_vec_in = bproc.gen_minv_prev_vec_in()
#for port in minv_prev_vec_in:
#    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
###

bw.writeLine("")
bw.writeLine("    //-------- RNEA INPUTS -----")
bw.writeLine("")

###
# num_links+1 because bproc takes 8 as a link input
bits = len(bin(num_links+1))-2
bw.writeLine("    method Action link_in_rnea(Bit#({bits}) v);".format(bits=bits))
###

bw.writeLine("    method Action sinq_val_in_rnea(Bit#(32) v);")
bw.writeLine("    method Action cosq_val_in_rnea(Bit#(32) v);")
bw.writeLine("")

###
f_upd_curr_vec_in_rnea = bproc.gen_f_upd_curr_vec_in_rnea_ports()
f_prev_vec_in_rnea = bproc.gen_f_prev_vec_in_rnea_ports()
vec_in_rnea = f_upd_curr_vec_in_rnea + f_prev_vec_in_rnea
for port in vec_in_rnea:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
bw.writeLine("   //--------------------------")
bw.writeLine("")
###

bw.writeLine("    //-------- DQ AND DQD INPUTS -----")
bw.writeLine("")
###
link_in_dq = bproc.gen_link_in_dq_ports()
link_in_dqd = bproc.gen_link_in_dqd_ports()
derv_in_dq = bproc.gen_derv_in_dq_ports()
derv_in_dqd = bproc.gen_derv_in_dqd_ports()
link_derv_dq_dqd = link_in_dq + link_in_dqd + derv_in_dq + derv_in_dqd
for port in link_derv_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#({bits}) v);".format(port=port, bits=bits))
bw.writeLine("")
###

###
sinq_val_in_dq = bproc.gen_sinq_val_in_dq_ports()
sinq_val_in_dqd = bproc.gen_sinq_val_in_dqd_ports()
cosq_val_in_dq = bproc.gen_cosq_val_in_dq_ports()
cosq_val_in_dqd = bproc.gen_cosq_val_in_dqd_ports()
param_dq_dqd = sinq_val_in_dq + sinq_val_in_dqd + \
        cosq_val_in_dq + cosq_val_in_dqd
for port in param_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
###

###
f_upd_curr_vec_in_dq = bproc.gen_f_upd_curr_vec_in_dq_ports()
dfdq_prev_vec_in_dq = bproc.gen_dfdq_prev_vec_in_dq_ports()
dfdqd_prev_vec_in_dqd = bproc.gen_dfdqd_prev_vec_in_dqd_ports()
dfdq_upd_curr_vec_in_dq = bproc.gen_dfdq_upd_curr_vec_in_dq_ports()
dfdqd_upd_curr_vec_in_dqd = bproc.gen_dfdqd_upd_curr_vec_in_dqd_ports()
vec_in_dq_dqd = f_upd_curr_vec_in_dq + \
        dfdq_prev_vec_in_dq + dfdq_upd_curr_vec_in_dq + \
        dfdqd_prev_vec_in_dqd + dfdqd_upd_curr_vec_in_dqd
for port in vec_in_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
###

bw.writeLine("    //-------- MINV EXTERNAL INPUTS -----")
bw.writeLine("")
###
minv_block_in_dqd = bproc.gen_minv_block_in_dqd_ports()
dtau_vec_in_dqd = bproc.gen_dtau_vec_in_dqd_ports()
minv_block_mult = minv_block_in_dqd + dtau_vec_in_dqd
for port in minv_block_mult:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")


###


bw.writeLine("    //--------------------------")
bw.writeLine("")

bw.writeLine("    method Bit#(1) output_ready();")
bw.writeLine("    method Bit#(1) output_ready_minv();")
bw.writeLine("")

bw.writeLine("")
bw.writeLine("    //-------- RNEA OUTPUTS -----")
bw.writeLine("")

bw.writeLine("    method Bit#(32) tau_curr_out_rnea();")
###
vec_out_rnea = bproc.gen_f_upd_prev_vec_out_rnea_ports()
for port in vec_out_rnea:
    bw.writeLine("    method Bit#(32) {port}();".format(port=port))
bw.writeLine("")
###
bw.writeLine("    //-----------------------")
bw.writeLine("")

bw.writeLine("    //----------- DQ DQD OUTPUTS -----")
bw.writeLine("")

###
dtau_curr_out_dq = bproc.gen_dtau_curr_out_dq_ports()
dtau_curr_out_dqd = bproc.gen_dtau_curr_out_dqd_ports()
dfdq_upd_prev_vec_out = bproc.gen_dfdq_upd_prev_vec_out_dq_ports()
dfdqd_upd_prev_vec_out = bproc.gen_dfdqd_upd_prev_vec_out_dqd_ports()
vec_out_dq_dqd = dtau_curr_out_dq + dtau_curr_out_dqd + \
        dfdq_upd_prev_vec_out + dfdqd_upd_prev_vec_out
for port in vec_out_dq_dqd:
    bw.writeLine("    method Bit#(32) {port}();".format(port=port))
bw.writeLine("")
###


bw.writeLine("    //----------- MINV EXTERNAL OUTPUTS -----")
bw.writeLine("")
###
minv_vec_out = bproc.gen_minv_vec_out_dqd_ports()
for port in minv_vec_out:
    bw.writeLine("    method Bit#(32) {port}();".format(port=port))
bw.writeLine("")
###



bw.writeLine("endinterface")
bw.writeLine("")
bw.writeLine("import \"BVI\" bproc =")
bw.writeLine("module mkBProc(BProc);")
bw.writeLine("   default_clock clk();")
bw.writeLine("    default_reset rst();")
bw.writeLine("    input_clock (clk) <- exposeCurrentClock; ")
bw.writeLine("    input_reset (reset) <- invertCurrentReset;")
bw.writeLine("    method get_data() enable(get_data);")
bw.writeLine("    method get_data_minv() enable(get_data_minv);")
bw.writeLine("")
bw.writeLine("")
bw.writeLine("    method link_in_rnea(link_in_rnea) enable((*inhigh*) EN_link_in_rnea) ;")
bw.writeLine("    method sinq_val_in_rnea(sinq_val_in_rnea) enable((*inhigh*) EN_sinq_val_in_rnea) ;")
bw.writeLine("    method cosq_val_in_rnea(cosq_val_in_rnea) enable((*inhigh*) EN_cosq_val_in_rnea) ;")
bw.writeLine("")

###
for port in vec_in_rnea:
    bw.writeLine("    method {port}({port}) enable((*inhigh*) EN_{port}) ;".format(port=port))
bw.writeLine("")
for port in link_derv_dq_dqd:
    bw.writeLine("    method {port}({port}) enable((*inhigh*) EN_{port}) ;".format(port=port))
bw.writeLine("")
for port in param_dq_dqd:
    bw.writeLine("    method {port}({port}) enable((*inhigh*) EN_{port}) ;".format(port=port))
bw.writeLine("")
for port in vec_in_dq_dqd:
    bw.writeLine("    method {port}({port}) enable((*inhigh*) EN_{port}) ;".format(port=port))
bw.writeLine("")
for port in minv_block_mult:
    bw.writeLine("    method {port}({port}) enable((*inhigh*) EN_{port}) ;".format(port=port))
bw.writeLine("")
###


bw.writeLine("    method output_ready output_ready();")
bw.writeLine("    method output_ready_minv output_ready_minv();")
bw.writeLine("    method tau_curr_out_rnea tau_curr_out_rnea();")
bw.writeLine("")

###
bw.writeLine("")
for port in vec_out_rnea:
    bw.writeLine("    method {port} {port}();".format(port=port))
bw.writeLine("")
for port in vec_out_dq_dqd:
    bw.writeLine("    method {port} {port}();".format(port=port))
for port in minv_vec_out:
    bw.writeLine("    method {port} {port}();".format(port=port))
bw.writeLine("")
###

bw.writeLine("    schedule (")
bw.writeLine("        get_data, get_data_minv, link_in_rnea, sinq_val_in_rnea, cosq_val_in_rnea,")

###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_in_rnea, last=0)
bw.writeCommaSep(link_derv_dq_dqd, last=0)
bw.writeCommaSep(param_dq_dqd, last=0)
bw.writeCommaSep(vec_in_dq_dqd, last=0)
bw.writeCommaSep(minv_block_mult, last=0)
###
bw.writeLine("        output_ready, output_ready_minv, tau_curr_out_rnea,")
###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_out_rnea, last=0)
bw.writeCommaSep(vec_out_dq_dqd, last=0)
bw.writeCommaSep(minv_vec_out, last=1)
###


bw.writeLine("    ) CF (")
bw.writeLine("        get_data, get_data_minv, link_in_rnea, sinq_val_in_rnea, cosq_val_in_rnea,")

###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_in_rnea, last=0)
bw.writeCommaSep(link_derv_dq_dqd, last=0)
bw.writeCommaSep(param_dq_dqd, last=0)
bw.writeCommaSep(vec_in_dq_dqd, last=0)
bw.writeCommaSep(minv_block_mult, last=0)
###
bw.writeLine("        output_ready, output_ready_minv, tau_curr_out_rnea,")
###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_out_rnea, last=0)
bw.writeCommaSep(vec_out_dq_dqd, last=0)
bw.writeCommaSep(minv_vec_out, last=1)
###

bw.writeLine("    );")
bw.writeLine("")
bw.writeLine("endmodule")
