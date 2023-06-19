from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import BluespecWriter, FileManager
from rbd_config import dim_list, urdf_file, num_PEs

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()

#-------- File management -------------------

fm = FileManager("BviIfcFProc_rev1.bsv")
output_file_path = fm.get_output_file_path()
bvi_ifc_file = open(output_file_path, "w")

bw = BluespecWriter(bvi_ifc_file, dim_list, num_links)

#---------------------------------------------

fproc = fpga_codegen.get_fproc(num_PEs)

#---------------------------------------------

bw.writeLine("import Clocks::*;")
bw.writeLine("(* always_ready, always_enabled *)")
bw.writeLine("interface FProc;")
bw.writeLine("    (* always_ready *)")
bw.writeLine("    method Action get_data();")

bw.writeLine("")
bw.writeLine("    //-------- RNEA INPUTS -----")
bw.writeLine("")

###
bits = len(bin(num_links))-2
bw.writeLine("    method Action link_in_rnea(Bit#({bits}) v);".format(bits=bits))
###

bw.writeLine("    method Action sinq_val_in_rnea(Bit#(32) v);")
bw.writeLine("    method Action cosq_val_in_rnea(Bit#(32) v);")
bw.writeLine("")
bw.writeLine("    method Action qd_val_in_rnea(Bit#(32) v);")
bw.writeLine("    method Action qdd_val_in_rnea(Bit#(32) v);")
bw.writeLine("")

###
v_prev_vec_in_rnea = fproc.gen_v_prev_vec_in_rnea_ports()
a_prev_vec_in_rnea = fproc.gen_a_prev_vec_in_rnea_ports()
vec_in_rnea = v_prev_vec_in_rnea + a_prev_vec_in_rnea
for port in vec_in_rnea:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
bw.writeLine("   //--------------------------")
bw.writeLine("")
###

bw.writeLine("    //-------- DQ AND DQD INPUTS -----")
bw.writeLine("")
###
link_in_dq = fproc.gen_link_in_dq_ports()
link_in_dqd = fproc.gen_link_in_dqd_ports()
derv_in_dq = fproc.gen_derv_in_dq_ports()
derv_in_dqd = fproc.gen_derv_in_dqd_ports()
link_derv_dq_dqd = link_in_dq + link_in_dqd + derv_in_dq + derv_in_dqd
for port in link_derv_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#({bits}) v);".format(port=port, bits=bits))
bw.writeLine("")
###

###
sinq_val_in_dq = fproc.gen_sinq_val_in_dq_ports()
sinq_val_in_dqd = fproc.gen_sinq_val_in_dqd_ports()
cosq_val_in_dq = fproc.gen_cosq_val_in_dq_ports()
cosq_val_in_dqd = fproc.gen_cosq_val_in_dqd_ports()
qd_val_in_dq = fproc.gen_qd_val_in_dq_ports()
qd_val_in_dqd = fproc.gen_qd_val_in_dqd_ports()
param_dq_dqd = sinq_val_in_dq + sinq_val_in_dqd + \
        cosq_val_in_dq + cosq_val_in_dqd + \
        qd_val_in_dq + qd_val_in_dqd
for port in param_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
###

###
v_curr_vec_in_dq = fproc.gen_v_curr_vec_in_dq_ports()
v_curr_vec_in_dqd = fproc.gen_v_curr_vec_in_dqd_ports()
a_curr_vec_in_dq = fproc.gen_a_curr_vec_in_dq_ports()
a_curr_vec_in_dqd = fproc.gen_a_curr_vec_in_dqd_ports()
v_prev_vec_in_dq = fproc.gen_v_prev_vec_in_dq_ports()
a_prev_vec_in_dq = fproc.gen_a_prev_vec_in_dq_ports()
vec_in_dq_dqd = v_curr_vec_in_dq + v_curr_vec_in_dqd + \
        a_curr_vec_in_dq + a_curr_vec_in_dqd + \
        v_prev_vec_in_dq + a_prev_vec_in_dq
for port in vec_in_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
###

###
dvdq_prev_vec_in_dq = fproc.gen_dvdq_prev_vec_in_dq_ports()
dvdqd_prev_vec_in_dqd = fproc.gen_dvdqd_prev_vec_in_dqd_ports()
dadq_prev_vec_in_dq = fproc.gen_dadq_prev_vec_in_dq_ports()
dadqd_prev_vec_in_dqd = fproc.gen_dadqd_prev_vec_in_dqd_ports()
ext_branch_in_dq_dqd = dvdq_prev_vec_in_dq + dvdqd_prev_vec_in_dqd + \
        dadq_prev_vec_in_dq + dadqd_prev_vec_in_dqd
for port in ext_branch_in_dq_dqd:
    bw.writeLine("    method Action {port}(Bit#(32) v);".format(port=port))
bw.writeLine("")
###

bw.writeLine("    //--------------------------")
bw.writeLine("")

bw.writeLine("    // output_ready")
bw.writeLine("    method Bit#(1) output_ready();")

bw.writeLine("")
bw.writeLine("    //-------- RNEA OUTPUTS -----")
bw.writeLine("")

###
v_curr_vec_out_rnea = fproc.gen_v_curr_vec_out_rnea_ports()
a_curr_vec_out_rnea = fproc.gen_a_curr_vec_out_rnea_ports()
f_curr_vec_out_rnea = fproc.gen_f_curr_vec_out_rnea_ports()
vec_out_rnea = v_curr_vec_out_rnea + a_curr_vec_out_rnea + f_curr_vec_out_rnea
for port in vec_out_rnea:
    bw.writeLine("    method Bit#(32) {port}();".format(port=port))
bw.writeLine("")
###
bw.writeLine("    //-----------------------")
bw.writeLine("")

bw.writeLine("    //----------- DQ DQD OUTPUTS -----")
bw.writeLine("")

###
dfdq_curr_vec_out_dq = fproc.gen_dfdq_curr_vec_out_dq_ports()
dfdqd_curr_vec_out_dqd = fproc.gen_dfdqd_curr_vec_out_dqd_ports()
vec_out_dq_dqd = dfdq_curr_vec_out_dq + dfdqd_curr_vec_out_dqd
for port in vec_out_dq_dqd:
    bw.writeLine("    method Bit#(32) {port}();".format(port=port))
bw.writeLine("")
###

###
dvdq_curr_vec_out_dq = fproc.gen_dvdq_curr_vec_out_dq_ports()
dvdqd_curr_vec_out_dqd = fproc.gen_dvdqd_curr_vec_out_dqd_ports()
dadq_curr_vec_out_dq = fproc.gen_dadq_curr_vec_out_dq_ports()
dadqd_curr_vec_out_dqd = fproc.gen_dadqd_curr_vec_out_dqd_ports()
ext_branch_out_dq_dqd = dvdq_curr_vec_out_dq + dvdqd_curr_vec_out_dqd + \
        dadq_curr_vec_out_dq + dadqd_curr_vec_out_dqd
for port in ext_branch_out_dq_dqd:
    bw.writeLine("    method Bit#(32) {port}();".format(port=port))
bw.writeLine("")
###
bw.writeLine("    //-----------------------")
bw.writeLine("")

bw.writeLine("endinterface")
bw.writeLine("")
bw.writeLine("import \"BVI\" fproc =")
bw.writeLine("module mkFProc(FProc);")
bw.writeLine("    default_clock clk();")
bw.writeLine("    default_reset rst();")
bw.writeLine("    input_clock (clk) <- exposeCurrentClock; ")
bw.writeLine("    input_reset (reset) <- invertCurrentReset;")
bw.writeLine("    method get_data() enable(get_data);")
bw.writeLine("")
bw.writeLine("    method link_in_rnea(link_in_rnea) enable((*inhigh*) EN_link_in_rnea) ;")
bw.writeLine("    method sinq_val_in_rnea(sinq_val_in_rnea) enable((*inhigh*) EN_sinq_val_in_rnea) ;")
bw.writeLine("    method cosq_val_in_rnea(cosq_val_in_rnea) enable((*inhigh*) EN_cosq_val_in_rnea) ;")
bw.writeLine("    method qd_val_in_rnea(qd_val_in_rnea) enable((*inhigh*) EN_qd_val_in_rnea) ;")
bw.writeLine("    method qdd_val_in_rnea(qdd_val_in_rnea) enable((*inhigh*) EN_qdd_val_in_rnea) ;")
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
for port in ext_branch_in_dq_dqd:
    bw.writeLine("    method {port}({port}) enable((*inhigh*) EN_{port}) ;".format(port=port))
bw.writeLine("")
###

bw.writeLine("    method output_ready output_ready();")
bw.writeLine("")
###
for port in vec_out_rnea:
    bw.writeLine("    method {port} {port}();".format(port=port))
bw.writeLine("")
for port in vec_out_dq_dqd:
    bw.writeLine("    method {port} {port}();".format(port=port))
bw.writeLine("")
for port in ext_branch_out_dq_dqd:
    bw.writeLine("    method {port} {port}();".format(port=port))
bw.writeLine("")
###

bw.writeLine("    schedule (")
bw.writeLine("        get_data, link_in_rnea, sinq_val_in_rnea, cosq_val_in_rnea, qd_val_in_rnea, qdd_val_in_rnea,")

###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_in_rnea, last=0)
bw.writeCommaSep(link_derv_dq_dqd, last=0)
bw.writeCommaSep(param_dq_dqd, last=0)
bw.writeCommaSep(vec_in_dq_dqd, last=0)
bw.writeCommaSep(ext_branch_in_dq_dqd, last=0)
###
bw.writeLine("        output_ready,")
###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_out_rnea, last=0)
bw.writeCommaSep(vec_out_dq_dqd, last=0)
bw.writeCommaSep(ext_branch_out_dq_dqd, last=1)
###

bw.writeLine("    ) CF (")
bw.writeLine("        get_data, link_in_rnea, sinq_val_in_rnea, cosq_val_in_rnea, qd_val_in_rnea, qdd_val_in_rnea,")

###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_in_rnea, last=0)
bw.writeCommaSep(link_derv_dq_dqd, last=0)
bw.writeCommaSep(param_dq_dqd, last=0)
bw.writeCommaSep(vec_in_dq_dqd, last=0)
bw.writeCommaSep(ext_branch_in_dq_dqd, last=0)
###

bw.writeLine("         output_ready,")
###
bw.setIndentLevel("        ")
bw.writeCommaSep(vec_out_rnea, last=0)
bw.writeCommaSep(vec_out_dq_dqd, last=0)
bw.writeCommaSep(ext_branch_out_dq_dqd, last=1)
###

bw.writeLine("    );")
bw.writeLine("")
bw.writeLine("endmodule")
