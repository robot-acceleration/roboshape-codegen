from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()
xform_bools_full = fpga_codegen.get_Xmat_sparsity_boolean_matrix_OR()
xform_bools = fpga_codegen.left_half_matrix(xform_bools_full)

#-------- File management -------------------

fm = FileManager("rneabpx.v")
output_file_path = fm.get_output_file_path()
rneabpx_file = open(output_file_path, "w")

vw = VerilogWriter(rneabpx_file, dim_list, num_links)

#---------------------------------------------

############

vw.writeLine("`timescale 1ns / 1ps")
vw.writeLine("")
vw.writeLine("// RNEA for Link i")
vw.writeLine("")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("// rneabpx Module")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("module rneabpx#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(")
vw.writeLine("   // link_in")
vw.writeLine("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]")
vw.writeLine("      link_in,")
vw.writeLine("   // sin(q) and cos(q)")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      sinq_curr_in,cosq_curr_in,")
vw.writeLine("   // f_curr_vec_in")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      f_curr_vec_in_AX,f_curr_vec_in_AY,f_curr_vec_in_AZ,f_curr_vec_in_LX,f_curr_vec_in_LY,f_curr_vec_in_LZ,")
vw.writeLine("   // f_prev_vec_in")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      f_prev_vec_in_AX,f_prev_vec_in_AY,f_prev_vec_in_AZ,f_prev_vec_in_LX,f_prev_vec_in_LY,f_prev_vec_in_LZ,")
vw.writeLine("   // tau_curr_out")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeLine("      tau_curr_out,")
vw.writeLine("   // f_prev_upd_vec_out")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeLine("      f_prev_upd_vec_out_AX,f_prev_upd_vec_out_AY,f_prev_upd_vec_out_AZ,f_prev_upd_vec_out_LX,f_prev_upd_vec_out_LY,f_prev_upd_vec_out_LZ")
vw.writeLine("   );")

###
vw.writeLine("   // internal wires and state")
mat_str = vw.makeXformWires(xform_bools)
rneabpx_file.write(mat_str)
###

vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      xtdot_out_AX,xtdot_out_AY,xtdot_out_AZ,xtdot_out_LX,xtdot_out_LY,xtdot_out_LZ;")
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      f_prev_updated_AX,f_prev_updated_AY,f_prev_updated_AZ,f_prev_updated_LX,f_prev_updated_LY,f_prev_updated_LZ;")
vw.writeLine("")
vw.writeLine("   // xform generation")
vw.writeLine("   xgens{num_links}#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))".format(num_links=num_links))
vw.writeLine("      xgens_unit(")
vw.writeLine("      // link_in")
vw.writeLine("      .link_in(link_in),")
vw.writeLine("      // sin(q) and cos(q)")
vw.writeLine("      .sinq_in(sinq_curr_in),.cosq_in(cosq_curr_in),")

###
rneabpx_file.write("      // xform_out"+"\n")
mat_str = vw.makeXformXgensPortAssignmentsWirePrefix(xform_bools,"xform_")
rneabpx_file.write(mat_str)
###

vw.writeLine("      );")
vw.writeLine("")
vw.writeLine("   // output dtau/dqd")
vw.writeLine("   assign tau_curr_out = f_curr_vec_in_AZ;")
vw.writeLine("")
vw.writeLine("   // update df/dqd")
vw.writeLine("   xtdot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      xtdot_unit(")

###
rneabpx_file.write("      // xform_in"+"\n")
mat_str = vw.makeXformXdotPortAssignments(xform_bools)
rneabpx_file.write(mat_str)
###

vw.writeLine("      // vec_in, 6 values")
vw.writeLine("      .vec_in_AX(f_curr_vec_in_AX),.vec_in_AY(f_curr_vec_in_AY),.vec_in_AZ(f_curr_vec_in_AZ),.vec_in_LX(f_curr_vec_in_LX),.vec_in_LY(f_curr_vec_in_LY),.vec_in_LZ(f_curr_vec_in_LZ),")
vw.writeLine("      // xtvec_out, 6 values")
vw.writeLine("      .xtvec_out_AX(xtdot_out_AX),.xtvec_out_AY(xtdot_out_AY),.xtvec_out_AZ(xtdot_out_AZ),.xtvec_out_LX(xtdot_out_LX),.xtvec_out_LY(xtdot_out_LY),.xtvec_out_LZ(xtdot_out_LZ)")
vw.writeLine("      );")
vw.writeLine("   // (6 adds)")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_AX(.a_in(f_prev_vec_in_AX),.b_in(xtdot_out_AX),.sum_out(f_prev_updated_AX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_AY(.a_in(f_prev_vec_in_AY),.b_in(xtdot_out_AY),.sum_out(f_prev_updated_AY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_AZ(.a_in(f_prev_vec_in_AZ),.b_in(xtdot_out_AZ),.sum_out(f_prev_updated_AZ));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_LX(.a_in(f_prev_vec_in_LX),.b_in(xtdot_out_LX),.sum_out(f_prev_updated_LX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_LY(.a_in(f_prev_vec_in_LY),.b_in(xtdot_out_LY),.sum_out(f_prev_updated_LY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_LZ(.a_in(f_prev_vec_in_LZ),.b_in(xtdot_out_LZ),.sum_out(f_prev_updated_LZ));")
vw.writeLine("")
vw.writeLine("   // output df/dqd updates")
vw.writeLine("   assign f_prev_upd_vec_out_AX = f_prev_updated_AX;")
vw.writeLine("   assign f_prev_upd_vec_out_AY = f_prev_updated_AY;")
vw.writeLine("   assign f_prev_upd_vec_out_AZ = f_prev_updated_AZ;")
vw.writeLine("   assign f_prev_upd_vec_out_LX = f_prev_updated_LX;")
vw.writeLine("   assign f_prev_upd_vec_out_LY = f_prev_updated_LY;")
vw.writeLine("   assign f_prev_upd_vec_out_LZ = f_prev_updated_LZ;")
vw.writeLine("")
vw.writeLine("endmodule")
