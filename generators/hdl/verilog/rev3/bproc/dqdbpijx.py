from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file, block_size

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()
xform_bools_full = fpga_codegen.get_Xmat_sparsity_boolean_matrix_OR()
xform_bools = fpga_codegen.left_half_matrix(xform_bools_full)

#-------- File management -------------------

fm = FileManager("dqdbpijx.v")
output_file_path = fm.get_output_file_path()
dqdbpijx_file = open(output_file_path, "w")

vw = VerilogWriter(dqdbpijx_file, dim_list, num_links, block_size)

#---------------------------------------------

xtdotminv = fpga_codegen.get_xtdotminv(vw, block_size)

#---------------------------------------------

############

vw.writeLine("`timescale 1ns / 1ps")
vw.writeLine("")
vw.writeLine("// dqd Backward Pass for Link i Input j and Minv")
vw.writeLine("")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("// dqdbpijx Module")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("module dqdbpijx#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(")
vw.writeLine("   // link_in")
vw.writeLine("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]")
vw.writeLine("      link_in,")
vw.writeLine("   // sin(q) and cos(q)")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      sinq_in,cosq_in,")
vw.writeLine("   // dfdqd_curr_in, 6 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      dfdqd_curr_in_AX,dfdqd_curr_in_AY,dfdqd_curr_in_AZ,dfdqd_curr_in_LX,dfdqd_curr_in_LY,dfdqd_curr_in_LZ,")
vw.writeLine("   // dfdqd_prev_in, 6 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      dfdqd_prev_in_AX,dfdqd_prev_in_AY,dfdqd_prev_in_AZ,dfdqd_prev_in_LX,dfdqd_prev_in_LY,dfdqd_prev_in_LZ,")
vw.writeLine("   // dtau_dqd_out")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeLine("      dtau_dqd_out,")
vw.writeLine("   // dfdqd_prev_out, 6 values")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeLine("      dfdqd_prev_out_AX,dfdqd_prev_out_AY,dfdqd_prev_out_AZ,dfdqd_prev_out_LX,dfdqd_prev_out_LY,dfdqd_prev_out_LZ,")
vw.writeLine("   // minv boolean")
vw.writeLine("   input  minv,")

###
vw.writeLine("   // minvm_in")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.setIndentLevel("      ")
vw.writeCommaSep(vw.genColRowVector_1d_block_size("minvm_in"), lineSepElements=block_size)
###

###
vw.writeLine("   // tau_vec_in")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.setIndentLevel("       ")
vw.writeCommaSep(vw.blockSizeFormatStr("tau_vec_in_R{b}"))
###

###
vw.writeLine("   // minv_vec_out")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.setIndentLevel("       ")
vw.writeCommaSep(vw.blockSizeFormatStr("minv_vec_out_R{b}"), last=True)
###
vw.writeLine("   );")

###
vw.writeLine("   // internal wires and state")
mat_str = vw.makeXformWires(xform_bools)
dqdbpijx_file.write(mat_str)
###

vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      lprev_dfi_dqd_updated_AX,lprev_dfi_dqd_updated_AY,lprev_dfi_dqd_updated_AZ,lprev_dfi_dqd_updated_LX,lprev_dfi_dqd_updated_LY,lprev_dfi_dqd_updated_LZ;")

###
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.setIndentLevel("      ")
vw.writeCommaSep(xtdotminv.gen_xtdotminv_minvm_in(), lineSepElements=block_size, last=2)
###

###
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.setIndentLevel("      ")
vw.writeCommaSep(xtdotminv.gen_xtdotminv_vec_in(), last=2)
###

###
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.setIndentLevel("      ")
vw.writeCommaSep(xtdotminv.gen_xtdotminv_vec_out(), last=2)
###

vw.writeLine("   // xform generation")
vw.writeLine("   xgens{num_links}#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))".format(num_links=num_links))
vw.writeLine("      xgens_unit(")
vw.writeLine("      // link_in")
vw.writeLine("      .link_in(link_in),")
vw.writeLine("      // sin(q) and cos(q)")
vw.writeLine("      .sinq_in(sinq_in),.cosq_in(cosq_in),")

###
dqdbpijx_file.write("      // xform_out"+"\n")
mat_str = vw.makeXformXgensPortAssignmentsWirePrefix(xform_bools,"xform_")
dqdbpijx_file.write(mat_str)
###

vw.writeLine("      );")
vw.writeLine("")
vw.writeLine("   // output dtau/dqd")
vw.writeLine("   assign dtau_dqd_out = dfdqd_curr_in_AZ;")

###
vw.writeLine("   // xtdotminv input muxes")
vw.setIndentLevel("   ")
xtdotminv.write_xtdotminv_minvm_in_mux()
vw.writeLine("")
xtdotminv.write_xtdotminv_vec_in_mux()
vw.writeLine("")
###

vw.writeLine("   // update df/dqd or minv matrix multiplication")
vw.writeLine("   xtdotminv#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      xtdotminv_unit(")
vw.writeLine("      // minv boolean")
vw.writeLine("      .minv(minv),")

###
vw.writeLine("      // xform_in, 23 values")
vw.setIndentLevel("      ")
xtdotminv.write_xtdotminv_minvm_in_port_assignments()
vw.writeLine("      // vec_in, 6 values")
xtdotminv.write_xtdotminv_vec_in_port_assignments()
vw.writeLine("      // xtvec_out, 6 values")
xtdotminv.write_xtdotminv_vec_out_port_assignments()
####
vw.writeLine("      );")

vw.writeLine("   // (6 adds)")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_AX(.a_in(dfdqd_prev_in_AX),.b_in(xtdot_vec_out_AX),.sum_out(lprev_dfi_dqd_updated_AX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_AY(.a_in(dfdqd_prev_in_AY),.b_in(xtdot_vec_out_AY),.sum_out(lprev_dfi_dqd_updated_AY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_AZ(.a_in(dfdqd_prev_in_AZ),.b_in(xtdot_vec_out_AZ),.sum_out(lprev_dfi_dqd_updated_AZ));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_LX(.a_in(dfdqd_prev_in_LX),.b_in(xtdot_vec_out_LX),.sum_out(lprev_dfi_dqd_updated_LX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_LY(.a_in(dfdqd_prev_in_LY),.b_in(xtdot_vec_out_LY),.sum_out(lprev_dfi_dqd_updated_LY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_dfdqd_LZ(.a_in(dfdqd_prev_in_LZ),.b_in(xtdot_vec_out_LZ),.sum_out(lprev_dfi_dqd_updated_LZ));")
vw.writeLine("")
vw.writeLine("   // output df/dqd updates")
vw.writeLine("   assign dfdqd_prev_out_AX = lprev_dfi_dqd_updated_AX;")
vw.writeLine("   assign dfdqd_prev_out_AY = lprev_dfi_dqd_updated_AY;")
vw.writeLine("   assign dfdqd_prev_out_AZ = lprev_dfi_dqd_updated_AZ;")
vw.writeLine("   assign dfdqd_prev_out_LX = lprev_dfi_dqd_updated_LX;")
vw.writeLine("   assign dfdqd_prev_out_LY = lprev_dfi_dqd_updated_LY;")
vw.writeLine("   assign dfdqd_prev_out_LZ = lprev_dfi_dqd_updated_LZ;")

###
vw.writeLine("   // minv outputs")
vw.setIndentLevel("   ")
xtdotminv.assign_minv_outputs()
   
vw.writeLine("endmodule")
