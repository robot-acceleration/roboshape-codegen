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

fm = FileManager("dqbpijx.v")
output_file_path = fm.get_output_file_path()
dqbpijx_file = open(output_file_path, "w")

vw = VerilogWriter(dqbpijx_file, dim_list, num_links)

#---------------------------------------------

############

vw.writeLine("`timescale 1ns / 1ps")
vw.writeLine("")
vw.writeLine("// dq Backward Pass for Link i Input j")
vw.writeLine("")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("// dqbpijx Module")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("module dqbpijx#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(")
vw.writeLine("   // link_in")
vw.writeLine("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]")
vw.writeLine("      link_in,")
vw.writeLine("   // sin(q) and cos(q)")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      sinq_in,cosq_in,")
vw.writeLine("   // fcurr_in, 6 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      fcurr_in_AX,fcurr_in_AY,fcurr_in_AZ,fcurr_in_LX,fcurr_in_LY,fcurr_in_LZ,")
vw.writeLine("   // dfdq_curr_in, 6 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      dfdq_curr_in_AX,dfdq_curr_in_AY,dfdq_curr_in_AZ,dfdq_curr_in_LX,dfdq_curr_in_LY,dfdq_curr_in_LZ,")
vw.writeLine("   // fcross boolean")
vw.writeLine("   input  fcross,")
vw.writeLine("   // dfdq_prev_in, 6 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      dfdq_prev_in_AX,dfdq_prev_in_AY,dfdq_prev_in_AZ,dfdq_prev_in_LX,dfdq_prev_in_LY,dfdq_prev_in_LZ,")
vw.writeLine("   // dtau_dq_out")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeLine("      dtau_dq_out,")
vw.writeLine("   // dfdq_prev_out, 6 values")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeLine("      dfdq_prev_out_AX,dfdq_prev_out_AY,dfdq_prev_out_AZ,dfdq_prev_out_LX,dfdq_prev_out_LY,dfdq_prev_out_LZ")
vw.writeLine("   );")

###
vw.writeLine("   // internal wires and state")
mat_str = vw.makeXformWires(xform_bools)
dqbpijx_file.write(mat_str)
###

vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      xtdot_out_AX,xtdot_out_AY,xtdot_out_AZ,xtdot_out_LX,xtdot_out_LY,xtdot_out_LZ;")
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      ficross_AX,ficross_AY,ficross_AZ,ficross_LX,ficross_LY,ficross_LZ;")
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      fcross_out_AX,fcross_out_AY,fcross_out_AZ,fcross_out_LX,fcross_out_LY,fcross_out_LZ;")
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      lprev_dfi_dq_xtdot_updated_AX,lprev_dfi_dq_xtdot_updated_AY,lprev_dfi_dq_xtdot_updated_AZ,lprev_dfi_dq_xtdot_updated_LX,lprev_dfi_dq_xtdot_updated_LY,lprev_dfi_dq_xtdot_updated_LZ;")
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      lprev_dfi_dq_fcross_updated_AX,lprev_dfi_dq_fcross_updated_AY,lprev_dfi_dq_fcross_updated_AZ,lprev_dfi_dq_fcross_updated_LX,lprev_dfi_dq_fcross_updated_LY,lprev_dfi_dq_fcross_updated_LZ;")
vw.writeLine("   wire signed[(WIDTH-1):0]")
vw.writeLine("      lprev_dfi_dq_out_AX,lprev_dfi_dq_out_AY,lprev_dfi_dq_out_AZ,lprev_dfi_dq_out_LX,lprev_dfi_dq_out_LY,lprev_dfi_dq_out_LZ;")
vw.writeLine("")
vw.writeLine("   // xform generation")
vw.writeLine("   xgens{num_links}#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))".format(num_links=num_links))
vw.writeLine("      xgens_unit(")
vw.writeLine("      // link_in")
vw.writeLine("      .link_in(link_in),")
vw.writeLine("      // sin(q) and cos(q)")
vw.writeLine("      .sinq_in(sinq_in),.cosq_in(cosq_in),")

###
dqbpijx_file.write("      // xform_out"+"\n")
mat_str = vw.makeXformXgensPortAssignmentsWirePrefix(xform_bools,"xform_")
dqbpijx_file.write(mat_str)
###

vw.writeLine("      );")
vw.writeLine("")
vw.writeLine("   // output dtau/dq")
vw.writeLine("   assign dtau_dq_out = dfdq_curr_in_AZ;")
vw.writeLine("")
vw.writeLine("   // update df/dq")
vw.writeLine("   xtdot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      xtdot_xtdot(")

###
dqbpijx_file.write("      // xform_in"+"\n")
mat_str = vw.makeXformXdotPortAssignments(xform_bools)
dqbpijx_file.write(mat_str)
###

vw.writeLine("      // vec_in, 6 values")
vw.writeLine("      .vec_in_AX(dfdq_curr_in_AX),.vec_in_AY(dfdq_curr_in_AY),.vec_in_AZ(dfdq_curr_in_AZ),.vec_in_LX(dfdq_curr_in_LX),.vec_in_LY(dfdq_curr_in_LY),.vec_in_LZ(dfdq_curr_in_LZ),")
vw.writeLine("      // xtvec_out, 6 values")
vw.writeLine("      .xtvec_out_AX(xtdot_out_AX),.xtvec_out_AY(xtdot_out_AY),.xtvec_out_AZ(xtdot_out_AZ),.xtvec_out_LX(xtdot_out_LX),.xtvec_out_LY(xtdot_out_LY),.xtvec_out_LZ(xtdot_out_LZ)")
vw.writeLine("      );")
vw.writeLine("   // (6 adds)")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_xtdot_AX(.a_in(dfdq_prev_in_AX),.b_in(xtdot_out_AX),.sum_out(lprev_dfi_dq_xtdot_updated_AX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_xtdot_AY(.a_in(dfdq_prev_in_AY),.b_in(xtdot_out_AY),.sum_out(lprev_dfi_dq_xtdot_updated_AY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_xtdot_AZ(.a_in(dfdq_prev_in_AZ),.b_in(xtdot_out_AZ),.sum_out(lprev_dfi_dq_xtdot_updated_AZ));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_xtdot_LX(.a_in(dfdq_prev_in_LX),.b_in(xtdot_out_LX),.sum_out(lprev_dfi_dq_xtdot_updated_LX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_xtdot_LY(.a_in(dfdq_prev_in_LY),.b_in(xtdot_out_LY),.sum_out(lprev_dfi_dq_xtdot_updated_LY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_xtdot_LZ(.a_in(dfdq_prev_in_LZ),.b_in(xtdot_out_LZ),.sum_out(lprev_dfi_dq_xtdot_updated_LZ));")
vw.writeLine("")
vw.writeLine("   // fcross")
vw.writeLine("   assign ficross_AX = -fcurr_in_AY;")
vw.writeLine("   assign ficross_AY =  fcurr_in_AX;")
vw.writeLine("   assign ficross_AZ = 32'd0;")
vw.writeLine("   assign ficross_LX = -fcurr_in_LY;")
vw.writeLine("   assign ficross_LY =  fcurr_in_LX;")
vw.writeLine("   assign ficross_LZ = 32'd0;")
vw.writeLine("   xtdot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      xtdot_fcross(")

###
dqbpijx_file.write("      // xform_in"+"\n")
mat_str = vw.makeXformXdotPortAssignments(xform_bools)
dqbpijx_file.write(mat_str)
###

vw.writeLine("      // vec_in, 6 values")
vw.writeLine("      .vec_in_AX(ficross_AX),.vec_in_AY(ficross_AY),.vec_in_AZ(ficross_AZ),.vec_in_LX(ficross_LX),.vec_in_LY(ficross_LY),.vec_in_LZ(ficross_LZ),")
vw.writeLine("      // xtvec_out, 6 values")
vw.writeLine("      .xtvec_out_AX(fcross_out_AX),.xtvec_out_AY(fcross_out_AY),.xtvec_out_AZ(fcross_out_AZ),.xtvec_out_LX(fcross_out_LX),.xtvec_out_LY(fcross_out_LY),.xtvec_out_LZ(fcross_out_LZ)")
vw.writeLine("      );")
vw.writeLine("   // (6 adds)")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_fcross_AX(.a_in(fcross_out_AX),.b_in(lprev_dfi_dq_xtdot_updated_AX),.sum_out(lprev_dfi_dq_fcross_updated_AX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_fcross_AY(.a_in(fcross_out_AY),.b_in(lprev_dfi_dq_xtdot_updated_AY),.sum_out(lprev_dfi_dq_fcross_updated_AY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_fcross_AZ(.a_in(fcross_out_AZ),.b_in(lprev_dfi_dq_xtdot_updated_AZ),.sum_out(lprev_dfi_dq_fcross_updated_AZ));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_fcross_LX(.a_in(fcross_out_LX),.b_in(lprev_dfi_dq_xtdot_updated_LX),.sum_out(lprev_dfi_dq_fcross_updated_LX));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_fcross_LY(.a_in(fcross_out_LY),.b_in(lprev_dfi_dq_xtdot_updated_LY),.sum_out(lprev_dfi_dq_fcross_updated_LY));")
vw.writeLine("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))")
vw.writeLine("      add_fcross_LZ(.a_in(fcross_out_LZ),.b_in(lprev_dfi_dq_xtdot_updated_LZ),.sum_out(lprev_dfi_dq_fcross_updated_LZ));")
vw.writeLine("")
vw.writeLine("   // df/dq update mux")
vw.writeLine("   assign dfdq_prev_out_AX = fcross ? lprev_dfi_dq_fcross_updated_AX : lprev_dfi_dq_xtdot_updated_AX;")
vw.writeLine("   assign dfdq_prev_out_AY = fcross ? lprev_dfi_dq_fcross_updated_AY : lprev_dfi_dq_xtdot_updated_AY;")
vw.writeLine("   assign dfdq_prev_out_AZ = fcross ? lprev_dfi_dq_fcross_updated_AZ : lprev_dfi_dq_xtdot_updated_AZ;")
vw.writeLine("   assign dfdq_prev_out_LX = fcross ? lprev_dfi_dq_fcross_updated_LX : lprev_dfi_dq_xtdot_updated_LX;")
vw.writeLine("   assign dfdq_prev_out_LY = fcross ? lprev_dfi_dq_fcross_updated_LY : lprev_dfi_dq_xtdot_updated_LY;")
vw.writeLine("   assign dfdq_prev_out_LZ = fcross ? lprev_dfi_dq_fcross_updated_LZ : lprev_dfi_dq_xtdot_updated_LZ;")
vw.writeLine("")
vw.writeLine("endmodule")
vw.writeLine("")
