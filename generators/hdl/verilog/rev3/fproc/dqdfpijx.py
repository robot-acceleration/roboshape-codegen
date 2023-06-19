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

fm = FileManager("dqdfpijx.v")
output_file_path = fm.get_output_file_path()
dqdfpijx_file = open(output_file_path, "w")

vw = VerilogWriter(dqdfpijx_file, dim_list, num_links)

#--------- Setting commonly used bitwidths -----

# Find Bitwidth of # Links
bitwidth_num_links = vw.get_bitwidth(num_links)
bitwidth_num_links_str = str(bitwidth_num_links)

#---------------------------------------------

# Make 6D Vector String
def makeD6VecString(prefix,suffix,dim_list):
   vec_str = ""
   last_element_bool = False
   for dim,d6 in enumerate(dim_list):
      vec_str = vec_str+prefix+d6+suffix
      last_element_bool = (dim == len(dim_list)-1)
      if (last_element_bool):
         vec_str = vec_str
      else:
         vec_str = vec_str+","
   return vec_str

#-------------------------------------------------------------------------------
dqdfpijx_file.write("`timescale 1ns / 1ps"+"\n")
dqdfpijx_file.write(""+"\n")
dqdfpijx_file.write("// dqd Forward Pass for Link i Input j"+"\n")
dqdfpijx_file.write(""+"\n")
dqdfpijx_file.write("//------------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("// dqdfpijx Module"+"\n")
dqdfpijx_file.write("//------------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("module dqdfpijx#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)("+"\n")
dqdfpijx_file.write("   // clock"+"\n")
dqdfpijx_file.write("   input clk,"+"\n")
dqdfpijx_file.write("   // reset"+"\n")
dqdfpijx_file.write("   input reset,"+"\n")
dqdfpijx_file.write("   // state_reg"+"\n")
dqdfpijx_file.write("   input  [2:0]"+"\n")
dqdfpijx_file.write("      state_reg,"+"\n")
dqdfpijx_file.write("   // stage booleans"+"\n")
dqdfpijx_file.write("   input"+"\n")
dqdfpijx_file.write("      s1_bool_in,s2_bool_in,s3_bool_in,"+"\n")
dqdfpijx_file.write("   // link_in"+"\n")
dqdfpijx_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]"+"\n")
dqdfpijx_file.write("      link_in,"+"\n")
dqdfpijx_file.write("   // sin(q) and cos(q)"+"\n")
dqdfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqdfpijx_file.write("      sinq_val_in,cosq_val_in,"+"\n")
dqdfpijx_file.write("   // qd_val_in"+"\n")
dqdfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqdfpijx_file.write("      qd_val_in,"+"\n")
dqdfpijx_file.write("   // v_curr_vec_in, 6 values"+"\n")
dqdfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqdfpijx_file.write("      v_curr_vec_in_AX,v_curr_vec_in_AY,v_curr_vec_in_AZ,v_curr_vec_in_LX,v_curr_vec_in_LY,v_curr_vec_in_LZ,"+"\n")
dqdfpijx_file.write("   // a_curr_vec_in, 6 values"+"\n")
dqdfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqdfpijx_file.write("      a_curr_vec_in_AX,a_curr_vec_in_AY,a_curr_vec_in_AZ,a_curr_vec_in_LX,a_curr_vec_in_LY,a_curr_vec_in_LZ,"+"\n")
#-------------------------------------------------------------------------------
dqdfpijx_file.write("   // dvdqd_prev_vec_in, 6 values"+"\n")
dqdfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "dvdqd_prev_vec_in_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqdfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqdfpijx_file.write("   // dadqd_prev_vec_in, 6 values"+"\n")
dqdfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "dadqd_prev_vec_in_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqdfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqdfpijx_file.write("   // mcross boolean"+"\n")
dqdfpijx_file.write("   input  mcross,"+"\n")
#-------------------------------------------------------------------------------
dqdfpijx_file.write("   // dvdqd_curr_vec_out, 6 values"+"\n")
dqdfpijx_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "dvdqd_curr_vec_out_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqdfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqdfpijx_file.write("   // dadqd_curr_vec_out, 6 values"+"\n")
dqdfpijx_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "dadqd_curr_vec_out_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqdfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqdfpijx_file.write("   // dfdqd_curr_vec_out, 6 values"+"\n")
dqdfpijx_file.write("   output signed[(WIDTH-1):0]"+"\n")
dqdfpijx_file.write("      dfdqd_curr_vec_out_AX,dfdqd_curr_vec_out_AY,dfdqd_curr_vec_out_AZ,dfdqd_curr_vec_out_LX,dfdqd_curr_vec_out_LY,dfdqd_curr_vec_out_LZ"+"\n")
dqdfpijx_file.write("   );"+"\n")
dqdfpijx_file.write(""+"\n")
dqdfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("   // internal wires and state"+"\n")
dqdfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("   // mux"+"\n")
dqdfpijx_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqdfpijx_file.write("      dadqd_prev_vec_mux_AX,dadqd_prev_vec_mux_AY,dadqd_prev_vec_mux_AZ,dadqd_prev_vec_mux_LX,dadqd_prev_vec_mux_LY,dadqd_prev_vec_mux_LZ,"+"\n")
dqdfpijx_file.write("      dvdqd_prev_vec_mux_AX,dvdqd_prev_vec_mux_AY,dvdqd_prev_vec_mux_AZ,dvdqd_prev_vec_mux_LX,dvdqd_prev_vec_mux_LY,dvdqd_prev_vec_mux_LZ;"+"\n")
dqdfpijx_file.write(""+"\n")
dqdfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("   // input muxes"+"\n")
dqdfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("   // dvdqd prev"+"\n")
dqdfpijx_file.write("   assign dvdqd_prev_vec_mux_AX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdqd_prev_vec_in_AX;"+"\n")
dqdfpijx_file.write("   assign dvdqd_prev_vec_mux_AY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdqd_prev_vec_in_AY;"+"\n")
dqdfpijx_file.write("   assign dvdqd_prev_vec_mux_AZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdqd_prev_vec_in_AZ;"+"\n")
dqdfpijx_file.write("   assign dvdqd_prev_vec_mux_LX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdqd_prev_vec_in_LX;"+"\n")
dqdfpijx_file.write("   assign dvdqd_prev_vec_mux_LY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdqd_prev_vec_in_LY;"+"\n")
dqdfpijx_file.write("   assign dvdqd_prev_vec_mux_LZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdqd_prev_vec_in_LZ;"+"\n")
dqdfpijx_file.write("   // dadqd prev"+"\n")
dqdfpijx_file.write("   assign dadqd_prev_vec_mux_AX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadqd_prev_vec_in_AX;"+"\n")
dqdfpijx_file.write("   assign dadqd_prev_vec_mux_AY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadqd_prev_vec_in_AY;"+"\n")
dqdfpijx_file.write("   assign dadqd_prev_vec_mux_AZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadqd_prev_vec_in_AZ;"+"\n")
dqdfpijx_file.write("   assign dadqd_prev_vec_mux_LX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadqd_prev_vec_in_LX;"+"\n")
dqdfpijx_file.write("   assign dadqd_prev_vec_mux_LY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadqd_prev_vec_in_LY;"+"\n")
dqdfpijx_file.write("   assign dadqd_prev_vec_mux_LZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadqd_prev_vec_in_LZ;"+"\n")
dqdfpijx_file.write(""+"\n")
dqdfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("   // in --> stage 1 --> [reg] --> stage 2 --> [reg] --> stage 3 --> out"+"\n")
dqdfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqdfpijx_file.write("   dqdfpijx_seq#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqdfpijx_file.write("      dqdfpijx_seq_unit("+"\n")
dqdfpijx_file.write("      // clock"+"\n")
dqdfpijx_file.write("      .clk(clk),"+"\n")
dqdfpijx_file.write("      // stage booleans"+"\n")
dqdfpijx_file.write("      .s1_bool_in(s1_bool_in),.s2_bool_in(s2_bool_in),.s3_bool_in(s3_bool_in),"+"\n")
dqdfpijx_file.write("      // link_in"+"\n")
dqdfpijx_file.write("      .link_in(link_in),"+"\n")
dqdfpijx_file.write("      // sin(q) and cos(q)"+"\n")
dqdfpijx_file.write("      .sinq_val_in(sinq_val_in),.cosq_val_in(cosq_val_in),"+"\n")
dqdfpijx_file.write("      // qd_val_in"+"\n")
dqdfpijx_file.write("      .qd_val_in(qd_val_in),"+"\n")
dqdfpijx_file.write("      // v_vec_in, 6 values"+"\n")
dqdfpijx_file.write("      .v_vec_in_AX(v_curr_vec_in_AX),.v_vec_in_AY(v_curr_vec_in_AY),.v_vec_in_AZ(v_curr_vec_in_AZ),.v_vec_in_LX(v_curr_vec_in_LX),.v_vec_in_LY(v_curr_vec_in_LY),.v_vec_in_LZ(v_curr_vec_in_LZ),"+"\n")
dqdfpijx_file.write("      // mcross boolean"+"\n")
dqdfpijx_file.write("      .mcross(mcross),"+"\n")
dqdfpijx_file.write("      // dv_vec_in, 6 values"+"\n")
dqdfpijx_file.write("      .dv_vec_in_AX(dvdqd_prev_vec_mux_AX),.dv_vec_in_AY(dvdqd_prev_vec_mux_AY),.dv_vec_in_AZ(dvdqd_prev_vec_mux_AZ),.dv_vec_in_LX(dvdqd_prev_vec_mux_LX),.dv_vec_in_LY(dvdqd_prev_vec_mux_LY),.dv_vec_in_LZ(dvdqd_prev_vec_mux_LZ),"+"\n")
dqdfpijx_file.write("      // da_vec_in, 6 values"+"\n")
dqdfpijx_file.write("      .da_vec_in_AX(dadqd_prev_vec_mux_AX),.da_vec_in_AY(dadqd_prev_vec_mux_AY),.da_vec_in_AZ(dadqd_prev_vec_mux_AZ),.da_vec_in_LX(dadqd_prev_vec_mux_LX),.da_vec_in_LY(dadqd_prev_vec_mux_LY),.da_vec_in_LZ(dadqd_prev_vec_mux_LZ),"+"\n")
dqdfpijx_file.write("      // dvdqd_vec_out, 6 values"+"\n")
dqdfpijx_file.write("      .dvdqd_vec_out_AX(dvdqd_curr_vec_out_AX),.dvdqd_vec_out_AY(dvdqd_curr_vec_out_AY),.dvdqd_vec_out_AZ(dvdqd_curr_vec_out_AZ),.dvdqd_vec_out_LX(dvdqd_curr_vec_out_LX),.dvdqd_vec_out_LY(dvdqd_curr_vec_out_LY),.dvdqd_vec_out_LZ(dvdqd_curr_vec_out_LZ),"+"\n")
dqdfpijx_file.write("      // dadqd_vec_out, 6 values"+"\n")
dqdfpijx_file.write("      .dadqd_vec_out_AX(dadqd_curr_vec_out_AX),.dadqd_vec_out_AY(dadqd_curr_vec_out_AY),.dadqd_vec_out_AZ(dadqd_curr_vec_out_AZ),.dadqd_vec_out_LX(dadqd_curr_vec_out_LX),.dadqd_vec_out_LY(dadqd_curr_vec_out_LY),.dadqd_vec_out_LZ(dadqd_curr_vec_out_LZ),"+"\n")
dqdfpijx_file.write("      // dfdqd_vec_out, 6 values"+"\n")
dqdfpijx_file.write("      .dfdqd_vec_out_AX(dfdqd_curr_vec_out_AX),.dfdqd_vec_out_AY(dfdqd_curr_vec_out_AY),.dfdqd_vec_out_AZ(dfdqd_curr_vec_out_AZ),.dfdqd_vec_out_LX(dfdqd_curr_vec_out_LX),.dfdqd_vec_out_LY(dfdqd_curr_vec_out_LY),.dfdqd_vec_out_LZ(dfdqd_curr_vec_out_LZ)"+"\n")
dqdfpijx_file.write("      );"+"\n")
dqdfpijx_file.write(""+"\n")
dqdfpijx_file.write("endmodule"+"\n")
