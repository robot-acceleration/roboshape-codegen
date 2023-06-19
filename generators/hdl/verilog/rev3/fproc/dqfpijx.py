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

fm = FileManager("dqfpijx.v")
output_file_path = fm.get_output_file_path()
dqfpijx_file = open(output_file_path, "w")

vw = VerilogWriter(dqfpijx_file, dim_list, num_links)

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
dqfpijx_file.write("`timescale 1ns / 1ps"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("// dq Forward Pass for Link i Input j"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("//------------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("// dqfpijx Module"+"\n")
dqfpijx_file.write("//------------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("module dqfpijx#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)("+"\n")
dqfpijx_file.write("   // clock"+"\n")
dqfpijx_file.write("   input clk,"+"\n")
dqfpijx_file.write("   // reset"+"\n")
dqfpijx_file.write("   input reset,"+"\n")
dqfpijx_file.write("   // state_reg"+"\n")
dqfpijx_file.write("   input  [2:0]"+"\n")
dqfpijx_file.write("      state_reg,"+"\n")
dqfpijx_file.write("   // stage booleans"+"\n")
dqfpijx_file.write("   input"+"\n")
dqfpijx_file.write("      s1_bool_in,s2_bool_in,s3_bool_in,"+"\n")
dqfpijx_file.write("   // link_in"+"\n")
dqfpijx_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]"+"\n")
dqfpijx_file.write("      link_in,"+"\n")
dqfpijx_file.write("   // sin(q) and cos(q)"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      sinq_val_in,cosq_val_in,"+"\n")
dqfpijx_file.write("   // qd_val_in"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      qd_val_in,"+"\n")
dqfpijx_file.write("   // v_curr_vec_in, 6 values"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      v_curr_vec_in_AX,v_curr_vec_in_AY,v_curr_vec_in_AZ,v_curr_vec_in_LX,v_curr_vec_in_LY,v_curr_vec_in_LZ,"+"\n")
dqfpijx_file.write("   // a_curr_vec_in, 6 values"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      a_curr_vec_in_AX,a_curr_vec_in_AY,a_curr_vec_in_AZ,a_curr_vec_in_LX,a_curr_vec_in_LY,a_curr_vec_in_LZ,"+"\n")
dqfpijx_file.write("   // v_prev_vec_in, 6 values"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      v_prev_vec_in_AX,v_prev_vec_in_AY,v_prev_vec_in_AZ,v_prev_vec_in_LX,v_prev_vec_in_LY,v_prev_vec_in_LZ,"+"\n")
dqfpijx_file.write("   // a_prev_vec_in, 6 values"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      a_prev_vec_in_AX,a_prev_vec_in_AY,a_prev_vec_in_AZ,a_prev_vec_in_LX,a_prev_vec_in_LY,a_prev_vec_in_LZ,"+"\n")
#-------------------------------------------------------------------------------
dqfpijx_file.write("   // dvdq_prev_vec_in, 6 values"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "dvdq_prev_vec_in_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqfpijx_file.write("   // dadq_prev_vec_in, 6 values"+"\n")
dqfpijx_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "dadq_prev_vec_in_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqfpijx_file.write("   // mcross boolean"+"\n")
dqfpijx_file.write("   input  mcross,"+"\n")
#-------------------------------------------------------------------------------
dqfpijx_file.write("   // dvdq_curr_vec_out, 6 values"+"\n")
dqfpijx_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "dvdq_curr_vec_out_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqfpijx_file.write("   // dadq_curr_vec_out, 6 values"+"\n")
dqfpijx_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "dadq_curr_vec_out_"
suffix = ""
vec_str = makeD6VecString(prefix,suffix,dim_list)
dqfpijx_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
dqfpijx_file.write("   // dfdq_curr_vec_out, 6 values"+"\n")
dqfpijx_file.write("   output signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      dfdq_curr_vec_out_AX,dfdq_curr_vec_out_AY,dfdq_curr_vec_out_AZ,dfdq_curr_vec_out_LX,dfdq_curr_vec_out_LY,dfdq_curr_vec_out_LZ"+"\n")
dqfpijx_file.write("   );"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("   // internal wires and state"+"\n")
dqfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("   // mux"+"\n")
dqfpijx_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      dadq_prev_vec_mux_AX,dadq_prev_vec_mux_AY,dadq_prev_vec_mux_AZ,dadq_prev_vec_mux_LX,dadq_prev_vec_mux_LY,dadq_prev_vec_mux_LZ,"+"\n")
dqfpijx_file.write("      dvdq_prev_vec_mux_AX,dvdq_prev_vec_mux_AY,dvdq_prev_vec_mux_AZ,dvdq_prev_vec_mux_LX,dvdq_prev_vec_mux_LY,dvdq_prev_vec_mux_LZ;"+"\n")
dqfpijx_file.write("   // mcross"+"\n")
dqfpijx_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijx_file.write("      vordv_prev_vec_mux_AX,vordv_prev_vec_mux_AY,vordv_prev_vec_mux_AZ,vordv_prev_vec_mux_LX,vordv_prev_vec_mux_LY,vordv_prev_vec_mux_LZ,"+"\n")
dqfpijx_file.write("      aorda_prev_vec_mux_AX,aorda_prev_vec_mux_AY,aorda_prev_vec_mux_AZ,aorda_prev_vec_mux_LX,aorda_prev_vec_mux_LY,aorda_prev_vec_mux_LZ;"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("   // input muxes"+"\n")
dqfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("   // dvdq prev"+"\n")
dqfpijx_file.write("   assign dvdq_prev_vec_mux_AX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdq_prev_vec_in_AX;"+"\n")
dqfpijx_file.write("   assign dvdq_prev_vec_mux_AY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdq_prev_vec_in_AY;"+"\n")
dqfpijx_file.write("   assign dvdq_prev_vec_mux_AZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdq_prev_vec_in_AZ;"+"\n")
dqfpijx_file.write("   assign dvdq_prev_vec_mux_LX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdq_prev_vec_in_LX;"+"\n")
dqfpijx_file.write("   assign dvdq_prev_vec_mux_LY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdq_prev_vec_in_LY;"+"\n")
dqfpijx_file.write("   assign dvdq_prev_vec_mux_LZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dvdq_prev_vec_in_LZ;"+"\n")
dqfpijx_file.write("   // dadq prev"+"\n")
dqfpijx_file.write("   assign dadq_prev_vec_mux_AX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadq_prev_vec_in_AX;"+"\n")
dqfpijx_file.write("   assign dadq_prev_vec_mux_AY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadq_prev_vec_in_AY;"+"\n")
dqfpijx_file.write("   assign dadq_prev_vec_mux_AZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadq_prev_vec_in_AZ;"+"\n")
dqfpijx_file.write("   assign dadq_prev_vec_mux_LX = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadq_prev_vec_in_LX;"+"\n")
dqfpijx_file.write("   assign dadq_prev_vec_mux_LY = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadq_prev_vec_in_LY;"+"\n")
dqfpijx_file.write("   assign dadq_prev_vec_mux_LZ = (link_in == "+bitwidth_num_links_str+"'d1) ? 32'd0 : dadq_prev_vec_in_LZ;"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("   // in --> stage 1 --> [reg] --> stage 2 --> [reg] --> stage 3 --> out"+"\n")
dqfpijx_file.write("   //---------------------------------------------------------------------------"+"\n")
dqfpijx_file.write("   // mcross"+"\n")
dqfpijx_file.write("   assign vordv_prev_vec_mux_AX = (mcross) ? v_prev_vec_in_AX : dvdq_prev_vec_mux_AX;"+"\n")
dqfpijx_file.write("   assign vordv_prev_vec_mux_AY = (mcross) ? v_prev_vec_in_AY : dvdq_prev_vec_mux_AY;"+"\n")
dqfpijx_file.write("   assign vordv_prev_vec_mux_AZ = (mcross) ? v_prev_vec_in_AZ : dvdq_prev_vec_mux_AZ;"+"\n")
dqfpijx_file.write("   assign vordv_prev_vec_mux_LX = (mcross) ? v_prev_vec_in_LX : dvdq_prev_vec_mux_LX;"+"\n")
dqfpijx_file.write("   assign vordv_prev_vec_mux_LY = (mcross) ? v_prev_vec_in_LY : dvdq_prev_vec_mux_LY;"+"\n")
dqfpijx_file.write("   assign vordv_prev_vec_mux_LZ = (mcross) ? v_prev_vec_in_LZ : dvdq_prev_vec_mux_LZ;"+"\n")
dqfpijx_file.write("   assign aorda_prev_vec_mux_AX = (mcross) ? a_prev_vec_in_AX : dadq_prev_vec_mux_AX;"+"\n")
dqfpijx_file.write("   assign aorda_prev_vec_mux_AY = (mcross) ? a_prev_vec_in_AY : dadq_prev_vec_mux_AY;"+"\n")
dqfpijx_file.write("   assign aorda_prev_vec_mux_AZ = (mcross) ? a_prev_vec_in_AZ : dadq_prev_vec_mux_AZ;"+"\n")
dqfpijx_file.write("   assign aorda_prev_vec_mux_LX = (mcross) ? a_prev_vec_in_LX : dadq_prev_vec_mux_LX;"+"\n")
dqfpijx_file.write("   assign aorda_prev_vec_mux_LY = (mcross) ? a_prev_vec_in_LY : dadq_prev_vec_mux_LY;"+"\n")
dqfpijx_file.write("   assign aorda_prev_vec_mux_LZ = (mcross) ? a_prev_vec_in_LZ : dadq_prev_vec_mux_LZ;"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("   dqfpijx_seq#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijx_file.write("      dqfpijx_seq_unit("+"\n")
dqfpijx_file.write("      // clock"+"\n")
dqfpijx_file.write("      .clk(clk),"+"\n")
dqfpijx_file.write("      // stage booleans"+"\n")
dqfpijx_file.write("      .s1_bool_in(s1_bool_in),.s2_bool_in(s2_bool_in),.s3_bool_in(s3_bool_in),"+"\n")
dqfpijx_file.write("      // link_in"+"\n")
dqfpijx_file.write("      .link_in(link_in),"+"\n")
dqfpijx_file.write("      // sin(q) and cos(q)"+"\n")
dqfpijx_file.write("      .sinq_val_in(sinq_val_in),.cosq_val_in(cosq_val_in),"+"\n")
dqfpijx_file.write("      // qd_val_in"+"\n")
dqfpijx_file.write("      .qd_val_in(qd_val_in),"+"\n")
dqfpijx_file.write("      // v_vec_in, 6 values"+"\n")
dqfpijx_file.write("      .v_vec_in_AX(v_curr_vec_in_AX),.v_vec_in_AY(v_curr_vec_in_AY),.v_vec_in_AZ(v_curr_vec_in_AZ),.v_vec_in_LX(v_curr_vec_in_LX),.v_vec_in_LY(v_curr_vec_in_LY),.v_vec_in_LZ(v_curr_vec_in_LZ),"+"\n")
dqfpijx_file.write("      // mcross boolean"+"\n")
dqfpijx_file.write("      .mcross(mcross),"+"\n")
dqfpijx_file.write("      // dv_vec_in, 6 values"+"\n")
dqfpijx_file.write("      .dv_vec_in_AX(vordv_prev_vec_mux_AX),.dv_vec_in_AY(vordv_prev_vec_mux_AY),.dv_vec_in_AZ(vordv_prev_vec_mux_AZ),.dv_vec_in_LX(vordv_prev_vec_mux_LX),.dv_vec_in_LY(vordv_prev_vec_mux_LY),.dv_vec_in_LZ(vordv_prev_vec_mux_LZ),"+"\n")
dqfpijx_file.write("      // da_vec_in, 6 values"+"\n")
dqfpijx_file.write("      .da_vec_in_AX(aorda_prev_vec_mux_AX),.da_vec_in_AY(aorda_prev_vec_mux_AY),.da_vec_in_AZ(aorda_prev_vec_mux_AZ),.da_vec_in_LX(aorda_prev_vec_mux_LX),.da_vec_in_LY(aorda_prev_vec_mux_LY),.da_vec_in_LZ(aorda_prev_vec_mux_LZ),"+"\n")
dqfpijx_file.write("      // dvdq_vec_out, 6 values"+"\n")
dqfpijx_file.write("      .dvdq_vec_out_AX(dvdq_curr_vec_out_AX),.dvdq_vec_out_AY(dvdq_curr_vec_out_AY),.dvdq_vec_out_AZ(dvdq_curr_vec_out_AZ),.dvdq_vec_out_LX(dvdq_curr_vec_out_LX),.dvdq_vec_out_LY(dvdq_curr_vec_out_LY),.dvdq_vec_out_LZ(dvdq_curr_vec_out_LZ),"+"\n")
dqfpijx_file.write("      // dadq_vec_out, 6 values"+"\n")
dqfpijx_file.write("      .dadq_vec_out_AX(dadq_curr_vec_out_AX),.dadq_vec_out_AY(dadq_curr_vec_out_AY),.dadq_vec_out_AZ(dadq_curr_vec_out_AZ),.dadq_vec_out_LX(dadq_curr_vec_out_LX),.dadq_vec_out_LY(dadq_curr_vec_out_LY),.dadq_vec_out_LZ(dadq_curr_vec_out_LZ),"+"\n")
dqfpijx_file.write("      // dfdq_vec_out, 6 values"+"\n")
dqfpijx_file.write("      .dfdq_vec_out_AX(dfdq_curr_vec_out_AX),.dfdq_vec_out_AY(dfdq_curr_vec_out_AY),.dfdq_vec_out_AZ(dfdq_curr_vec_out_AZ),.dfdq_vec_out_LX(dfdq_curr_vec_out_LX),.dfdq_vec_out_LY(dfdq_curr_vec_out_LY),.dfdq_vec_out_LZ(dfdq_curr_vec_out_LZ)"+"\n")
dqfpijx_file.write("      );"+"\n")
dqfpijx_file.write(""+"\n")
dqfpijx_file.write("endmodule"+"\n")
