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

fm = FileManager("dqfpijxfold.v")
output_file_path = fm.get_output_file_path()
dqfpijxfold_file = open(output_file_path, "w")

vw = VerilogWriter(dqfpijxfold_file, dim_list, num_links)

#-------------------------------------------------------------------------------

# Make Transform S1 Muxes
def makeXformS1Muxes(dim_list,xform_bools):
   mat_str = ""
   xfm_str = ""
   xfm_str = vw.makeXformMat1CondAssignString("xform_","s1_bool","xgens_out_","xform_in_",xform_bools,"   ")
   mat_str = mat_str+xfm_str
   mat_str = mat_str+"\n"
   return mat_str

#-------------------------------------------------------------------------------
dqfpijxfold_file.write("`timescale 1ns / 1ps"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("// dq Forward Pass for Link i Input j"+"\n")
dqfpijxfold_file.write("//"+"\n")
dqfpijxfold_file.write("// (174 mult, 124 add)"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("//------------------------------------------------------------------------------"+"\n")
dqfpijxfold_file.write("// dqfpijxfold Module"+"\n")
dqfpijxfold_file.write("//------------------------------------------------------------------------------"+"\n")
dqfpijxfold_file.write("module dqfpijxfold#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)("+"\n")
dqfpijxfold_file.write("   // link_in"+"\n")
dqfpijxfold_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]"+"\n")
dqfpijxfold_file.write("      link_in,"+"\n")
dqfpijxfold_file.write("   // sin(q) and cos(q)"+"\n")
dqfpijxfold_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      sinq_in,cosq_in,"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // xform_in"+"\n")
mat_str = vw.makeXformInputPort(xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // mcross boolean"+"\n")
dqfpijxfold_file.write("   input  mcross,"+"\n")
dqfpijxfold_file.write("   // stage booleans"+"\n")
dqfpijxfold_file.write("   input  s1_bool,s2_bool,s3_bool,"+"\n")
dqfpijxfold_file.write("   // stage 1 inputs"+"\n")
dqfpijxfold_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      s1r1_in_AX,s1r1_in_AY,s1r1_in_AZ,s1r1_in_LX,s1r1_in_LY,s1r1_in_LZ,"+"\n")
dqfpijxfold_file.write("      s1r2_in_AX,s1r2_in_AY,s1r2_in_AZ,s1r2_in_LX,s1r2_in_LY,s1r2_in_LZ,"+"\n")
dqfpijxfold_file.write("      s1r3_in,"+"\n")
dqfpijxfold_file.write("      s1r4_in_AX,s1r4_in_AY,s1r4_in_AZ,s1r4_in_LX,s1r4_in_LY,s1r4_in_LZ,"+"\n")
dqfpijxfold_file.write("   // stage 2 inputs"+"\n")
dqfpijxfold_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      s2r1_in_AX,s2r1_in_AY,s2r1_in_AZ,s2r1_in_LX,s2r1_in_LY,s2r1_in_LZ,"+"\n")
dqfpijxfold_file.write("      s2r2_in_AX,s2r2_in_AY,s2r2_in_AZ,s2r2_in_LX,s2r2_in_LY,s2r2_in_LZ,"+"\n")
dqfpijxfold_file.write("      s2r3_in_AX,s2r3_in_AY,s2r3_in_AZ,s2r3_in_LX,s2r3_in_LY,s2r3_in_LZ,"+"\n")
dqfpijxfold_file.write("      s2r4_in,"+"\n")
dqfpijxfold_file.write("      s2r5_in_AX,s2r5_in_AY,s2r5_in_AZ,s2r5_in_LX,s2r5_in_LY,s2r5_in_LZ,"+"\n")
dqfpijxfold_file.write("   // stage 3 inputs"+"\n")
dqfpijxfold_file.write("   input  signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      s3r1_in_AX,s3r1_in_AY,s3r1_in_AZ,s3r1_in_LX,s3r1_in_LY,s3r1_in_LZ,"+"\n")
dqfpijxfold_file.write("      s3r2_in_AX,s3r2_in_AY,s3r2_in_AZ,s3r2_in_LX,s3r2_in_LY,s3r2_in_LZ,"+"\n")
dqfpijxfold_file.write("      s3r3_in_AX,s3r3_in_AY,s3r3_in_AZ,s3r3_in_LX,s3r3_in_LY,s3r3_in_LZ,"+"\n")
dqfpijxfold_file.write("      s3r4_in_AX,s3r4_in_AY,s3r4_in_AZ,s3r4_in_LX,s3r4_in_LY,s3r4_in_LZ,"+"\n")
dqfpijxfold_file.write("      s3r5_in_AX,s3r5_in_AY,s3r5_in_AZ,s3r5_in_LX,s3r5_in_LY,s3r5_in_LZ,"+"\n")
dqfpijxfold_file.write("   // stage 1 outputs"+"\n")
dqfpijxfold_file.write("   output signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      s1r1_out_AX,s1r1_out_AY,s1r1_out_AZ,s1r1_out_LX,s1r1_out_LY,s1r1_out_LZ,"+"\n")
dqfpijxfold_file.write("      s1r2_out_AX,s1r2_out_AY,s1r2_out_AZ,s1r2_out_LX,s1r2_out_LY,s1r2_out_LZ,"+"\n")
dqfpijxfold_file.write("      s1r3_out_AX,s1r3_out_AY,s1r3_out_AZ,s1r3_out_LX,s1r3_out_LY,s1r3_out_LZ,"+"\n")
dqfpijxfold_file.write("      s1r4_out,"+"\n")
dqfpijxfold_file.write("      s1r5_out_AX,s1r5_out_AY,s1r5_out_AZ,s1r5_out_LX,s1r5_out_LY,s1r5_out_LZ,"+"\n")
dqfpijxfold_file.write("   // stage 2 outputs"+"\n")
dqfpijxfold_file.write("   output signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      s2r1_out_AX,s2r1_out_AY,s2r1_out_AZ,s2r1_out_LX,s2r1_out_LY,s2r1_out_LZ,"+"\n")
dqfpijxfold_file.write("      s2r2_out_AX,s2r2_out_AY,s2r2_out_AZ,s2r2_out_LX,s2r2_out_LY,s2r2_out_LZ,"+"\n")
dqfpijxfold_file.write("      s2r3_out_AX,s2r3_out_AY,s2r3_out_AZ,s2r3_out_LX,s2r3_out_LY,s2r3_out_LZ,"+"\n")
dqfpijxfold_file.write("      s2r4_out_AX,s2r4_out_AY,s2r4_out_AZ,s2r4_out_LX,s2r4_out_LY,s2r4_out_LZ,"+"\n")
dqfpijxfold_file.write("      s2r5_out_AX,s2r5_out_AY,s2r5_out_AZ,s2r5_out_LX,s2r5_out_LY,s2r5_out_LZ,"+"\n")
dqfpijxfold_file.write("   // stage 3 outputs"+"\n")
dqfpijxfold_file.write("   output signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      s3r1_out_AX,s3r1_out_AY,s3r1_out_AZ,s3r1_out_LX,s3r1_out_LY,s3r1_out_LZ,"+"\n")
dqfpijxfold_file.write("      s3r2_out_AX,s3r2_out_AY,s3r2_out_AZ,s3r2_out_LX,s3r2_out_LY,s3r2_out_LZ,"+"\n")
dqfpijxfold_file.write("      s3r3_out_AX,s3r3_out_AY,s3r3_out_AZ,s3r3_out_LX,s3r3_out_LY,s3r3_out_LZ,"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // xform_out"+"\n")
mat_str = vw.makeXformOutputPort(xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   );"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("   // internal wires and state"+"\n")
dqfpijxfold_file.write("   // xform"+"\n")
#-------------------------------------------------------------------------------
mat_str = vw.makeXformWiresPrefix(xform_bools,"xgens_out_")
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
mat_str = vw.makeXformWires(xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // fxdot"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      fxdot_in_fxvec_AX,fxdot_in_fxvec_AY,fxdot_in_fxvec_AZ,fxdot_in_fxvec_LX,fxdot_in_fxvec_LY,fxdot_in_fxvec_LZ;"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      fxdot_in_dotvec_AX,fxdot_in_dotvec_AY,fxdot_in_dotvec_AZ,fxdot_in_dotvec_LX,fxdot_in_dotvec_LY,fxdot_in_dotvec_LZ;"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      fxdot_out_AX,fxdot_out_AY,fxdot_out_AZ,fxdot_out_LX,fxdot_out_LY,fxdot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // idot"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      idot_in_vec_AX,idot_in_vec_AY,idot_in_vec_AZ,idot_in_vec_LX,idot_in_vec_LY,idot_in_vec_LZ;"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      idot_out_AX,idot_out_AY,idot_out_AZ,idot_out_LX,idot_out_LY,idot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // xdot"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      xdot_in_vec_AX,xdot_in_vec_AY,xdot_in_vec_AZ,xdot_in_vec_LX,xdot_in_vec_LY,xdot_in_vec_LZ;"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      xdot_out_AX,xdot_out_AY,xdot_out_AZ,xdot_out_LX,xdot_out_LY,xdot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // vjdot"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      vjdot_out_AX,vjdot_out_AY,vjdot_out_AZ,vjdot_out_LX,vjdot_out_LY,vjdot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // add3"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      add3_temp_AX,add3_temp_AY,add3_temp_AZ,add3_temp_LX,add3_temp_LY,add3_temp_LZ;"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      add3_out_AX,add3_out_AY,add3_out_AZ,add3_out_LX,add3_out_LY,add3_out_LZ;"+"\n")
dqfpijxfold_file.write("   // add2"+"\n")
dqfpijxfold_file.write("   wire signed[(WIDTH-1):0]"+"\n")
dqfpijxfold_file.write("      add2_out_AX,add2_out_AY,add2_out_AZ,add2_out_LX,add2_out_LY,add2_out_LZ;"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("   // xform generation"+"\n")
dqfpijxfold_file.write("   xgens{num_links}#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))".format(num_links=num_links)+"\n")
dqfpijxfold_file.write("      xgens_unit("+"\n")
dqfpijxfold_file.write("      // link_in"+"\n")
dqfpijxfold_file.write("      .link_in(link_in),"+"\n")
dqfpijxfold_file.write("      // sin(q) and cos(q)"+"\n")
dqfpijxfold_file.write("      .sinq_in(sinq_in),.cosq_in(cosq_in),"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("      // xform_out, 15 values"+"\n")
mat_str = vw.makeXformXgensPortAssignments(xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("      );"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("   // input muxes"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // xform"+"\n")
mat_str = makeXformS1Muxes(dim_list,xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // fxdot"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_fxvec_AX  = s2_bool ? s2r5_in_AX : s3r2_in_AX;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_fxvec_AY  = s2_bool ? s2r5_in_AY : s3r2_in_AY;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_fxvec_AZ  = s2_bool ? s2r5_in_AZ : s3r2_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_fxvec_LX  = s2_bool ? s2r5_in_LX : s3r2_in_LX;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_fxvec_LY  = s2_bool ? s2r5_in_LY : s3r2_in_LY;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_fxvec_LZ  = s2_bool ? s2r5_in_LZ : s3r2_in_LZ;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_dotvec_AX = s2_bool ? s2r2_in_AX : s3r3_in_AX;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_dotvec_AY = s2_bool ? s2r2_in_AY : s3r3_in_AY;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_dotvec_AZ = s2_bool ? s2r2_in_AZ : s3r3_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_dotvec_LX = s2_bool ? s2r2_in_LX : s3r3_in_LX;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_dotvec_LY = s2_bool ? s2r2_in_LY : s3r3_in_LY;"+"\n")
dqfpijxfold_file.write("   assign fxdot_in_dotvec_LZ = s2_bool ? s2r2_in_LZ : s3r3_in_LZ;"+"\n")
dqfpijxfold_file.write("   // idot"+"\n")
dqfpijxfold_file.write("   assign idot_in_vec_AX = s1_bool ? s1r1_in_AX :"+"\n")
dqfpijxfold_file.write("                           s2_bool ? s2r5_in_AX : s3r4_in_AX;"+"\n")
dqfpijxfold_file.write("   assign idot_in_vec_AY = s1_bool ? s1r1_in_AY :"+"\n")
dqfpijxfold_file.write("                           s2_bool ? s2r5_in_AY : s3r4_in_AY;"+"\n")
dqfpijxfold_file.write("   assign idot_in_vec_AZ = s1_bool ? s1r1_in_AZ :"+"\n")
dqfpijxfold_file.write("                           s2_bool ? s2r5_in_AZ : s3r4_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign idot_in_vec_LX = s1_bool ? s1r1_in_LX :"+"\n")
dqfpijxfold_file.write("                           s2_bool ? s2r5_in_LX : s3r4_in_LX;"+"\n")
dqfpijxfold_file.write("   assign idot_in_vec_LY = s1_bool ? s1r1_in_LY :"+"\n")
dqfpijxfold_file.write("                           s2_bool ? s2r5_in_LY : s3r4_in_LY;"+"\n")
dqfpijxfold_file.write("   assign idot_in_vec_LZ = s1_bool ? s1r1_in_LZ :"+"\n")
dqfpijxfold_file.write("                           s2_bool ? s2r5_in_LZ : s3r4_in_LZ;"+"\n")
dqfpijxfold_file.write("   // xdot"+"\n")
dqfpijxfold_file.write("   assign xdot_in_vec_AX  = s1_bool ? s1r4_in_AX : s2r3_in_AX;"+"\n")
dqfpijxfold_file.write("   assign xdot_in_vec_AY  = s1_bool ? s1r4_in_AY : s2r3_in_AY;"+"\n")
dqfpijxfold_file.write("   assign xdot_in_vec_AZ  = s1_bool ? s1r4_in_AZ : s2r3_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign xdot_in_vec_LX  = s1_bool ? s1r4_in_LX : s2r3_in_LX;"+"\n")
dqfpijxfold_file.write("   assign xdot_in_vec_LY  = s1_bool ? s1r4_in_LY : s2r3_in_LY;"+"\n")
dqfpijxfold_file.write("   assign xdot_in_vec_LZ  = s1_bool ? s1r4_in_LZ : s2r3_in_LZ;"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("   // functional units"+"\n")
dqfpijxfold_file.write("   // fxdot"+"\n")
dqfpijxfold_file.write("   fxdot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      fxdot_unit("+"\n")
dqfpijxfold_file.write("      // fxvec_in, 6 values"+"\n")
dqfpijxfold_file.write("      .fxvec_in_AX(fxdot_in_fxvec_AX),.fxvec_in_AY(fxdot_in_fxvec_AY),.fxvec_in_AZ(fxdot_in_fxvec_AZ),.fxvec_in_LX(fxdot_in_fxvec_LX),.fxvec_in_LY(fxdot_in_fxvec_LY),.fxvec_in_LZ(fxdot_in_fxvec_LZ),"+"\n")
dqfpijxfold_file.write("      // dotvec_in, 6 values"+"\n")
dqfpijxfold_file.write("      .dotvec_in_AX(fxdot_in_dotvec_AX),.dotvec_in_AY(fxdot_in_dotvec_AY),.dotvec_in_AZ(fxdot_in_dotvec_AZ),.dotvec_in_LX(fxdot_in_dotvec_LX),.dotvec_in_LY(fxdot_in_dotvec_LY),.dotvec_in_LZ(fxdot_in_dotvec_LZ),"+"\n")
dqfpijxfold_file.write("      // fxdotvec_out, 6 values"+"\n")
dqfpijxfold_file.write("      .fxdotvec_out_AX(fxdot_out_AX),.fxdotvec_out_AY(fxdot_out_AY),.fxdotvec_out_AZ(fxdot_out_AZ),.fxdotvec_out_LX(fxdot_out_LX),.fxdotvec_out_LY(fxdot_out_LY),.fxdotvec_out_LZ(fxdot_out_LZ)"+"\n")
dqfpijxfold_file.write("      );"+"\n")
dqfpijxfold_file.write("   // idots"+"\n")
dqfpijxfold_file.write("   idots{num_links}#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))".format(num_links=num_links)+"\n")
dqfpijxfold_file.write("      idots_unit("+"\n")
dqfpijxfold_file.write("      // link_in"+"\n")
dqfpijxfold_file.write("      .link_in(link_in),"+"\n")
dqfpijxfold_file.write("      // vec_in, 6 values"+"\n")
dqfpijxfold_file.write("      .vec_in_AX(idot_in_vec_AX),.vec_in_AY(idot_in_vec_AY),.vec_in_AZ(idot_in_vec_AZ),.vec_in_LX(idot_in_vec_LX),.vec_in_LY(idot_in_vec_LY),.vec_in_LZ(idot_in_vec_LZ),"+"\n")
dqfpijxfold_file.write("      // ivec_out, 6 values"+"\n")
dqfpijxfold_file.write("      .ivec_out_AX(idot_out_AX),.ivec_out_AY(idot_out_AY),.ivec_out_AZ(idot_out_AZ),.ivec_out_LX(idot_out_LX),.ivec_out_LY(idot_out_LY),.ivec_out_LZ(idot_out_LZ)"+"\n")
dqfpijxfold_file.write("      );"+"\n")
dqfpijxfold_file.write("   // xdot"+"\n")
dqfpijxfold_file.write("   xdot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      xdot_unit("+"\n")
dqfpijxfold_file.write("      // mcross boolean"+"\n")
dqfpijxfold_file.write("      .mcross(mcross),"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("      // xform_in, 23 values"+"\n")
mat_str = vw.makeXformXdotPortAssignments(xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("      // vec_in, 6 values"+"\n")
dqfpijxfold_file.write("      .vec_in_AX(xdot_in_vec_AX),.vec_in_AY(xdot_in_vec_AY),.vec_in_AZ(xdot_in_vec_AZ),.vec_in_LX(xdot_in_vec_LX),.vec_in_LY(xdot_in_vec_LY),.vec_in_LZ(xdot_in_vec_LZ),"+"\n")
dqfpijxfold_file.write("      // xvec_out, 6 values"+"\n")
dqfpijxfold_file.write("      .xvec_out_AX(xdot_out_AX),.xvec_out_AY(xdot_out_AY),.xvec_out_AZ(xdot_out_AZ),.xvec_out_LX(xdot_out_LX),.xvec_out_LY(xdot_out_LY),.xvec_out_LZ(xdot_out_LZ)"+"\n")
dqfpijxfold_file.write("      );"+"\n")
dqfpijxfold_file.write("   // vjdot"+"\n")
dqfpijxfold_file.write("   vjdot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      vjdot_unit("+"\n")
dqfpijxfold_file.write("      // vjval_in"+"\n")
dqfpijxfold_file.write("      .vjval_in(s2r4_in),"+"\n")
dqfpijxfold_file.write("      // colvec_in, 6 values"+"\n")
dqfpijxfold_file.write("      .colvec_in_AX(s2r5_in_AX),.colvec_in_AY(s2r5_in_AY),.colvec_in_AZ(s2r5_in_AZ),.colvec_in_LX(s2r5_in_LX),.colvec_in_LY(s2r5_in_LY),.colvec_in_LZ(s2r5_in_LZ),"+"\n")
dqfpijxfold_file.write("      // vjvec_out, 6 values"+"\n")
dqfpijxfold_file.write("      .vjvec_out_AX(vjdot_out_AX),.vjvec_out_AY(vjdot_out_AY),.vjvec_out_AZ(vjdot_out_AZ),.vjvec_out_LX(vjdot_out_LX),.vjvec_out_LY(vjdot_out_LY),.vjvec_out_LZ(vjdot_out_LZ)"+"\n")
dqfpijxfold_file.write("      );"+"\n")
dqfpijxfold_file.write("   // add3"+"\n")
dqfpijxfold_file.write("   // (6 add)"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit1_AX(.a_in(s3r1_in_AX),.b_in(fxdot_out_AX),.sum_out(add3_temp_AX));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit1_AY(.a_in(s3r1_in_AY),.b_in(fxdot_out_AY),.sum_out(add3_temp_AY));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit1_AZ(.a_in(s3r1_in_AZ),.b_in(fxdot_out_AZ),.sum_out(add3_temp_AZ));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit1_LX(.a_in(s3r1_in_LX),.b_in(fxdot_out_LX),.sum_out(add3_temp_LX));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit1_LY(.a_in(s3r1_in_LY),.b_in(fxdot_out_LY),.sum_out(add3_temp_LY));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit1_LZ(.a_in(s3r1_in_LZ),.b_in(fxdot_out_LZ),.sum_out(add3_temp_LZ));"+"\n")
dqfpijxfold_file.write("   // (6 add)"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit2_AX(.a_in(add3_temp_AX),.b_in(idot_out_AX),.sum_out(add3_out_AX));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit2_AY(.a_in(add3_temp_AY),.b_in(idot_out_AY),.sum_out(add3_out_AY));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit2_AZ(.a_in(add3_temp_AZ),.b_in(idot_out_AZ),.sum_out(add3_out_AZ));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit2_LX(.a_in(add3_temp_LX),.b_in(idot_out_LX),.sum_out(add3_out_LX));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit2_LY(.a_in(add3_temp_LY),.b_in(idot_out_LY),.sum_out(add3_out_LY));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add3_unit2_LZ(.a_in(add3_temp_LZ),.b_in(idot_out_LZ),.sum_out(add3_out_LZ));"+"\n")
dqfpijxfold_file.write("   // add2"+"\n")
dqfpijxfold_file.write("   // (4 adds)"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add2_unit_AX(.a_in(xdot_out_AX),.b_in(vjdot_out_AX),.sum_out(add2_out_AX));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add2_unit_AY(.a_in(xdot_out_AY),.b_in(vjdot_out_AY),.sum_out(add2_out_AY));"+"\n")
dqfpijxfold_file.write("   assign add2_out_AZ = xdot_out_AZ;"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add2_unit_LX(.a_in(xdot_out_LX),.b_in(vjdot_out_LX),.sum_out(add2_out_LX));"+"\n")
dqfpijxfold_file.write("   add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n")
dqfpijxfold_file.write("      add2_unit_LY(.a_in(xdot_out_LY),.b_in(vjdot_out_LY),.sum_out(add2_out_LY));"+"\n")
dqfpijxfold_file.write("   assign add2_out_LZ = xdot_out_LZ;"+"\n")
dqfpijxfold_file.write("\n")
dqfpijxfold_file.write("   // outputs"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // xform"+"\n")
mat_str = vw.makeXformOutputAssignments(xform_bools)
dqfpijxfold_file.write(mat_str)
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("   // stage 1"+"\n")
dqfpijxfold_file.write("   // s1r1_out = s1r1_in"+"\n")
dqfpijxfold_file.write("   assign s1r1_out_AX = s1r1_in_AX;"+"\n")
dqfpijxfold_file.write("   assign s1r1_out_AY = s1r1_in_AY;"+"\n")
dqfpijxfold_file.write("   assign s1r1_out_AZ = s1r1_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign s1r1_out_LX = s1r1_in_LX;"+"\n")
dqfpijxfold_file.write("   assign s1r1_out_LY = s1r1_in_LY;"+"\n")
dqfpijxfold_file.write("   assign s1r1_out_LZ = s1r1_in_LZ;"+"\n")
dqfpijxfold_file.write("   // s1r2_out = idot_out"+"\n")
dqfpijxfold_file.write("   assign s1r2_out_AX = idot_out_AX;"+"\n")
dqfpijxfold_file.write("   assign s1r2_out_AY = idot_out_AY;"+"\n")
dqfpijxfold_file.write("   assign s1r2_out_AZ = idot_out_AZ;"+"\n")
dqfpijxfold_file.write("   assign s1r2_out_LX = idot_out_LX;"+"\n")
dqfpijxfold_file.write("   assign s1r2_out_LY = idot_out_LY;"+"\n")
dqfpijxfold_file.write("   assign s1r2_out_LZ = idot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // s1r3_out = s1r2_in"+"\n")
dqfpijxfold_file.write("   assign s1r3_out_AX = s1r2_in_AX;"+"\n")
dqfpijxfold_file.write("   assign s1r3_out_AY = s1r2_in_AY;"+"\n")
dqfpijxfold_file.write("   assign s1r3_out_AZ = s1r2_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign s1r3_out_LX = s1r2_in_LX;"+"\n")
dqfpijxfold_file.write("   assign s1r3_out_LY = s1r2_in_LY;"+"\n")
dqfpijxfold_file.write("   assign s1r3_out_LZ = s1r2_in_LZ;"+"\n")
dqfpijxfold_file.write("   // s1r4_out = s1r3_in"+"\n")
dqfpijxfold_file.write("   assign s1r4_out = s1r3_in;"+"\n")
dqfpijxfold_file.write("   // s1r5_out = xdot_out"+"\n")
dqfpijxfold_file.write("   assign s1r5_out_AX = xdot_out_AX;"+"\n")
dqfpijxfold_file.write("   assign s1r5_out_AY = xdot_out_AY;"+"\n")
dqfpijxfold_file.write("   assign s1r5_out_AZ = xdot_out_AZ;"+"\n")
dqfpijxfold_file.write("   assign s1r5_out_LX = xdot_out_LX;"+"\n")
dqfpijxfold_file.write("   assign s1r5_out_LY = xdot_out_LY;"+"\n")
dqfpijxfold_file.write("   assign s1r5_out_LZ = xdot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // stage 2"+"\n")
dqfpijxfold_file.write("   // s2r1_out = fxdot_out"+"\n")
dqfpijxfold_file.write("   assign s2r1_out_AX = fxdot_out_AX;"+"\n")
dqfpijxfold_file.write("   assign s2r1_out_AY = fxdot_out_AY;"+"\n")
dqfpijxfold_file.write("   assign s2r1_out_AZ = fxdot_out_AZ;"+"\n")
dqfpijxfold_file.write("   assign s2r1_out_LX = fxdot_out_LX;"+"\n")
dqfpijxfold_file.write("   assign s2r1_out_LY = fxdot_out_LY;"+"\n")
dqfpijxfold_file.write("   assign s2r1_out_LZ = fxdot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // s2r2_out = s2r1_in"+"\n")
dqfpijxfold_file.write("   assign s2r2_out_AX = s2r1_in_AX;"+"\n")
dqfpijxfold_file.write("   assign s2r2_out_AY = s2r1_in_AY;"+"\n")
dqfpijxfold_file.write("   assign s2r2_out_AZ = s2r1_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign s2r2_out_LX = s2r1_in_LX;"+"\n")
dqfpijxfold_file.write("   assign s2r2_out_LY = s2r1_in_LY;"+"\n")
dqfpijxfold_file.write("   assign s2r2_out_LZ = s2r1_in_LZ;"+"\n")
dqfpijxfold_file.write("   // s2r3_out = idot_out"+"\n")
dqfpijxfold_file.write("   assign s2r3_out_AX = idot_out_AX;"+"\n")
dqfpijxfold_file.write("   assign s2r3_out_AY = idot_out_AY;"+"\n")
dqfpijxfold_file.write("   assign s2r3_out_AZ = idot_out_AZ;"+"\n")
dqfpijxfold_file.write("   assign s2r3_out_LX = idot_out_LX;"+"\n")
dqfpijxfold_file.write("   assign s2r3_out_LY = idot_out_LY;"+"\n")
dqfpijxfold_file.write("   assign s2r3_out_LZ = idot_out_LZ;"+"\n")
dqfpijxfold_file.write("   // s2r4_out = add2_out"+"\n")
dqfpijxfold_file.write("   assign s2r4_out_AX = add2_out_AX;"+"\n")
dqfpijxfold_file.write("   assign s2r4_out_AY = add2_out_AY;"+"\n")
dqfpijxfold_file.write("   assign s2r4_out_AZ = add2_out_AZ;"+"\n")
dqfpijxfold_file.write("   assign s2r4_out_LX = add2_out_LX;"+"\n")
dqfpijxfold_file.write("   assign s2r4_out_LY = add2_out_LY;"+"\n")
dqfpijxfold_file.write("   assign s2r4_out_LZ = add2_out_LZ;"+"\n")
dqfpijxfold_file.write("   // s2r5_out = s2r5_in"+"\n")
dqfpijxfold_file.write("   assign s2r5_out_AX = s2r5_in_AX;"+"\n")
dqfpijxfold_file.write("   assign s2r5_out_AY = s2r5_in_AY;"+"\n")
dqfpijxfold_file.write("   assign s2r5_out_AZ = s2r5_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign s2r5_out_LX = s2r5_in_LX;"+"\n")
dqfpijxfold_file.write("   assign s2r5_out_LY = s2r5_in_LY;"+"\n")
dqfpijxfold_file.write("   assign s2r5_out_LZ = s2r5_in_LZ;"+"\n")
dqfpijxfold_file.write("   // stage 3"+"\n")
dqfpijxfold_file.write("   // s3r1_out = add3_out"+"\n")
dqfpijxfold_file.write("   assign s3r1_out_AX = add3_out_AX;"+"\n")
dqfpijxfold_file.write("   assign s3r1_out_AY = add3_out_AY;"+"\n")
dqfpijxfold_file.write("   assign s3r1_out_AZ = add3_out_AZ;"+"\n")
dqfpijxfold_file.write("   assign s3r1_out_LX = add3_out_LX;"+"\n")
dqfpijxfold_file.write("   assign s3r1_out_LY = add3_out_LY;"+"\n")
dqfpijxfold_file.write("   assign s3r1_out_LZ = add3_out_LZ;"+"\n")
dqfpijxfold_file.write("   // s3r2_out = s3r4_in"+"\n")
dqfpijxfold_file.write("   assign s3r2_out_AX = s3r4_in_AX;"+"\n")
dqfpijxfold_file.write("   assign s3r2_out_AY = s3r4_in_AY;"+"\n")
dqfpijxfold_file.write("   assign s3r2_out_AZ = s3r4_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign s3r2_out_LX = s3r4_in_LX;"+"\n")
dqfpijxfold_file.write("   assign s3r2_out_LY = s3r4_in_LY;"+"\n")
dqfpijxfold_file.write("   assign s3r2_out_LZ = s3r4_in_LZ;"+"\n")
dqfpijxfold_file.write("   // s3r3_out = s3r5_in"+"\n")
dqfpijxfold_file.write("   assign s3r3_out_AX = s3r5_in_AX;"+"\n")
dqfpijxfold_file.write("   assign s3r3_out_AY = s3r5_in_AY;"+"\n")
dqfpijxfold_file.write("   assign s3r3_out_AZ = s3r5_in_AZ;"+"\n")
dqfpijxfold_file.write("   assign s3r3_out_LX = s3r5_in_LX;"+"\n")
dqfpijxfold_file.write("   assign s3r3_out_LY = s3r5_in_LY;"+"\n")
dqfpijxfold_file.write("   assign s3r3_out_LZ = s3r5_in_LZ;"+"\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("\n")
#-------------------------------------------------------------------------------
dqfpijxfold_file.write("endmodule"+"\n")
#-------------------------------------------------------------------------------

# Close file
dqfpijxfold_file.close()
