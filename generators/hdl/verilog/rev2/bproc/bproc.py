from URDFParser import URDFParser
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()

#-------- File management -------------------

fm = FileManager("bproc.v")
output_file_path = fm.get_output_file_path()
bproc_file = open(output_file_path, "w")

vw = VerilogWriter(bproc_file, dim_list, num_links)

#---------------------------------------------

###

# Make 6-D Vector String
def make6VecString(prefix,dim_list):
   last_element_bool = False
   vec_str = ""
   for dim,d6 in enumerate(dim_list):
      last_element_bool = (dim == len(dim_list)-1)
      vec_str = vec_str+prefix+d6
      if (last_element_bool):
         vec_str = vec_str+""
      else:
         vec_str = vec_str+","
   return vec_str

# Make N-D Vector String
def makeNVecString(prefix,num_links,tag_str):
   last_element_bool = False
   vec_str = ""
   for link in range(1,num_links+1):
      last_element_bool = (link==num_links)
      vec_str = vec_str+prefix+tag_str+str(link)
      if (last_element_bool):
         vec_str = vec_str+""
      else:
         vec_str = vec_str+","
   return vec_str

# Make 6xN Matrix String
def make6XNMatString(prefix,dim_list,num_links,indent):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for dim,d6 in enumerate(dim_list):
      row_str = ""
      for link in range(1,num_links+1):
         last_element_bool = (dim == len(dim_list)-1)&(link==num_links)
         if (last_element_bool):
            row_str = row_str+prefix+d6+"_J"+str(link)
         else:
            row_str = row_str+prefix+d6+"_J"+str(link)+","
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make NxN Matrix String
def makeNXNMatString(prefix,num_links,indent):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for row in range(1,num_links+1):
      row_str = ""
      for col in range(1,num_links+1):
         last_element_bool = (row==num_links)&(col==num_links)
         if (last_element_bool):
            row_str = row_str+prefix+"R"+str(row)+"_C"+str(col)
         else:
            row_str = row_str+prefix+"R"+str(row)+"_C"+str(col)+","
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make NxN Transpose Matrix String
def makeNXNTranMatString(prefix,num_links,indent):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for row in range(1,num_links+1):
      row_str = ""
      for col in range(1,num_links+1):
         last_element_bool = (row==num_links)&(col==num_links)
         if (last_element_bool):
            row_str = row_str+prefix+"R"+str(col)+"_J"+str(row)
         else:
            row_str = row_str+prefix+"R"+str(col)+"_J"+str(row)+","
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make 6-D Vector Conditional Assignment String
def make6Vec1CondAssignString(prefix,dim_list,cond_str,prefix_true,prefix_false,reset_bool):
   first_row_bool = True
   mat_str = ""
   for dim,d6 in enumerate(dim_list):
      assignment_true = ""
      if (reset_bool):
         assignment_true = assignment_true+"32'd0"
      else:
         assignment_true = assignment_true+prefix_true+d6
      asn_str = ""
      asn_str = asn_str+"assign "+prefix+d6+" = "+cond_str+" ? "+assignment_true+" : "+prefix_false+d6+";"
      if (first_row_bool):
         mat_str = mat_str+indent+asn_str
      else:
         mat_str = mat_str+"\n"+indent+asn_str
      first_row_bool = False
   return mat_str

# Make NxN Matrix 1-Conditional Assignment String
def makeNXNMat1CondAssignString(prefix,num_links,cond_str,prefix_true,prefix_false,suffix_JorC,suffix_true_JorC,suffix_false_JorC,byrow_bool):
   first_row_bool = True
   mat_str = ""
   for row in range(num_links,0,-1):
      for col in range(1,num_links+1):
         asn_str = ""
         assignee = ""
         assign_t = ""
         assign_f = ""
         if (byrow_bool):
            assignee = assignee+prefix+"R"+str(row)+"_"+suffix_JorC+str(col)
            assign_t = prefix_true+"R"+str(row)+"_"+suffix_true_JorC+str(col)
            assign_f = prefix_false+"R"+str(row)+"_"+suffix_false_JorC+str(col)
         else:
            assignee = assignee+prefix+"R"+str(col)+"_"+suffix_JorC+str(num_links+1-row)
            assign_t = prefix_true+"R"+str(col)+"_"+suffix_true_JorC+str(num_links+1-row)
            assign_f = prefix_false+"R"+str(col)+"_"+suffix_false_JorC+str(num_links+1-row)
         asn_str = asn_str+"assign "+assignee+" = "+cond_str+" ? "+assign_t+" : "+assign_f+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

# Make Conditional String List for NxN Matrix 2-Conditional Assignment String
def makeNVec2CondStringList(num_links,c1_str,c1_type,c1_start,c1_end,c2_str,c2_type,c2_start,c2_end):
   last_row_bool = False
   con_str_list = []
   c2_counter = c2_start
   for row in range(num_links,0,-1):
      last_row_bool = (row == 1)
      con_str = ""
      if (c1_type == "last")&(last_row_bool):
         c1_val = c1_end
      else:
         c1_val = c1_start
      c2_val = c2_counter
      con_str = con_str+"(("+c1_str+str(c1_val)+")&&("+c2_str+str(c2_val)+"))"
      c2_counter = c2_counter-1
      con_str_list.append(con_str)
   return con_str_list

# Make NxN Matrix 2-Conditional Assignment String
def makeNXNMat2CondAssignString(prefix,num_links,cond_str_list,prefix_true,prefix_false):
   first_row_bool = True
   mat_str = ""
   row_index = 0
   for row in range(num_links,0,-1):
      cond_str = ""
      cond_str = cond_str_list[row_index]
      for col in range(1,num_links+1):
         asn_str = ""
         asn_str = asn_str+"assign "+prefix+"R"+str(row)+"_C"+str(col)+" = "+cond_str+" ? "+prefix_true+str(col)+" : "+prefix_false+"R"+str(row)+"_C"+str(col)+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
      row_index+=1
   return mat_str

# Make 6xN Matrix 1-Conditional Assignment String
def make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,reset_bool):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_links+1):
      for dim,d6 in enumerate(dim_list):
         assignment_true = ""
         if (reset_bool):
            assignment_true = assignment_true+"32'd0"
         else:
            assignment_true = assignment_true+prefix_true+d6+"_J"+str(row)
         asn_str = ""
         asn_str = asn_str+"assign "+prefix+d6+"_J"+str(row)+" = "+cond_str+" ? "+assignment_true+" : "+prefix_false+d6+"_J"+str(row)+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

# Make 6-D Vector Nonblocking Assignment String
def make6VecNonblockingAssignmentString(prefix,dim_list,zero_bool,prefix_in):
   first_row_bool = True
   mat_str = ""
   for dim,d6 in enumerate(dim_list):
      assignment = ""
      if (zero_bool):
         assignment = assignment+"32'd0"
      else:
         assignment = assignment+prefix_in+d6
      asn_str = ""
      asn_str = asn_str+prefix+d6+" <= "+assignment+";"
      if (first_row_bool):
         mat_str = mat_str+indent+asn_str
      else:
         mat_str = mat_str+"\n"+indent+asn_str
      first_row_bool = False
   return mat_str

# Make NxN Matrix Nonblocking Assignment String
def makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_links+1):
      for col in range(1,num_links+1):
         assignment = ""
         if (zero_bool):
            assignment = assignment+"32'd0"
         else:
            assignment = assignment+prefix_in+"R"+str(row)+"_C"+str(col)
         asn_str = ""
         asn_str = asn_str+prefix+"R"+str(row)+"_C"+str(col)+" <= "+assignment+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

# Make 6xN Matrix Nonblocking Assignment String
def make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_links+1):
      for dim,d6 in enumerate(dim_list):
         assignment = ""
         if (zero_bool):
            assignment = assignment+"32'd0"
         else:
            assignment = assignment+prefix_in+d6+"_J"+str(row)
         asn_str = ""
         asn_str = asn_str+prefix+d6+"_J"+str(row)+" <= "+assignment+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

# Make N-D Vector Conditional Boolean Assignment String
def makeNVec1CondBooleanAssignString(prefix,num_links,cond_str):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_links+1):
      asn_str = ""
      asn_str = asn_str+"assign "+prefix+str(row)+" = ("+cond_str+str(row)+") ? 1 : 0;"
      if (first_row_bool):
         mat_str = mat_str+indent+asn_str
      else:
         mat_str = mat_str+"\n"+indent+asn_str
      first_row_bool = False
   return mat_str

# Make 6-D Vector Port Assignment String
def make6VecPortAssignmentString(dim_list,port_prefix,wire_prefix,wire_suffix):
   last_element_bool = False
   vec_str = ""
   for dim,d6 in enumerate(dim_list):
      last_element_bool = (dim == len(dim_list)-1)
      vec_str = vec_str+"."+port_prefix+d6+"("+wire_prefix+d6+wire_suffix+")"
      if (last_element_bool):
         vec_str = vec_str+""
      else:
         vec_str = vec_str+","
   return vec_str

# Make Inverse Dynamics Processing Element
def makeIDPE(dim_list):
   mat_str = ""
   mat_str = mat_str+indent+"wire ["+str(vw.get_bitwidth_array_def(num_links))+":0] link_in_3bits;"+"\n"
   mat_str = mat_str+indent+"assign link_in_3bits = link_in_reg;"+"\n"
   mat_str = mat_str+indent+"rneabpx#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
   mat_str = mat_str+indent+"   uut("+"\n"
   mat_str = mat_str+indent+"   // link_in"+"\n"
   mat_str = mat_str+indent+"   .link_in(link_in_3bits),"+"\n"
   mat_str = mat_str+indent+"   // sin(q) and cos(q)"+"\n"
   mat_str = mat_str+indent+"   .sinq_curr_in(sinq_curr_mux),.cosq_curr_in(cosq_curr_mux),"+"\n"
   mat_str = mat_str+indent+"   // f_curr_vec_in"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"f_curr_vec_in_","f_curr_upd_mux_","")
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // f_prev_vec_in"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"f_prev_vec_in_","f_prev_vec_reg_","")
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // tau_curr_out"+"\n"
   mat_str = mat_str+indent+"   .tau_curr_out(tau_curr_out),"+"\n"
   mat_str = mat_str+indent+"   // f_prev_upd_vec_out"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"f_prev_upd_vec_out_","f_prev_upd_vec_out_","")
   mat_str = mat_str+indent+"   "+vec_str+"\n"
   mat_str = mat_str+indent+"   );"
   return mat_str

# Make Inverse Dynamics Gradient wrt q Processing Element
def makeGradIDdqPE(link,dim_list):
   j_str = "_J"+str(link)
   mat_str = ""
   mat_str = mat_str+indent+"// Joint "+str(link)+"\n"
   mat_str = mat_str+indent+"dqbpijx#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
   mat_str = mat_str+indent+"   dqbpi"+str(link)+"("+"\n"
   mat_str = mat_str+indent+"   // link_in"+"\n"
   mat_str = mat_str+indent+"   .link_in(link_in_3bits),"+"\n"
   mat_str = mat_str+indent+"   // sin(q) and cos(q)"+"\n"
   mat_str = mat_str+indent+"   .sinq_in(sinq_curr_mux),.cosq_in(cosq_curr_mux),"+"\n"
   mat_str = mat_str+indent+"   // fcurr_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"fcurr_in_","f_curr_upd_mux_","")
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dfdq_curr_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdq_curr_in_","dfidq_curr_oupd_mat_mux_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // fcross boolean"+"\n"
   mat_str = mat_str+indent+"   .fcross(fx"+str(link)+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdq_prev_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdq_prev_in_","dfidq_prev_mat_reg_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dtau_dq_out"+"\n"
   mat_str = mat_str+indent+"   .dtau_dq_out(dtauidq_curr_vec_out_J"+str(link)+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdq_prev_out, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdq_prev_out_","dfidq_prev_upd_mat_out_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+"\n"
   mat_str = mat_str+indent+"   );"
   return mat_str

# Make NxN Matrix Transpose Port Assignment String
def makeNXNMatTransposePortAssignmentString(num_links,indent,port_prefix,wire_prefix):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for row in range(1,num_links+1):
      row_str = ""
      for col in range(1,num_links+1):
         last_element_bool = (row==num_links)&(col==num_links)
         row_str = row_str+"."+port_prefix+"C"+str(row)+"_R"+str(col)+"("+wire_prefix+"R"+str(col)+"_C"+str(row)+")"
         if (last_element_bool):
            row_str = row_str+""
         else:
            row_str = row_str+","
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make N-D Vector Port Assignment String
def makeNVecPortAssignmentString(num_links,port_prefix,wire_prefix,wire_suffix):
   last_element_bool = False
   vec_str = ""
   for link in range(1,num_links+1):
      last_element_bool = (link==num_links)
      vec_str = vec_str+"."+port_prefix+"R"+str(link)+"("+wire_prefix+"R"+str(link)+wire_suffix+")"
      if (last_element_bool):
         vec_str = vec_str+""
      else:
         vec_str = vec_str+","
   return vec_str

# Make Inverse Dynamics Gradient wrt qd Processing Element
def makeGradIDdqdPE(link,num_links,dim_list):
   j_str = "_J"+str(link)
   mat_str = ""
   mat_str = mat_str+indent+"// Joint "+str(link)+"\n"
   mat_str = mat_str+indent+"dqdbpijx#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
   mat_str = mat_str+indent+"   dqdbpi"+str(link)+"("+"\n"
   mat_str = mat_str+indent+"   // link_in"+"\n"
   mat_str = mat_str+indent+"   .link_in(link_in_3bits),"+"\n"
   mat_str = mat_str+indent+"   // sin(q) and cos(q)"+"\n"
   mat_str = mat_str+indent+"   .sinq_in(sinq_curr_mux),.cosq_in(cosq_curr_mux),"+"\n"
   mat_str = mat_str+indent+"   // dfdqd_curr_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdqd_curr_in_","dfidqd_curr_oupd_mat_mux_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dfdqd_prev_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdqd_prev_in_","dfidqd_prev_mat_reg_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dtau_dqd_out"+"\n"
   mat_str = mat_str+indent+"   .dtau_dqd_out(dtauidqd_curr_vec_out_J"+str(link)+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdqd_prev_out, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdqd_prev_out_","dfidqd_prev_upd_mat_out_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // minv boolean"+"\n"
   mat_str = mat_str+indent+"   .minv(minv_bool_reg),"+"\n"
   mat_str = mat_str+indent+"   // minvm_in"+"\n"
   minv_str = makeNXNMatTransposePortAssignmentString(num_links,"      ","minvm_in_","minv_prev_vec_reg_")
   mat_str = mat_str+minv_str+","+"\n"
   mat_str = mat_str+indent+"   // tau_vec_in"+"\n"
   vec_str = makeNVecPortAssignmentString(num_links,"tau_vec_in_","tau_vec_in_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // minv_vec_out"+"\n"
   vec_str = makeNVecPortAssignmentString(num_links,"minv_vec_out_","minv_vec_out_",j_str)
   mat_str = mat_str+indent+"   "+vec_str+"\n"
   mat_str = mat_str+indent+"   );"
   return mat_str

# Make NxN Matrix Assignment String
def makeNXNMatAssignString(prefix,num_links,prefix_in,suffix_JorC,suffix_in_JorC,byrow_bool):
   first_row_bool = True
   mat_str = ""
   for row in range(num_links,0,-1):
      for col in range(1,num_links+1):
         asn_str = ""
         assignee = ""
         assigner = ""
         if (byrow_bool):
            assignee = assignee+prefix+"R"+str(num_links+1-row)+"_"+suffix_JorC+str(col)
            assigner = prefix_in+"R"+str(num_links+1-row)+"_"+suffix_in_JorC+str(col)
         else:
            assignee = assignee+prefix+"R"+str(col)+"_"+suffix_JorC+str(num_links+1-row)
            assigner = prefix_in+"R"+str(col)+"_"+suffix_in_JorC+str(num_links+1-row)
         asn_str = asn_str+"assign "+assignee+" = "+assigner+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

#-------------------------------------------------------------------------------
bproc_file.write("`timescale 1ns / 1ps"+"\n")
bproc_file.write("\n")
bproc_file.write("// Backward Pass Row Unit with RNEA and Minv"+"\n")
bproc_file.write("\n")
bproc_file.write("//------------------------------------------------------------------------------"+"\n")
bproc_file.write("// bproc Module"+"\n")
bproc_file.write("//------------------------------------------------------------------------------"+"\n")
bproc_file.write("module bproc#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)("+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // clock"+"\n")
bproc_file.write("   input  clk,"+"\n")
bproc_file.write("   // reset"+"\n")
bproc_file.write("   input reset,"+"\n")
bproc_file.write("   // get_data"+"\n")
bproc_file.write("   input get_data,"+"\n")
bproc_file.write("   // sin(q) and cos(q)"+"\n")
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_prev_in,cosq_prev_in,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // f_prev_vec_in"+"\n")
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "f_prev_vec_in_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv_prev_vec_in"+"\n")
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "minv_prev_vec_in_"
vec_str = makeNVecString(prefix,num_links,"C")
bproc_file.write(vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // dfidq_prev_mat_in"+"\n")
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_prev_mat_in_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // dfidqd_prev_mat_in"+"\n")
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_prev_mat_in_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // output_ready"+"\n")
bproc_file.write("   output output_ready,"+"\n")
bproc_file.write("   // dummy_output"+"\n")
bproc_file.write("   output dummy_output,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv_dtaudq_out"+"\n")
bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "minv_dtaudq_out_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv_dtaudqd_out"+"\n")
bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "minv_dtaudqd_out_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+"\n")
bproc_file.write("   );"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // external wires and state"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // registers"+"\n")
bproc_file.write("   reg get_data_reg;"+"\n")
bproc_file.write("   reg output_ready_reg;"+"\n")
bproc_file.write("   reg dummy_output_reg;"+"\n")
bproc_file.write("   reg minv_bool_reg;"+"\n")
bproc_file.write("   reg dqin_bool_reg;"+"\n")
bproc_file.write("   reg ["+str(vw.get_bitwidth_array_def(num_links+2))+":0]"+"\n")
bproc_file.write("      link_in_reg;"+"\n")
bproc_file.write("   reg [2:0]"+"\n")
bproc_file.write("      state_reg;"+"\n")
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_prev_reg,cosq_prev_reg,"+"\n")
bproc_file.write("      sinq_curr_reg,cosq_curr_reg,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("      ")
prefix = "f_prev_vec_reg_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "minv_prev_vec_reg_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "dtauidq_curr_vec_reg_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "dtauidqd_curr_vec_reg_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "minv_out_dtaudq_reg_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "f_curr_upd_reg_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_curr_oupd_mat_reg_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_curr_oupd_mat_reg_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_prev_mat_reg_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_prev_mat_reg_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // next"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire get_data_next;"+"\n")
bproc_file.write("   wire output_ready_next;"+"\n")
bproc_file.write("   wire dummy_output_next;"+"\n")
bproc_file.write("   wire minv_bool_next;"+"\n")
bproc_file.write("   wire dqin_bool_next;"+"\n")
bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+2))+":0]"+"\n")
bproc_file.write("      link_in_next;"+"\n")
bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+2))+":0]"+"\n")
bproc_file.write("      link_in_sub1;"+"\n")
bproc_file.write("   wire [2:0]"+"\n")
bproc_file.write("      state_next;"+"\n")
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_prev_next,cosq_prev_next,"+"\n")
bproc_file.write("      sinq_curr_next,cosq_curr_next,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("      ")
prefix = "f_prev_vec_next_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "minv_prev_vec_next_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dtauidq_curr_vec_next_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dtauidqd_curr_vec_next_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "minv_out_dtaudq_next_"
indent = "      "
mat_str = makeNXNMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "f_curr_upd_next_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_curr_oupd_mat_next_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_curr_oupd_mat_next_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_prev_mat_next_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_prev_mat_next_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // out"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_prev_upd_mat_out_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_prev_upd_mat_out_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // mux"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_prev_mux,cosq_prev_mux,"+"\n")
bproc_file.write("      sinq_curr_mux,cosq_curr_mux;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "f_curr_upd_mux_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidq_curr_oupd_mat_mux_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "dfidqd_curr_oupd_mat_mux_"
indent = "      "
mat_str = make6XNMatString(prefix,dim_list,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // RNEA"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      tau_curr_out;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "f_prev_upd_vec_out_"
vec_str = make6VecString(prefix,dim_list)
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "tau_vec_in_"
indent = "      "
mat_str = makeNXNTranMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
prefix = "minv_vec_out_"
indent = "      "
mat_str = makeNXNTranMatString(prefix,num_links,indent)
bproc_file.write(mat_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // dtau"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "dtauidq_curr_vec_out_"
vec_str = makeNVecString(prefix,num_links,"J")
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      ")
prefix = "dtauidqd_curr_vec_out_"
vec_str = makeNVecString(prefix,num_links,"J")
bproc_file.write(vec_str+";"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // external assignments"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // inputs"+"\n")
bproc_file.write("   assign get_data_next = get_data;"+"\n")
bproc_file.write("   // output"+"\n")
bproc_file.write("   assign output_ready = output_ready_reg;"+"\n")
bproc_file.write("   assign output_ready_next = ((state_reg == 3'd0)&&(get_data == 1))                        ? 1 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd0)&&(get_data == 0))                        ? 0 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg != {bitwidth}'d2)) ? 1 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 0)&&(link_in_reg != {bitwidth}'d2)) ? 0 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg == {bitwidth}'d2)) ? 0 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 0)&&(link_in_reg == {bitwidth}'d2)) ? 0 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                              ((state_reg == 3'd2)&&(get_data == 1))                        ? 0 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd2)&&(get_data == 0))                        ? 0 :"+"\n")
bproc_file.write("                               (state_reg == 3'd3)                                          ? 0 :"+"\n")
bproc_file.write("                               (state_reg == 3'd4)                                          ? 1 : output_ready_reg;"+"\n")
bproc_file.write("   assign dummy_output = dummy_output_reg;"+"\n")
bproc_file.write("   assign dummy_output_next = ((state_reg == 3'd0)&&(get_data == 1)&&(link_in_reg == {bitwidth}'d{num_links_plus2})) ? 1 :".format(bitwidth=vw.get_bitwidth(num_links+2), num_links_plus2=num_links+2)+"\n")
bproc_file.write("                              ((state_reg == 3'd0)&&(get_data == 1)&&(link_in_reg != {bitwidth}'d{num_links_plus2})) ? 0 :".format(bitwidth=vw.get_bitwidth(num_links+2), num_links_plus2=num_links+2)+"\n")
bproc_file.write("                              ((state_reg == 3'd0)&&(get_data == 0)) ? 0 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg == {bitwidth}'d1)) ? 1 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg != {bitwidth}'d1)) ? 0 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 0)) ? 0 : dummy_output_reg;"+"\n")
bproc_file.write("   // minv"+"\n")
bproc_file.write("   assign minv_bool_next =  (state_reg == 3'd3) ? 1 :"+"\n")
bproc_file.write("                            (state_reg == 3'd4) ? 1 : 0;"+"\n")
bproc_file.write("   assign dqin_bool_next =  (state_reg == 3'd3) ? 1 :"+"\n")
bproc_file.write("                            (state_reg == 3'd4) ? 0 : 0;"+"\n")
bproc_file.write("   // link"+"\n")
bproc_file.write("   assign link_in_sub1   = link_in_reg - 1;"+"\n")
bproc_file.write("   assign link_in_next   = ((state_reg == 3'd0)&&(get_data == 1))                        ? link_in_sub1 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd0)&&(get_data == 0))                        ? link_in_reg  :"+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg != {bitwidth}'d2)) ? link_in_sub1 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 0)&&(link_in_reg != {bitwidth}'d2)) ? link_in_reg  :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg == {bitwidth}'d2)) ? link_in_sub1 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 0)&&(link_in_reg == {bitwidth}'d2)) ? link_in_reg  :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd2)&&(get_data == 1))                        ? link_in_sub1 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd2)&&(get_data == 0))                        ? link_in_reg  :"+"\n")
bproc_file.write("                            (state_reg == 3'd3)                                          ? link_in_reg  :"+"\n")
bproc_file.write("                            (state_reg == 3'd4)                                          ? {bitwidth}'d{num_links_plus2}         : link_in_reg;".format(bitwidth=vw.get_bitwidth(num_links+2), num_links_plus2=num_links+2)+"\n")
bproc_file.write("   // state"+"\n")
bproc_file.write("   assign state_next     = ((state_reg == 3'd0)&&(get_data == 1)) ? 3'd1 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd0)&&(get_data == 0)) ? 3'd0 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg != {bitwidth}'d2)) ? 3'd1 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 0)&&(link_in_reg != {bitwidth}'d2)) ? 3'd0 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 1)&&(link_in_reg == {bitwidth}'d2)) ? 3'd3 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 0)&&(link_in_reg == {bitwidth}'d2)) ? 3'd2 :".format(bitwidth=vw.get_bitwidth(num_links+2))+"\n")
bproc_file.write("                           ((state_reg == 3'd2)&&(get_data == 1)) ? 3'd3 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd2)&&(get_data == 0)) ? 3'd2 :"+"\n")
bproc_file.write("                            (state_reg == 3'd3) ? 3'd4 :"+"\n")
bproc_file.write("                            (state_reg == 3'd4) ? 3'd0 : state_reg;"+"\n")
bproc_file.write("   // sinq, cosq prev"+"\n")
bproc_file.write("   assign sinq_prev_next     = (get_data == 1) ? sinq_prev_in : sinq_prev_reg;"+"\n")
bproc_file.write("   assign cosq_prev_next     = (get_data == 1) ? cosq_prev_in : cosq_prev_reg;"+"\n")
bproc_file.write("   // sinq, cosq curr"+"\n")
bproc_file.write("   assign sinq_curr_next     = (get_data == 1) ? sinq_prev_reg : sinq_curr_reg;"+"\n")
bproc_file.write("   assign cosq_curr_next     = (get_data == 1) ? cosq_prev_reg : cosq_curr_reg;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // f prev raw"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_prev_vec_next_"
cond_str = "(get_data == 1)"
prefix_true = "f_prev_vec_in_"
prefix_false = "f_prev_vec_reg_"
indent = "   "
mat_str = make6Vec1CondAssignString(prefix,dim_list,cond_str,prefix_true,prefix_false,False)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv prev"+"\n")
#-------------------------------------------------------------------------------
c1_str = "get_data == "
c1_type = "constant"
c1_start = 1
c1_end = 1
c2_str = "link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"
c2_type = "countdown"
c2_start = num_links+2
c2_end = 1+2
cond_str_list = makeNVec2CondStringList(num_links,c1_str,c1_type,c1_start,c1_end,c2_str,c2_type,c2_start,c2_end)
prefix = "minv_prev_vec_next_"
prefix_true = "minv_prev_vec_in_C"
prefix_false = "minv_prev_vec_reg_"
indent = "   "
mat_str = makeNXNMat2CondAssignString(prefix,num_links,cond_str_list,prefix_true,prefix_false)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // dtau/dq curr"+"\n")
#-------------------------------------------------------------------------------
c1_str = "state_reg == 3'd"
c1_type = "last"
c1_start = 1
c1_end = 3
c2_str = "link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"
c2_type = "countdown"
c2_start = num_links
c2_end = 1
cond_str_list = makeNVec2CondStringList(num_links,c1_str,c1_type,c1_start,c1_end,c2_str,c2_type,c2_start,c2_end)
prefix = "dtauidq_curr_vec_next_"
prefix_true = "dtauidq_curr_vec_out_J"
prefix_false = "dtauidq_curr_vec_reg_"
indent = "   "
mat_str = makeNXNMat2CondAssignString(prefix,num_links,cond_str_list,prefix_true,prefix_false)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // dtau/dqd curr"+"\n")
#-------------------------------------------------------------------------------
c1_str = "state_reg == 3'd"
c1_type = "last"
c1_start = 1
c1_end = 3
c2_str = "link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"
c2_type = "countdown"
c2_start = num_links
c2_end = 1
cond_str_list = makeNVec2CondStringList(num_links,c1_str,c1_type,c1_start,c1_end,c2_str,c2_type,c2_start,c2_end)
prefix = "dtauidqd_curr_vec_next_"
prefix_true = "dtauidqd_curr_vec_out_J"
prefix_false = "dtauidqd_curr_vec_reg_"
indent = "   "
mat_str = makeNXNMat2CondAssignString(prefix,num_links,cond_str_list,prefix_true,prefix_false)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv dtau/dq out"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_out_dtaudq_next_"
cond_str = "(state_reg == 3'd4)"
prefix_true = "minv_vec_out_"
prefix_false = "minv_out_dtaudq_reg_"
indent = "   "
mat_str = makeNXNMat1CondAssignString(prefix,num_links,cond_str,prefix_true,prefix_false,"C","J","C",True)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // f curr upd"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_curr_upd_next_"
cond_str = "(state_reg == 3'd1)"
prefix_true = "f_prev_upd_vec_out_"
prefix_false = "f_curr_upd_reg_"
indent = "   "
mat_str = make6Vec1CondAssignString(prefix,dim_list,cond_str,prefix_true,prefix_false,False)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // df/dq curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_curr_oupd_mat_next_"
cond_str = "(state_reg == 3'd1)"
prefix_true = "dfidq_prev_upd_mat_out_"
prefix_false = "dfidq_curr_oupd_mat_reg_"
indent = "   "
mat_str = make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,False)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // df/dqd curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_curr_oupd_mat_next_"
cond_str = "(state_reg == 3'd1)"
prefix_true = "dfidqd_prev_upd_mat_out_"
prefix_false = "dfidqd_curr_oupd_mat_reg_"
indent = "   "
mat_str = make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,False)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // df/dq prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_prev_mat_next_"
cond_str = "(get_data == 1)"
prefix_true = "dfidq_prev_mat_in_"
prefix_false = "dfidq_prev_mat_reg_"
indent = "   "
mat_str = make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,False)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // df/dqd prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_prev_mat_next_"
cond_str = "(get_data == 1)"
prefix_true = "dfidqd_prev_mat_in_"
prefix_false = "dfidqd_prev_mat_reg_"
indent = "   "
mat_str = make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,False)
bproc_file.write(mat_str+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // external registers"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   always @ (posedge clk or posedge reset)"+"\n")
bproc_file.write("   begin"+"\n")
bproc_file.write("      if (reset)"+"\n")
bproc_file.write("      begin"+"\n")
bproc_file.write("         // inputs"+"\n")
bproc_file.write("         get_data_reg <= 0;"+"\n")
bproc_file.write("         // output"+"\n")
bproc_file.write("         output_ready_reg <= 0;"+"\n")
bproc_file.write("         dummy_output_reg <= 0;"+"\n")
bproc_file.write("         // minv"+"\n")
bproc_file.write("         minv_bool_reg <= 0;"+"\n")
bproc_file.write("         dqin_bool_reg <= 0;"+"\n")
bproc_file.write("         // link"+"\n")
bproc_file.write("         link_in_reg <= "+str(vw.get_bitwidth(num_links+2))+"'d"+str(num_links+2)+";"+"\n")
bproc_file.write("         // state"+"\n")
bproc_file.write("         state_reg   <= 3'd0;"+"\n")
bproc_file.write("         // sinq, cosq prev"+"\n")
bproc_file.write("         sinq_prev_reg <= 32'd0;"+"\n")
bproc_file.write("         cosq_prev_reg <= 32'd0;"+"\n")
bproc_file.write("         // sinq, cosq curr"+"\n")
bproc_file.write("         sinq_curr_reg <= 32'd0;"+"\n")
bproc_file.write("         cosq_curr_reg <= 32'd0;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // f prev raw"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_prev_vec_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = make6VecNonblockingAssignmentString(prefix,dim_list,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // minv prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_prev_vec_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // dtau/dq curr"+"\n")
#-------------------------------------------------------------------------------
prefix = "dtauidq_curr_vec_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // dtau/dqd curr"+"\n")
#-------------------------------------------------------------------------------
prefix = "dtauidqd_curr_vec_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // minv dtau/dq out"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_out_dtaudq_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // f curr upd"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_curr_upd_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = make6VecNonblockingAssignmentString(prefix,dim_list,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dq curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_curr_oupd_mat_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dqd curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_curr_oupd_mat_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dq prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_prev_mat_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dqd prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_prev_mat_reg_"
zero_bool = True
prefix_in = ""
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("      end"+"\n")
bproc_file.write("      else"+"\n")
bproc_file.write("      begin"+"\n")
bproc_file.write("         // inputs"+"\n")
bproc_file.write("         get_data_reg <= get_data_next;"+"\n")
bproc_file.write("         // output"+"\n")
bproc_file.write("         output_ready_reg <= output_ready_next;"+"\n")
bproc_file.write("         dummy_output_reg <= dummy_output_next;"+"\n")
bproc_file.write("         // minv"+"\n")
bproc_file.write("         minv_bool_reg <= minv_bool_next;"+"\n")
bproc_file.write("         dqin_bool_reg <= dqin_bool_next;"+"\n")
bproc_file.write("         // link"+"\n")
bproc_file.write("         link_in_reg <= link_in_next;"+"\n")
bproc_file.write("         // state"+"\n")
bproc_file.write("         state_reg   <= state_next;"+"\n")
bproc_file.write("         // sinq, cosq prev"+"\n")
bproc_file.write("         sinq_prev_reg <= sinq_prev_next;"+"\n")
bproc_file.write("         cosq_prev_reg <= cosq_prev_next;"+"\n")
bproc_file.write("         // sinq, cosq curr"+"\n")
bproc_file.write("         sinq_curr_reg <= sinq_curr_next;"+"\n")
bproc_file.write("         cosq_curr_reg <= cosq_curr_next;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // f prev raw"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_prev_vec_reg_"
zero_bool = False
prefix_in = "f_prev_vec_next_"
indent = "         "
mat_str = make6VecNonblockingAssignmentString(prefix,dim_list,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // minv prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_prev_vec_reg_"
zero_bool = False
prefix_in = "minv_prev_vec_next_"
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // dtau/dq curr"+"\n")
#-------------------------------------------------------------------------------
prefix = "dtauidq_curr_vec_reg_"
zero_bool = False
prefix_in = "dtauidq_curr_vec_next_"
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // dtau/dqd curr"+"\n")
#-------------------------------------------------------------------------------
prefix = "dtauidqd_curr_vec_reg_"
zero_bool = False
prefix_in = "dtauidqd_curr_vec_next_"
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // minv dtau/dq out"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_out_dtaudq_reg_"
zero_bool = False
prefix_in = "minv_out_dtaudq_next_"
indent = "         "
mat_str = makeNXNMatNonblockingAssignmentString(prefix,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // f curr upd"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_curr_upd_reg_"
zero_bool = False
prefix_in = "f_curr_upd_next_"
indent = "         "
mat_str = make6VecNonblockingAssignmentString(prefix,dim_list,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dq curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_curr_oupd_mat_reg_"
zero_bool = False
prefix_in = "dfidq_curr_oupd_mat_next_"
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dqd curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_curr_oupd_mat_reg_"
zero_bool = False
prefix_in = "dfidqd_curr_oupd_mat_next_"
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dq prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_prev_mat_reg_"
zero_bool = False
prefix_in = "dfidq_prev_mat_next_"
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         // df/dqd prev"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_prev_mat_reg_"
zero_bool = False
prefix_in = "dfidqd_prev_mat_next_"
indent = "         "
mat_str = make6XNMatNonblockingAssignmentString(prefix,dim_list,num_links,zero_bool,prefix_in)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("      end"+"\n")
bproc_file.write("   end"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // fcross booleans"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   wire ")
prefix = "fx"
vec_str = makeNVecString(prefix,num_links,"")
bproc_file.write(vec_str+";"+"\n")
#-------------------------------------------------------------------------------
prefix = "fx"
cond_str = "link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"
indent = "   "
mat_str = makeNVec1CondBooleanAssignString(prefix,num_links,cond_str)
bproc_file.write(mat_str+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // input muxes"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // sinq, cosq curr"+"\n")
bproc_file.write("   assign sinq_curr_mux     = (link_in_reg == {bitwidth}'d{num_links_incr}) ? 32'd0 : sinq_curr_reg;".format(bitwidth=vw.get_bitwidth(num_links+2), num_links_incr=num_links+1)+"\n")
bproc_file.write("   assign cosq_curr_mux     = (link_in_reg == {bitwidth}'d{num_links_incr}) ? 32'd0 : cosq_curr_reg;".format(bitwidth=vw.get_bitwidth(num_links+2), num_links_incr=num_links+1)+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // f curr upd"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_curr_upd_mux_"
cond_str = "(link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"+str(num_links+1)+")"
prefix_true = ""
prefix_false = "f_curr_upd_reg_"
indent = "   "
mat_str = make6Vec1CondAssignString(prefix,dim_list,cond_str,prefix_true,prefix_false,True)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // df/dq curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidq_curr_oupd_mat_mux_"
cond_str = "(link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"+str(num_links+1)+")"
prefix_true = ""
prefix_false = "dfidq_curr_oupd_mat_reg_"
indent = "   "
mat_str = make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,True)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // df/dqd curr oupdated"+"\n")
#-------------------------------------------------------------------------------
prefix = "dfidqd_curr_oupd_mat_mux_"
cond_str = "(link_in_reg == "+str(vw.get_bitwidth(num_links+2))+"'d"+str(num_links+1)+")"
prefix_true = ""
prefix_false = "dfidqd_curr_oupd_mat_reg_"
indent = "   "
mat_str = make6XNMat1CondAssignString(prefix,dim_list,num_links,cond_str,prefix_true,prefix_false,True)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv"+"\n")
#-------------------------------------------------------------------------------
prefix = "tau_vec_in_"
cond_str = "(dqin_bool_reg)"
prefix_true = "dtauidq_curr_vec_reg_"
prefix_false = "dtauidqd_curr_vec_reg_"
indent = "   "
mat_str = makeNXNMat1CondAssignString(prefix,num_links,cond_str,prefix_true,prefix_false,"J","C","C",False)
bproc_file.write(mat_str+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // ID"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeIDPE(dim_list)
bproc_file.write(mat_str+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dID/dq"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
for link in range(1,num_links+1):
   mat_str = makeGradIDdqPE(link,dim_list)
   bproc_file.write(mat_str+"\n")
   bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dID/dqd"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
for link in range(1,num_links+1):
   mat_str = makeGradIDdqdPE(link,num_links,dim_list)
   bproc_file.write(mat_str+"\n")
   bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // outputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv dtau/dq out"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_dtaudq_out_"
prefix_in = "minv_out_dtaudq_reg_"
indent = "   "
mat_str = makeNXNMatAssignString(prefix,num_links,prefix_in,"C","C",True)
bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // minv dtau/dqd out"+"\n")
#-------------------------------------------------------------------------------
prefix = "minv_dtaudqd_out_"
prefix_in = "minv_vec_out_"
indent = "   "
mat_str = makeNXNMatAssignString(prefix,num_links,prefix_in,"C","J",True)
bproc_file.write(mat_str+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("endmodule"+"\n")
#-------------------------------------------------------------------------------

# Close file
bproc_file.close()
