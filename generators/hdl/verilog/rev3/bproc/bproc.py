from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file, num_PEs, block_size

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()

#-------- File management -------------------

fm = FileManager("bproc.v")
output_file_path = fm.get_output_file_path()
bproc_file = open(output_file_path, "w")

vw = VerilogWriter(bproc_file, dim_list, num_links)

#--------- Setting commonly used bitwidths -----

bitwidth_num_links = vw.get_bitwidth(num_links)
bitwidth_num_links_str = str(bitwidth_num_links)

#---------------------------------------------

#fproc = fpga_codegen.get_fproc(vw, num_PEs)
#print(fproc.gen_dfidq_prev_mat_out_ports_by_PE())

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

# Make N-D Vector String
def makeNVecString(prefix,suffix,num_N,tag_str):
   last_element_bool = False
   vec_str = ""
   for link in range(1,num_N+1):
      last_element_bool = (link==num_N)
      vec_str = vec_str+prefix+tag_str+str(link)+suffix
      if (last_element_bool):
         vec_str = vec_str+""
      else:
         vec_str = vec_str+","
   return vec_str

# Make 6xN Matrix String
def make6XNMatString(prefix,dim_list,num_N,indent):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for dim,d6 in enumerate(dim_list):
      row_str = ""
      for link in range(1,num_N+1):
         last_element_bool = (dim == len(dim_list)-1)&(link==num_N)
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
def makeNXNMatString(prefix,suffix,num_N,indent):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for row in range(1,num_N+1):
      row_str = ""
      for col in range(1,num_N+1):
         last_element_bool = (row==num_N)&(col==num_N)
         if (last_element_bool):
            row_str = row_str+prefix+"R"+str(row)+"_C"+str(col)+suffix
         else:
            row_str = row_str+prefix+"R"+str(row)+"_C"+str(col)+suffix+","
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make NxN Transpose Matrix String
def makeNXNTranMatString(prefix,suffix,num_N,indent):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for row in range(1,num_N+1):
      row_str = ""
      for col in range(1,num_N+1):
         last_element_bool = (row==num_N)&(col==num_N)
         if (last_element_bool):
            row_str = row_str+prefix+"R"+str(col)+"_C"+str(row)+suffix
         else:
            row_str = row_str+prefix+"R"+str(col)+"_C"+str(row)+suffix+","
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

# Make N-D Vector Conditional Assignment String
def makeNVec1CondAssignString(prefix,suffix,num_N,cond_str,prefix_true,prefix_false,reset_bool):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_N+1):
      assignment_true = ""
      if (reset_bool):
         assignment_true = assignment_true+"32'd0"
      else:
         assignment_true = assignment_true+prefix_true+"R"+str(row)+suffix
      asn_str = ""
      asn_str = asn_str+"assign "+prefix+"R"+str(row)+suffix+" = "+cond_str+" ? "+assignment_true+" : "+prefix_false+"R"+str(row)+suffix+";"
      if (first_row_bool):
         mat_str = mat_str+indent+asn_str
      else:
         mat_str = mat_str+"\n"+indent+asn_str
      first_row_bool = False
   return mat_str

# Make NxN Matrix 1-Conditional Assignment String
def makeNXNMat1CondAssignString(prefix,suffix,num_N,cond_str,prefix_true,prefix_false,suffix_JorC,suffix_true_JorC,suffix_false_JorC,byrow_bool):
   first_row_bool = True
   mat_str = ""
   for row in range(num_N,0,-1):
      for col in range(1,num_N+1):
         asn_str = ""
         assignee = ""
         assign_t = ""
         assign_f = ""
         if (byrow_bool):
            assignee = assignee+prefix+"R"+str(row)+"_"+suffix_JorC+str(col)+suffix
            assign_t = prefix_true+"R"+str(row)+"_"+suffix_true_JorC+str(col)+suffix
            assign_f = prefix_false+"R"+str(row)+"_"+suffix_false_JorC+str(col)+suffix
         else:
            assignee = assignee+prefix+"R"+str(col)+"_"+suffix_JorC+str(num_N+1-row)+suffix
            assign_t = prefix_true+"R"+str(col)+"_"+suffix_true_JorC+str(num_N+1-row)+suffix
            assign_f = prefix_false+"R"+str(col)+"_"+suffix_false_JorC+str(num_N+1-row)+suffix
         asn_str = asn_str+"assign "+assignee+" = "+cond_str+" ? "+assign_t+" : "+assign_f+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

# Make Conditional String List for N-D Vector 2-Conditional Assignment String
def makeNVec2CondStringList(num_N,c1_str,c1_type,c1_start,c1_end,c2_str,c2_type,c2_start,c2_end):
   last_row_bool = False
   con_str_list = []
   c2_counter = c2_start
   for row in range(num_N,0,-1):
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
def makeNXNMat2CondAssignString(prefix,num_N,cond_str_list,prefix_true,prefix_false):
   first_row_bool = True
   mat_str = ""
   row_index = 0
   for row in range(num_N,0,-1):
      cond_str = ""
      cond_str = cond_str_list[row_index]
      for col in range(1,num_N+1):
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
def make6XNMat1CondAssignString(prefix,dim_list,num_N,cond_str,prefix_true,prefix_false,reset_bool):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_N+1):
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

# Make N-D Vector Nonblocking Assignment String
def makeNVecNonblockingAssignmentString(prefix,suffix,num_N,dim_list,zero_bool,prefix_in):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_N+1):
      assignment = ""
      if (zero_bool):
         assignment = assignment+"32'd0"
      else:
         assignment = assignment+prefix_in+"R"+str(row)+suffix
      asn_str = ""
      asn_str = asn_str+prefix+"R"+str(row)+suffix+" <= "+assignment+";"
      if (first_row_bool):
         mat_str = mat_str+indent+asn_str
      else:
         mat_str = mat_str+"\n"+indent+asn_str
      first_row_bool = False
   return mat_str

# Make NxN Matrix Nonblocking Assignment String
def makeNXNMatNonblockingAssignmentString(prefix,suffix,num_N,zero_bool,prefix_in):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_N+1):
      for col in range(1,num_N+1):
         assignment = ""
         if (zero_bool):
            assignment = assignment+"32'd0"
         else:
            assignment = assignment+prefix_in+"R"+str(row)+"_C"+str(col)+suffix
         asn_str = ""
         asn_str = asn_str+prefix+"R"+str(row)+"_C"+str(col)+suffix+" <= "+assignment+";"
         if (first_row_bool):
            mat_str = mat_str+indent+asn_str
         else:
            mat_str = mat_str+"\n"+indent+asn_str
         first_row_bool = False
   return mat_str

# Make 6xN Matrix Nonblocking Assignment String
def make6XNMatNonblockingAssignmentString(prefix,dim_list,num_N,zero_bool,prefix_in):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_N+1):
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
def makeNVec1CondBooleanAssignString(prefix,num_N,cond_str):
   first_row_bool = True
   mat_str = ""
   for row in range(1,num_N+1):
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
def makeIDPE(suffix,dim_list):
   mat_str = ""
   mat_str = mat_str+indent+"rneabpx#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
   mat_str = mat_str+indent+"   uut("+"\n"
   mat_str = mat_str+indent+"   // link_in"+"\n"
   mat_str = mat_str+indent+"   .link_in(link_reg_rnea),"+"\n"
   mat_str = mat_str+indent+"   // sin(q) and cos(q)"+"\n"
   mat_str = mat_str+indent+"   .sinq_curr_in(sinq_val_reg_rnea),.cosq_curr_in(cosq_val_reg_rnea),"+"\n"
   mat_str = mat_str+indent+"   // f_curr_vec_in"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"f_curr_vec_in_","f_upd_curr_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // f_prev_vec_in"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"f_prev_vec_in_","f_prev_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // tau_curr_out"+"\n"
   mat_str = mat_str+indent+"   .tau_curr_out(tau_curr_out_rnea),"+"\n"
   mat_str = mat_str+indent+"   // f_prev_upd_vec_out"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"f_prev_upd_vec_out_","f_upd_prev_vec_out_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+"\n"
   mat_str = mat_str+indent+"   );"
   return mat_str

# Make Inverse Dynamics Gradient wrt q Processing Element
def makeGradIDdqPE(suffix,PEid,dim_list):
   PE_str = str(PEid)
   mat_str = ""
   mat_str = mat_str+indent+"// dqPE"+PE_str+""+"\n"
   mat_str = mat_str+indent+"dqbpijx#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
   mat_str = mat_str+indent+"   dqbpi"+PE_str+"("+"\n"
   mat_str = mat_str+indent+"   // link_in"+"\n"
   mat_str = mat_str+indent+"   .link_in(link_reg_dqPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // sin(q) and cos(q)"+"\n"
   mat_str = mat_str+indent+"   .sinq_in(sinq_val_reg_dqPE"+PE_str+"),.cosq_in(cosq_val_reg_dqPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // fcurr_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"fcurr_in_","f_upd_curr_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dfdq_curr_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdq_curr_in_","dfdq_upd_curr_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // fcross boolean"+"\n"
   mat_str = mat_str+indent+"   .fcross(fx_dqPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdq_prev_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdq_prev_in_","dfdq_prev_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dtau_dq_out"+"\n"
   mat_str = mat_str+indent+"   .dtau_dq_out(dtau_curr_out_dqPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdq_prev_out, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdq_prev_out_","dfdq_upd_prev_vec_out_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+"\n"
   mat_str = mat_str+indent+"   );"
   return mat_str

# Make NxN Matrix Transpose Port Assignment String
def makeNXNMatTransposePortAssignmentString(num_links,indent,port_prefix,wire_prefix,wire_suffix):
   first_row_bool = True
   last_element_bool = False
   mat_str = ""
   for row in range(1,num_links+1):
      row_str = ""
      for col in range(1,num_links+1):
         last_element_bool = (row==num_links)&(col==num_links)
         row_str = row_str+"."+port_prefix+"C"+str(row)+"_R"+str(col)+"("+wire_prefix+"R"+str(col)+"_C"+str(row)+wire_suffix+")"
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
def makeGradIDdqdPE(suffix,link,num_links,block_size,dim_list):
   PE_str = str(PEid)
   mat_str = ""
   mat_str = mat_str+indent+"// dqdPE"+PE_str+""+"\n"
   mat_str = mat_str+indent+"dqdbpijx#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
   mat_str = mat_str+indent+"   dqdbpi"+PE_str+"("+"\n"
   mat_str = mat_str+indent+"   // link_in"+"\n"
   mat_str = mat_str+indent+"   .link_in(link_reg_dqdPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // sin(q) and cos(q)"+"\n"
   mat_str = mat_str+indent+"   .sinq_in(sinq_val_reg_dqdPE"+PE_str+"),.cosq_in(cosq_val_reg_dqdPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdqd_curr_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdqd_curr_in_","dfdqd_upd_curr_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dfdqd_prev_in, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdqd_prev_in_","dfdqd_prev_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // dtau_dqd_out"+"\n"
   mat_str = mat_str+indent+"   .dtau_dqd_out(dtau_curr_out_dqdPE"+PE_str+"),"+"\n"
   mat_str = mat_str+indent+"   // dfdqd_prev_out, 6 values"+"\n"
   vec_str = make6VecPortAssignmentString(dim_list,"dfdqd_prev_out_","dfdqd_upd_prev_vec_out_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // minv boolean"+"\n"
   mat_str = mat_str+indent+"   .minv(minv_bool_reg),"+"\n"
   mat_str = mat_str+indent+"   // minvm_in"+"\n"
   minv_str = makeNXNMatTransposePortAssignmentString(block_size,"      ","minvm_in_","minv_block_reg_",suffix)
   mat_str = mat_str+minv_str+","+"\n"
   mat_str = mat_str+indent+"   // tau_vec_in"+"\n"
   vec_str = makeNVecPortAssignmentString(block_size,"tau_vec_in_","dtau_vec_reg_",suffix)
   mat_str = mat_str+indent+"   "+vec_str+","+"\n"
   mat_str = mat_str+indent+"   // minv_vec_out"+"\n"
   vec_str = makeNVecPortAssignmentString(block_size,"minv_vec_out_","minv_vec_out_",suffix)
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

# Write dq External Input Ports
def writeDQExternalInputPorts(PEid,output_file,dim_list,indent):
   PE_str = str(PEid)
   output_file.write("   // dqPE"+PE_str+""+"\n")
   output_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   output_file.write("      link_in_dqPE"+PE_str+","+"\n")
   output_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   output_file.write("      derv_in_dqPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      sinq_val_in_dqPE"+PE_str+",cosq_val_in_dqPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      f_upd_curr_vec_in_AX_dqPE"+PE_str+",f_upd_curr_vec_in_AY_dqPE"+PE_str+",f_upd_curr_vec_in_AZ_dqPE"+PE_str+",f_upd_curr_vec_in_LX_dqPE"+PE_str+",f_upd_curr_vec_in_LY_dqPE"+PE_str+",f_upd_curr_vec_in_LZ_dqPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      dfdq_prev_vec_in_AX_dqPE"+PE_str+",dfdq_prev_vec_in_AY_dqPE"+PE_str+",dfdq_prev_vec_in_AZ_dqPE"+PE_str+",dfdq_prev_vec_in_LX_dqPE"+PE_str+",dfdq_prev_vec_in_LY_dqPE"+PE_str+",dfdq_prev_vec_in_LZ_dqPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      dfdq_upd_curr_vec_in_AX_dqPE"+PE_str+",dfdq_upd_curr_vec_in_AY_dqPE"+PE_str+",dfdq_upd_curr_vec_in_AZ_dqPE"+PE_str+",dfdq_upd_curr_vec_in_LX_dqPE"+PE_str+",dfdq_upd_curr_vec_in_LY_dqPE"+PE_str+",dfdq_upd_curr_vec_in_LZ_dqPE"+PE_str+","+"\n")

# Write dqd External Input Ports
def writeDQDExternalInputPorts(PEid,output_file,dim_list,indent):
   PE_str = str(PEid)
   output_file.write("   // dqdPE"+PE_str+""+"\n")
   output_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   output_file.write("      link_in_dqdPE"+PE_str+","+"\n")
   output_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   output_file.write("      derv_in_dqdPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      sinq_val_in_dqdPE"+PE_str+",cosq_val_in_dqdPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      dfdqd_prev_vec_in_AX_dqdPE"+PE_str+",dfdqd_prev_vec_in_AY_dqdPE"+PE_str+",dfdqd_prev_vec_in_AZ_dqdPE"+PE_str+",dfdqd_prev_vec_in_LX_dqdPE"+PE_str+",dfdqd_prev_vec_in_LY_dqdPE"+PE_str+",dfdqd_prev_vec_in_LZ_dqdPE"+PE_str+","+"\n")
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      dfdqd_upd_curr_vec_in_AX_dqdPE"+PE_str+",dfdqd_upd_curr_vec_in_AY_dqdPE"+PE_str+",dfdqd_upd_curr_vec_in_AZ_dqdPE"+PE_str+",dfdqd_upd_curr_vec_in_LX_dqdPE"+PE_str+",dfdqd_upd_curr_vec_in_LY_dqdPE"+PE_str+",dfdqd_upd_curr_vec_in_LZ_dqdPE"+PE_str+","+"\n")

# Write Minv External Input Ports
def writeMinvExternalInputPorts(PEid,output_file,dim_list,block_size,indent):
   PE_str = str(PEid)
   output_file.write("   // dqdPE"+PE_str+""+"\n")
   #----------------------------------------------------------------------------
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   prefix = "minv_block_in_"
   suffix = "_dqdPE"+PE_str+""
   indent = "      "
   mat_str = makeNXNTranMatString(prefix,suffix,block_size,indent)
   output_file.write(mat_str+","+"\n")
   #----------------------------------------------------------------------------
   output_file.write("   input  signed[(WIDTH-1):0]"+"\n")
   output_file.write("      ")
   prefix = "dtau_vec_in_"
   suffix = "_dqdPE"+PE_str+""
   vec_str = makeNVecString(prefix,suffix,block_size,"R")
   output_file.write(vec_str+","+"\n")

#-------------------------------------------------------------------------------
bproc_file.write("`timescale 1ns / 1ps"+"\n")
bproc_file.write("\n")
bproc_file.write("// Backward Pass Row Unit with RNEA and Block Minv"+"\n")
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
bproc_file.write("   // get_data_minv"+"\n")
bproc_file.write("   input get_data_minv,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea"+"\n")
bproc_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
bproc_file.write("      link_in_rnea,"+"\n")
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_val_in_rnea,cosq_val_in_rnea,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "f_upd_curr_vec_in_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   input  signed[(WIDTH-1):0]"+"\n")
prefix = "f_prev_vec_in_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dq external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
indent = "   "
for PEid in range(1,num_PEs+1):
   writeDQExternalInputPorts(PEid,bproc_file,dim_list,indent)
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dqd external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
indent = "   "
for PEid in range(1,num_PEs+1):
   writeDQDExternalInputPorts(PEid,bproc_file,dim_list,indent)
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // minv external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
indent = "   "
for PEid in range(1,num_PEs+1):
   writeMinvExternalInputPorts(PEid,bproc_file,dim_list,block_size,indent)
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // output_ready"+"\n")
bproc_file.write("   output output_ready,"+"\n")
bproc_file.write("   // output_ready_minv"+"\n")
bproc_file.write("   output output_ready_minv,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea external outputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea"+"\n")
bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      tau_curr_out_rnea,"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
prefix = "f_upd_prev_vec_out_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dq external outputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("   // dqPE"+PE_str+""+"\n")
   bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
   bproc_file.write("      dtau_curr_out_dqPE"+PE_str+","+"\n")
   #----------------------------------------------------------------------------
   bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
   prefix = "dfdq_upd_prev_vec_out_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dqd external outputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
   bproc_file.write("      dtau_curr_out_dqdPE"+PE_str+","+"\n")
   #----------------------------------------------------------------------------
   bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
   prefix = "dfdqd_upd_prev_vec_out_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // minv external outputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
last_element_bool = False
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   #----------------------------------------------------------------------------
   bproc_file.write("   output signed[(WIDTH-1):0]"+"\n")
   prefix = "minv_vec_out_"
   vec_str = makeNVecString(prefix,suffix,block_size,"R")
   last_element_bool = (PEid == num_PEs)
   if (last_element_bool):
      bproc_file.write("      "+vec_str+"\n")
   else:
      bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   );"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // external wires and state"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   // registers"+"\n")
bproc_file.write("   reg get_data_reg;"+"\n")
bproc_file.write("   reg get_data_minv_reg;"+"\n")
bproc_file.write("   reg output_ready_reg;"+"\n")
bproc_file.write("   reg output_ready_minv_reg;"+"\n")
bproc_file.write("   reg minv_bool_reg;"+"\n")
bproc_file.write("   reg [2:0]"+"\n")
bproc_file.write("      state_reg;"+"\n")
bproc_file.write("   reg [2:0]"+"\n")
bproc_file.write("      state_minv_reg;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea"+"\n")
bproc_file.write("   reg ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
bproc_file.write("      link_reg_rnea;"+"\n")
bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_val_reg_rnea,cosq_val_reg_rnea,"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_upd_curr_vec_reg_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
prefix = "f_prev_vec_reg_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dq external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("   // dqPE"+PE_str+""+"\n")
   bproc_file.write("   reg ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      link_reg_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   reg ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      derv_reg_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
   bproc_file.write("      sinq_val_reg_dqPE"+PE_str+",cosq_val_reg_dqPE"+PE_str+","+"\n")
   prefix = "f_upd_curr_vec_reg_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
   prefix = "dfdq_prev_vec_reg_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
   prefix = "dfdq_upd_curr_vec_reg_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dqd external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   reg ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      link_reg_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   reg ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      derv_reg_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
   bproc_file.write("      sinq_val_reg_dqdPE"+PE_str+",cosq_val_reg_dqdPE"+PE_str+","+"\n")
   prefix = "dfdqd_prev_vec_reg_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
   prefix = "dfdqd_upd_curr_vec_reg_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // minv external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
   prefix = "minv_block_reg_"
   indent = "      "
   mat_str = makeNXNTranMatString(prefix,suffix,block_size,indent)
   bproc_file.write(mat_str+";"+"\n")
   bproc_file.write("   reg signed[(WIDTH-1):0]"+"\n")
   prefix = "dtau_vec_reg_"
   vec_str = makeNVecString(prefix,suffix,block_size,"R")
   bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // next"+"\n")
bproc_file.write("   wire get_data_next;"+"\n")
bproc_file.write("   wire get_data_minv_next;"+"\n")
bproc_file.write("   wire output_ready_next;"+"\n")
bproc_file.write("   wire output_ready_minv_next;"+"\n")
bproc_file.write("   wire minv_bool_next;"+"\n")
bproc_file.write("   wire [2:0]"+"\n")
bproc_file.write("      state_next;"+"\n")
bproc_file.write("   wire [2:0]"+"\n")
bproc_file.write("      state_minv_next;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea"+"\n")
bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
bproc_file.write("      link_next_rnea;"+"\n")
bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
bproc_file.write("      sinq_val_next_rnea,cosq_val_next_rnea,"+"\n")
#-------------------------------------------------------------------------------
prefix = "f_upd_curr_vec_next_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+","+"\n")
#-------------------------------------------------------------------------------
prefix = "f_prev_vec_next_"
suffix = "_rnea"
vec_str = makeD6VecString(prefix,suffix,dim_list)
bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dq external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("   // dqPE"+PE_str+""+"\n")
   bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      link_next_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      derv_next_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
   bproc_file.write("      sinq_val_next_dqPE"+PE_str+",cosq_val_next_dqPE"+PE_str+","+"\n")
   prefix = "f_upd_curr_vec_next_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
   prefix = "dfdq_prev_vec_next_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
   prefix = "dfdq_upd_curr_vec_next_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dqd external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      link_next_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   wire ["+str(vw.get_bitwidth_array_def(num_links+1))+":0]"+"\n")
   bproc_file.write("      derv_next_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
   bproc_file.write("      sinq_val_next_dqdPE"+PE_str+",cosq_val_next_dqdPE"+PE_str+","+"\n")
   prefix = "dfdqd_prev_vec_next_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+","+"\n")
   prefix = "dfdqd_upd_curr_vec_next_"
   vec_str = makeD6VecString(prefix,suffix,dim_list)
   bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // minv external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
   prefix = "minv_block_next_"
   indent = "      "
   mat_str = makeNXNTranMatString(prefix,suffix,block_size,indent)
   bproc_file.write(mat_str+";"+"\n")
   bproc_file.write("   wire signed[(WIDTH-1):0]"+"\n")
   prefix = "dtau_vec_next_"
   vec_str = makeNVecString(prefix,suffix,block_size,"R")
   bproc_file.write("      "+vec_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // external assignments"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // inputs"+"\n")
bproc_file.write("   assign get_data_next = get_data;"+"\n")
bproc_file.write("   assign get_data_minv_next = get_data_minv;"+"\n")
bproc_file.write("   // output"+"\n")
bproc_file.write("   assign output_ready = output_ready_reg;"+"\n")
bproc_file.write("   assign output_ready_minv = output_ready_minv_reg;"+"\n")
bproc_file.write("   assign output_ready_next = ((state_reg == 3'd0)&&(get_data == 1)) ? 1 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd0)&&(get_data == 0)) ? 0 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 1)) ? 1 :"+"\n")
bproc_file.write("                              ((state_reg == 3'd1)&&(get_data == 0)) ? 0 : output_ready_reg;"+"\n")
bproc_file.write("   assign output_ready_minv_next = ((state_minv_reg == 3'd0)&&(get_data_minv == 1)) ? 1 :"+"\n")
bproc_file.write("                                   ((state_minv_reg == 3'd0)&&(get_data_minv == 0)) ? 0 :"+"\n")
bproc_file.write("                                   ((state_minv_reg == 3'd1)&&(get_data_minv == 1)) ? 1 :"+"\n")
bproc_file.write("                                   ((state_minv_reg == 3'd1)&&(get_data_minv == 0)) ? 0 : output_ready_minv_reg;"+"\n")
bproc_file.write("   // minv"+"\n")
bproc_file.write("   assign minv_bool_next = ((state_minv_reg == 3'd0)&&(get_data_minv == 1)) ? 1 :"+"\n")
bproc_file.write("                           ((state_minv_reg == 3'd0)&&(get_data_minv == 0)) ? 0 :"+"\n")
bproc_file.write("                           ((state_minv_reg == 3'd1)&&(get_data_minv == 1)) ? 1 :"+"\n")
bproc_file.write("                           ((state_minv_reg == 3'd1)&&(get_data_minv == 0)) ? 0 : 0;"+"\n")
bproc_file.write("   // state"+"\n")
bproc_file.write("   assign state_next     = ((state_reg == 3'd0)&&(get_data == 1)&&(get_data_minv == 0)) ? 3'd1 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd0)&&(get_data == 1)&&(get_data_minv == 1)) ? 3'd0 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd0)&&(get_data == 0)&&(get_data_minv == 0)) ? 3'd0 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd0)&&(get_data == 0)&&(get_data_minv == 1)) ? 3'd0 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 1)&&(get_data_minv == 0)) ? 3'd1 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 1)&&(get_data_minv == 1)) ? 3'd0 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 0)&&(get_data_minv == 0)) ? 3'd0 :"+"\n")
bproc_file.write("                           ((state_reg == 3'd1)&&(get_data == 0)&&(get_data_minv == 1)) ? 3'd0 : state_reg;"+"\n")
bproc_file.write("   assign state_minv_next = ((state_minv_reg == 3'd0)&&(get_data_minv == 1)) ? 3'd1 :"+"\n")
bproc_file.write("                            ((state_minv_reg == 3'd0)&&(get_data_minv == 0)) ? 3'd0 :"+"\n")
bproc_file.write("                            ((state_minv_reg == 3'd1)&&(get_data_minv == 1)) ? 3'd1 :"+"\n")
bproc_file.write("                            ((state_minv_reg == 3'd1)&&(get_data_minv == 0)) ? 3'd0 : state_minv_reg;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // rnea"+"\n")
bproc_file.write("   assign link_next_rnea = (get_data == 1) ? link_in_rnea :"+"\n")
bproc_file.write("                           (get_data == 1) ? link_in_rnea : link_reg_rnea;"+"\n")
bproc_file.write("   assign sinq_val_next_rnea = (get_data == 1) ? sinq_val_in_rnea :"+"\n")
bproc_file.write("                               (get_data == 1) ? sinq_val_in_rnea : sinq_val_reg_rnea;"+"\n")
bproc_file.write("   assign cosq_val_next_rnea = (get_data == 1) ? cosq_val_in_rnea :"+"\n")
bproc_file.write("                               (get_data == 1) ? cosq_val_in_rnea : cosq_val_reg_rnea;"+"\n")
bproc_file.write("   // f updated curr"+"\n")
bproc_file.write("   assign f_upd_curr_vec_next_AX_rnea = (get_data == 1) ? f_upd_curr_vec_in_AX_rnea :"+"\n")
bproc_file.write("                                        (get_data == 1) ? f_upd_curr_vec_in_AX_rnea : f_upd_curr_vec_reg_AX_rnea;"+"\n")
bproc_file.write("   assign f_upd_curr_vec_next_AY_rnea = (get_data == 1) ? f_upd_curr_vec_in_AY_rnea :"+"\n")
bproc_file.write("                                        (get_data == 1) ? f_upd_curr_vec_in_AY_rnea : f_upd_curr_vec_reg_AY_rnea;"+"\n")
bproc_file.write("   assign f_upd_curr_vec_next_AZ_rnea = (get_data == 1) ? f_upd_curr_vec_in_AZ_rnea :"+"\n")
bproc_file.write("                                        (get_data == 1) ? f_upd_curr_vec_in_AZ_rnea : f_upd_curr_vec_reg_AZ_rnea;"+"\n")
bproc_file.write("   assign f_upd_curr_vec_next_LX_rnea = (get_data == 1) ? f_upd_curr_vec_in_LX_rnea :"+"\n")
bproc_file.write("                                        (get_data == 1) ? f_upd_curr_vec_in_LX_rnea : f_upd_curr_vec_reg_LX_rnea;"+"\n")
bproc_file.write("   assign f_upd_curr_vec_next_LY_rnea = (get_data == 1) ? f_upd_curr_vec_in_LY_rnea :"+"\n")
bproc_file.write("                                        (get_data == 1) ? f_upd_curr_vec_in_LY_rnea : f_upd_curr_vec_reg_LY_rnea;"+"\n")
bproc_file.write("   assign f_upd_curr_vec_next_LZ_rnea = (get_data == 1) ? f_upd_curr_vec_in_LZ_rnea :"+"\n")
bproc_file.write("                                        (get_data == 1) ? f_upd_curr_vec_in_LZ_rnea : f_upd_curr_vec_reg_LZ_rnea;"+"\n")
bproc_file.write("   // f prev"+"\n")
bproc_file.write("   assign f_prev_vec_next_AX_rnea = (get_data == 1) ? f_prev_vec_in_AX_rnea :"+"\n")
bproc_file.write("                                    (get_data == 1) ? f_prev_vec_in_AX_rnea : f_prev_vec_reg_AX_rnea;"+"\n")
bproc_file.write("   assign f_prev_vec_next_AY_rnea = (get_data == 1) ? f_prev_vec_in_AY_rnea :"+"\n")
bproc_file.write("                                    (get_data == 1) ? f_prev_vec_in_AY_rnea : f_prev_vec_reg_AY_rnea;"+"\n")
bproc_file.write("   assign f_prev_vec_next_AZ_rnea = (get_data == 1) ? f_prev_vec_in_AZ_rnea :"+"\n")
bproc_file.write("                                    (get_data == 1) ? f_prev_vec_in_AZ_rnea : f_prev_vec_reg_AZ_rnea;"+"\n")
bproc_file.write("   assign f_prev_vec_next_LX_rnea = (get_data == 1) ? f_prev_vec_in_LX_rnea :"+"\n")
bproc_file.write("                                    (get_data == 1) ? f_prev_vec_in_LX_rnea : f_prev_vec_reg_LX_rnea;"+"\n")
bproc_file.write("   assign f_prev_vec_next_LY_rnea = (get_data == 1) ? f_prev_vec_in_LY_rnea :"+"\n")
bproc_file.write("                                    (get_data == 1) ? f_prev_vec_in_LY_rnea : f_prev_vec_reg_LY_rnea;"+"\n")
bproc_file.write("   assign f_prev_vec_next_LZ_rnea = (get_data == 1) ? f_prev_vec_in_LZ_rnea :"+"\n")
bproc_file.write("                                    (get_data == 1) ? f_prev_vec_in_LZ_rnea : f_prev_vec_reg_LZ_rnea;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dq external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("   // dqPE"+PE_str+""+"\n")
   bproc_file.write("   assign link_next_dqPE"+PE_str+" = (get_data == 1) ? link_in_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                            (get_data == 1) ? link_in_dqPE"+PE_str+" : link_reg_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign derv_next_dqPE"+PE_str+" = (get_data == 1) ? derv_in_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                            (get_data == 1) ? derv_in_dqPE"+PE_str+" : derv_reg_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign sinq_val_next_dqPE"+PE_str+" = (get_data == 1) ? sinq_val_in_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                (get_data == 1) ? sinq_val_in_dqPE"+PE_str+" : sinq_val_reg_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign cosq_val_next_dqPE"+PE_str+" = (get_data == 1) ? cosq_val_in_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                (get_data == 1) ? cosq_val_in_dqPE"+PE_str+" : cosq_val_reg_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   // f updated curr"+"\n")
   bproc_file.write("   assign f_upd_curr_vec_next_AX_dqPE"+PE_str+" = (get_data == 1) ? f_upd_curr_vec_in_AX_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                         (get_data == 1) ? f_upd_curr_vec_in_AX_dqPE"+PE_str+" : f_upd_curr_vec_reg_AX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign f_upd_curr_vec_next_AY_dqPE"+PE_str+" = (get_data == 1) ? f_upd_curr_vec_in_AY_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                         (get_data == 1) ? f_upd_curr_vec_in_AY_dqPE"+PE_str+" : f_upd_curr_vec_reg_AY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign f_upd_curr_vec_next_AZ_dqPE"+PE_str+" = (get_data == 1) ? f_upd_curr_vec_in_AZ_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                         (get_data == 1) ? f_upd_curr_vec_in_AZ_dqPE"+PE_str+" : f_upd_curr_vec_reg_AZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign f_upd_curr_vec_next_LX_dqPE"+PE_str+" = (get_data == 1) ? f_upd_curr_vec_in_LX_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                         (get_data == 1) ? f_upd_curr_vec_in_LX_dqPE"+PE_str+" : f_upd_curr_vec_reg_LX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign f_upd_curr_vec_next_LY_dqPE"+PE_str+" = (get_data == 1) ? f_upd_curr_vec_in_LY_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                         (get_data == 1) ? f_upd_curr_vec_in_LY_dqPE"+PE_str+" : f_upd_curr_vec_reg_LY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign f_upd_curr_vec_next_LZ_dqPE"+PE_str+" = (get_data == 1) ? f_upd_curr_vec_in_LZ_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                         (get_data == 1) ? f_upd_curr_vec_in_LZ_dqPE"+PE_str+" : f_upd_curr_vec_reg_LZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   // df prev"+"\n")
   bproc_file.write("   assign dfdq_prev_vec_next_AX_dqPE"+PE_str+" = (get_data == 1) ? dfdq_prev_vec_in_AX_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                        (get_data == 1) ? dfdq_prev_vec_in_AX_dqPE"+PE_str+" : dfdq_prev_vec_reg_AX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_prev_vec_next_AY_dqPE"+PE_str+" = (get_data == 1) ? dfdq_prev_vec_in_AY_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                        (get_data == 1) ? dfdq_prev_vec_in_AY_dqPE"+PE_str+" : dfdq_prev_vec_reg_AY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_prev_vec_next_AZ_dqPE"+PE_str+" = (get_data == 1) ? dfdq_prev_vec_in_AZ_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                        (get_data == 1) ? dfdq_prev_vec_in_AZ_dqPE"+PE_str+" : dfdq_prev_vec_reg_AZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_prev_vec_next_LX_dqPE"+PE_str+" = (get_data == 1) ? dfdq_prev_vec_in_LX_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                        (get_data == 1) ? dfdq_prev_vec_in_LX_dqPE"+PE_str+" : dfdq_prev_vec_reg_LX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_prev_vec_next_LY_dqPE"+PE_str+" = (get_data == 1) ? dfdq_prev_vec_in_LY_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                        (get_data == 1) ? dfdq_prev_vec_in_LY_dqPE"+PE_str+" : dfdq_prev_vec_reg_LY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_prev_vec_next_LZ_dqPE"+PE_str+" = (get_data == 1) ? dfdq_prev_vec_in_LZ_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                        (get_data == 1) ? dfdq_prev_vec_in_LZ_dqPE"+PE_str+" : dfdq_prev_vec_reg_LZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   // f updated curr"+"\n")
   bproc_file.write("   assign dfdq_upd_curr_vec_next_AX_dqPE"+PE_str+" = (get_data == 1) ? dfdq_upd_curr_vec_in_AX_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                            (get_data == 1) ? dfdq_upd_curr_vec_in_AX_dqPE"+PE_str+" : dfdq_upd_curr_vec_reg_AX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_upd_curr_vec_next_AY_dqPE"+PE_str+" = (get_data == 1) ? dfdq_upd_curr_vec_in_AY_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                            (get_data == 1) ? dfdq_upd_curr_vec_in_AY_dqPE"+PE_str+" : dfdq_upd_curr_vec_reg_AY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_upd_curr_vec_next_AZ_dqPE"+PE_str+" = (get_data == 1) ? dfdq_upd_curr_vec_in_AZ_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                            (get_data == 1) ? dfdq_upd_curr_vec_in_AZ_dqPE"+PE_str+" : dfdq_upd_curr_vec_reg_AZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_upd_curr_vec_next_LX_dqPE"+PE_str+" = (get_data == 1) ? dfdq_upd_curr_vec_in_LX_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                            (get_data == 1) ? dfdq_upd_curr_vec_in_LX_dqPE"+PE_str+" : dfdq_upd_curr_vec_reg_LX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_upd_curr_vec_next_LY_dqPE"+PE_str+" = (get_data == 1) ? dfdq_upd_curr_vec_in_LY_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                            (get_data == 1) ? dfdq_upd_curr_vec_in_LY_dqPE"+PE_str+" : dfdq_upd_curr_vec_reg_LY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdq_upd_curr_vec_next_LZ_dqPE"+PE_str+" = (get_data == 1) ? dfdq_upd_curr_vec_in_LZ_dqPE"+PE_str+" :"+"\n")
   bproc_file.write("                                            (get_data == 1) ? dfdq_upd_curr_vec_in_LZ_dqPE"+PE_str+" : dfdq_upd_curr_vec_reg_LZ_dqPE"+PE_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dqd external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   assign link_next_dqdPE"+PE_str+" = (get_data == 1) ? link_in_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                             (get_data == 1) ? link_in_dqdPE"+PE_str+" : link_reg_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign derv_next_dqdPE"+PE_str+" = (get_data == 1) ? derv_in_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                             (get_data == 1) ? derv_in_dqdPE"+PE_str+" : derv_reg_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign sinq_val_next_dqdPE"+PE_str+" = (get_data == 1) ? sinq_val_in_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                 (get_data == 1) ? sinq_val_in_dqdPE"+PE_str+" : sinq_val_reg_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign cosq_val_next_dqdPE"+PE_str+" = (get_data == 1) ? cosq_val_in_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                 (get_data == 1) ? cosq_val_in_dqdPE"+PE_str+" : cosq_val_reg_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   // df prev"+"\n")
   bproc_file.write("   assign dfdqd_prev_vec_next_AX_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_prev_vec_in_AX_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                          (get_data == 1) ? dfdqd_prev_vec_in_AX_dqdPE"+PE_str+" : dfdqd_prev_vec_reg_AX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_prev_vec_next_AY_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_prev_vec_in_AY_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                          (get_data == 1) ? dfdqd_prev_vec_in_AY_dqdPE"+PE_str+" : dfdqd_prev_vec_reg_AY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_prev_vec_next_AZ_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_prev_vec_in_AZ_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                          (get_data == 1) ? dfdqd_prev_vec_in_AZ_dqdPE"+PE_str+" : dfdqd_prev_vec_reg_AZ_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_prev_vec_next_LX_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_prev_vec_in_LX_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                          (get_data == 1) ? dfdqd_prev_vec_in_LX_dqdPE"+PE_str+" : dfdqd_prev_vec_reg_LX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_prev_vec_next_LY_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_prev_vec_in_LY_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                          (get_data == 1) ? dfdqd_prev_vec_in_LY_dqdPE"+PE_str+" : dfdqd_prev_vec_reg_LY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_prev_vec_next_LZ_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_prev_vec_in_LZ_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                          (get_data == 1) ? dfdqd_prev_vec_in_LZ_dqdPE"+PE_str+" : dfdqd_prev_vec_reg_LZ_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   // f updated curr"+"\n")
   bproc_file.write("   assign dfdqd_upd_curr_vec_next_AX_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_upd_curr_vec_in_AX_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                              (get_data == 1) ? dfdqd_upd_curr_vec_in_AX_dqdPE"+PE_str+" : dfdqd_upd_curr_vec_reg_AX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_upd_curr_vec_next_AY_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_upd_curr_vec_in_AY_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                              (get_data == 1) ? dfdqd_upd_curr_vec_in_AY_dqdPE"+PE_str+" : dfdqd_upd_curr_vec_reg_AY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_upd_curr_vec_next_AZ_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_upd_curr_vec_in_AZ_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                              (get_data == 1) ? dfdqd_upd_curr_vec_in_AZ_dqdPE"+PE_str+" : dfdqd_upd_curr_vec_reg_AZ_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_upd_curr_vec_next_LX_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_upd_curr_vec_in_LX_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                              (get_data == 1) ? dfdqd_upd_curr_vec_in_LX_dqdPE"+PE_str+" : dfdqd_upd_curr_vec_reg_LX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_upd_curr_vec_next_LY_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_upd_curr_vec_in_LY_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                              (get_data == 1) ? dfdqd_upd_curr_vec_in_LY_dqdPE"+PE_str+" : dfdqd_upd_curr_vec_reg_LY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign dfdqd_upd_curr_vec_next_LZ_dqdPE"+PE_str+" = (get_data == 1) ? dfdqd_upd_curr_vec_in_LZ_dqdPE"+PE_str+" :"+"\n")
   bproc_file.write("                                              (get_data == 1) ? dfdqd_upd_curr_vec_in_LZ_dqdPE"+PE_str+" : dfdqd_upd_curr_vec_reg_LZ_dqdPE"+PE_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // minv external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   #----------------------------------------------------------------------------
   prefix = "minv_block_next_"
   cond_str = "(get_data_minv == 1)"
   prefix_true = "minv_block_in_"
   prefix_false = "minv_block_reg_"
   indent = "   "
   mat_str = makeNXNMat1CondAssignString(prefix,suffix,block_size,cond_str,prefix_true,prefix_false,"C","C","C",True)
   bproc_file.write(mat_str+"\n")
   #----------------------------------------------------------------------------
   prefix = "dtau_vec_next_"
   cond_str = "(get_data_minv == 1)"
   prefix_true = "dtau_vec_in_"
   prefix_false = "dtau_vec_reg_"
   indent = "   "
   mat_str = makeNVec1CondAssignString(prefix,suffix,block_size,cond_str,prefix_true,prefix_false,False)
   bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
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
bproc_file.write("         get_data_minv_reg <= 0;"+"\n")
bproc_file.write("         // output"+"\n")
bproc_file.write("         output_ready_reg <= 0;"+"\n")
bproc_file.write("         output_ready_minv_reg <= 0;"+"\n")
bproc_file.write("         // minv"+"\n")
bproc_file.write("         minv_bool_reg <= 0;"+"\n")
bproc_file.write("         // state"+"\n")
bproc_file.write("         state_reg      <= 3'd0;"+"\n")
bproc_file.write("         state_minv_reg <= 3'd0;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // rnea external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // rnea"+"\n")
bproc_file.write("         link_reg_rnea <= 4'd9;"+"\n")
bproc_file.write("         sinq_val_reg_rnea <= 32'd0;"+"\n")
bproc_file.write("         cosq_val_reg_rnea <= 32'd0;"+"\n")
bproc_file.write("         // f updated curr"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_AX_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_AY_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_AZ_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_LX_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_LY_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_LZ_rnea <= 32'd0;"+"\n")
bproc_file.write("         // f prev"+"\n")
bproc_file.write("         f_prev_vec_reg_AX_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_prev_vec_reg_AY_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_prev_vec_reg_AZ_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_prev_vec_reg_LX_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_prev_vec_reg_LY_rnea <= 32'd0;"+"\n")
bproc_file.write("         f_prev_vec_reg_LZ_rnea <= 32'd0;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // dq external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("         // dqPE"+PE_str+""+"\n")
   bproc_file.write("         link_reg_dqPE"+PE_str+" <= 4'd9;"+"\n")
   bproc_file.write("         derv_reg_dqPE"+PE_str+" <= 4'd0;"+"\n")
   bproc_file.write("         sinq_val_reg_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         cosq_val_reg_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         // f updated curr"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_AX_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_AY_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_AZ_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_LX_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_LY_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_LZ_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         // df prev"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_AX_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_AY_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_AZ_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_LX_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_LY_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_LZ_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         // f updated curr"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_AX_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_AY_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_AZ_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_LX_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_LY_dqPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_LZ_dqPE"+PE_str+" <= 32'd0;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // dqd external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("         // dqdPE"+PE_str+""+"\n")
   bproc_file.write("         link_reg_dqdPE"+PE_str+" <= 4'd9;"+"\n")
   bproc_file.write("         derv_reg_dqdPE"+PE_str+" <= 4'd0;"+"\n")
   bproc_file.write("         sinq_val_reg_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         cosq_val_reg_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         // df prev"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_AX_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_AY_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_AZ_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_LX_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_LY_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_LZ_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         // f updated curr"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_AX_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_AY_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_AZ_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_LX_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_LY_dqdPE"+PE_str+" <= 32'd0;"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_LZ_dqdPE"+PE_str+" <= 32'd0;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // minv external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("         // dqdPE"+PE_str+""+"\n")
   #-------------------------------------------------------------------------------
   prefix = "minv_block_reg_"
   zero_bool = True
   prefix_in = ""
   indent = "         "
   mat_str = makeNXNMatNonblockingAssignmentString(prefix,suffix,block_size,zero_bool,prefix_in)
   bproc_file.write(mat_str+"\n")
   #-------------------------------------------------------------------------------
   prefix = "dtau_vec_reg_"
   zero_bool = True
   prefix_in = ""
   indent = "         "
   mat_str = makeNVecNonblockingAssignmentString(prefix,suffix,block_size,dim_list,zero_bool,prefix_in)
   bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("      end"+"\n")
bproc_file.write("      else"+"\n")
bproc_file.write("      begin"+"\n")
bproc_file.write("         // inputs"+"\n")
bproc_file.write("         get_data_reg <= get_data_next;"+"\n")
bproc_file.write("         get_data_minv_reg <= get_data_minv_next;"+"\n")
bproc_file.write("         // output"+"\n")
bproc_file.write("         output_ready_reg <= output_ready_next;"+"\n")
bproc_file.write("         output_ready_minv_reg <= output_ready_minv_next;"+"\n")
bproc_file.write("         // minv"+"\n")
bproc_file.write("         minv_bool_reg <= minv_bool_next;"+"\n")
bproc_file.write("         // state"+"\n")
bproc_file.write("         state_reg      <= state_next;"+"\n")
bproc_file.write("         state_minv_reg <= state_minv_next;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // rnea external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // rnea"+"\n")
bproc_file.write("         link_reg_rnea <= link_next_rnea;"+"\n")
bproc_file.write("         sinq_val_reg_rnea <= sinq_val_next_rnea;"+"\n")
bproc_file.write("         cosq_val_reg_rnea <= cosq_val_next_rnea;"+"\n")
bproc_file.write("         // f updated curr"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_AX_rnea <= f_upd_curr_vec_next_AX_rnea;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_AY_rnea <= f_upd_curr_vec_next_AY_rnea;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_AZ_rnea <= f_upd_curr_vec_next_AZ_rnea;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_LX_rnea <= f_upd_curr_vec_next_LX_rnea;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_LY_rnea <= f_upd_curr_vec_next_LY_rnea;"+"\n")
bproc_file.write("         f_upd_curr_vec_reg_LZ_rnea <= f_upd_curr_vec_next_LZ_rnea;"+"\n")
bproc_file.write("         // f prev"+"\n")
bproc_file.write("         f_prev_vec_reg_AX_rnea <= f_prev_vec_next_AX_rnea;"+"\n")
bproc_file.write("         f_prev_vec_reg_AY_rnea <= f_prev_vec_next_AY_rnea;"+"\n")
bproc_file.write("         f_prev_vec_reg_AZ_rnea <= f_prev_vec_next_AZ_rnea;"+"\n")
bproc_file.write("         f_prev_vec_reg_LX_rnea <= f_prev_vec_next_LX_rnea;"+"\n")
bproc_file.write("         f_prev_vec_reg_LY_rnea <= f_prev_vec_next_LY_rnea;"+"\n")
bproc_file.write("         f_prev_vec_reg_LZ_rnea <= f_prev_vec_next_LZ_rnea;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // dq external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("         // dqPE"+PE_str+""+"\n")
   bproc_file.write("         link_reg_dqPE"+PE_str+" <= link_next_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         derv_reg_dqPE"+PE_str+" <= derv_next_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         sinq_val_reg_dqPE"+PE_str+" <= sinq_val_next_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         cosq_val_reg_dqPE"+PE_str+" <= cosq_val_next_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         // f updated curr"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_AX_dqPE"+PE_str+" <= f_upd_curr_vec_next_AX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_AY_dqPE"+PE_str+" <= f_upd_curr_vec_next_AY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_AZ_dqPE"+PE_str+" <= f_upd_curr_vec_next_AZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_LX_dqPE"+PE_str+" <= f_upd_curr_vec_next_LX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_LY_dqPE"+PE_str+" <= f_upd_curr_vec_next_LY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         f_upd_curr_vec_reg_LZ_dqPE"+PE_str+" <= f_upd_curr_vec_next_LZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         // df prev"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_AX_dqPE"+PE_str+" <= dfdq_prev_vec_next_AX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_AY_dqPE"+PE_str+" <= dfdq_prev_vec_next_AY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_AZ_dqPE"+PE_str+" <= dfdq_prev_vec_next_AZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_LX_dqPE"+PE_str+" <= dfdq_prev_vec_next_LX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_LY_dqPE"+PE_str+" <= dfdq_prev_vec_next_LY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_prev_vec_reg_LZ_dqPE"+PE_str+" <= dfdq_prev_vec_next_LZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         // f updated curr"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_AX_dqPE"+PE_str+" <= dfdq_upd_curr_vec_next_AX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_AY_dqPE"+PE_str+" <= dfdq_upd_curr_vec_next_AY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_AZ_dqPE"+PE_str+" <= dfdq_upd_curr_vec_next_AZ_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_LX_dqPE"+PE_str+" <= dfdq_upd_curr_vec_next_LX_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_LY_dqPE"+PE_str+" <= dfdq_upd_curr_vec_next_LY_dqPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdq_upd_curr_vec_reg_LZ_dqPE"+PE_str+" <= dfdq_upd_curr_vec_next_LZ_dqPE"+PE_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // dqd external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("         // dqdPE"+PE_str+""+"\n")
   bproc_file.write("         link_reg_dqdPE"+PE_str+" <= link_next_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         derv_reg_dqdPE"+PE_str+" <= derv_next_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         sinq_val_reg_dqdPE"+PE_str+" <= sinq_val_next_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         cosq_val_reg_dqdPE"+PE_str+" <= cosq_val_next_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         // df prev"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_AX_dqdPE"+PE_str+" <= dfdqd_prev_vec_next_AX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_AY_dqdPE"+PE_str+" <= dfdqd_prev_vec_next_AY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_AZ_dqdPE"+PE_str+" <= dfdqd_prev_vec_next_AZ_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_LX_dqdPE"+PE_str+" <= dfdqd_prev_vec_next_LX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_LY_dqdPE"+PE_str+" <= dfdqd_prev_vec_next_LY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_prev_vec_reg_LZ_dqdPE"+PE_str+" <= dfdqd_prev_vec_next_LZ_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         // f updated curr"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_AX_dqdPE"+PE_str+" <= dfdqd_upd_curr_vec_next_AX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_AY_dqdPE"+PE_str+" <= dfdqd_upd_curr_vec_next_AY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_AZ_dqdPE"+PE_str+" <= dfdqd_upd_curr_vec_next_AZ_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_LX_dqdPE"+PE_str+" <= dfdqd_upd_curr_vec_next_LX_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_LY_dqdPE"+PE_str+" <= dfdqd_upd_curr_vec_next_LY_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("         dfdqd_upd_curr_vec_reg_LZ_dqdPE"+PE_str+" <= dfdqd_upd_curr_vec_next_LZ_dqdPE"+PE_str+";"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("         // minv external inputs"+"\n")
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("         // dqdPE"+PE_str+""+"\n")
   #-------------------------------------------------------------------------------
   prefix = "minv_block_reg_"
   zero_bool = False
   prefix_in = "minv_block_next_"
   indent = "         "
   mat_str = makeNXNMatNonblockingAssignmentString(prefix,suffix,block_size,zero_bool,prefix_in)
   bproc_file.write(mat_str+"\n")
   #-------------------------------------------------------------------------------
   prefix = "dtau_vec_reg_"
   zero_bool = False
   prefix_in = "dtau_vec_next_"
   indent = "         "
   mat_str = makeNVecNonblockingAssignmentString(prefix,suffix,block_size,dim_list,zero_bool,prefix_in)
   bproc_file.write(mat_str+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("         //---------------------------------------------------------------------"+"\n")
bproc_file.write("      end"+"\n")
bproc_file.write("   end"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // ID"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
indent = "   "
suffix = "_rnea"
mat_str = makeIDPE(suffix,dim_list)
bproc_file.write(mat_str+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dq external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   bproc_file.write("   // dqPE"+PE_str+""+"\n")
   bproc_file.write("   wire fx_dqPE"+PE_str+";"+"\n")
   bproc_file.write("   assign fx_dqPE"+PE_str+" = (link_reg_dqPE"+PE_str+" == derv_reg_dqPE"+PE_str+") ? 1 : 0;"+"\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dqd external inputs"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   bproc_file.write("   // dqdPE"+PE_str+""+"\n")
   bproc_file.write("   wire fx_dqdPE"+PE_str+";"+"\n")
   bproc_file.write("   assign fx_dqdPE"+PE_str+" = (link_reg_dqdPE"+PE_str+" == derv_reg_dqdPE"+PE_str+") ? 1 : 0;"+"\n")
bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dID/dq"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("\n")
indent = "   "
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqPE"+PE_str+""
   mat_str = makeGradIDdqPE(suffix,PEid,dim_list)
   bproc_file.write(mat_str+"\n")
   bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("   // dID/dqd"+"\n")
bproc_file.write("   //---------------------------------------------------------------------------"+"\n")
bproc_file.write("\n")
indent = "   "
for PEid in range(1,num_PEs+1):
   PE_str = str(PEid)
   suffix = "_dqdPE"+PE_str+""
   mat_str = makeGradIDdqdPE(suffix,PEid,num_links,block_size,dim_list)
   bproc_file.write(mat_str+"\n")
   bproc_file.write("\n")
#-------------------------------------------------------------------------------
bproc_file.write("endmodule"+"\n")
#-------------------------------------------------------------------------------

# Close file
bproc_file.close()
