from URDFParser import URDFParser
from util import BluespecWriter, FileManager
from rbd_config import dim_list, urdf_file, num_PEs, block_size
from DataStructSizes import *
from math import log2, floor, ceil

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()

#-------- File management -------------------

fm = FileManager("GradientPipeline.bsv")
output_file_path = fm.get_output_file_path()
gp_file = open(output_file_path, "w")

bw = BluespecWriter(gp_file, dim_list, num_links, block_size)

#---------------------------------------------

num_links_decr = num_links-1
num_links_incr = num_links+1
# num_links+2 is for the "world" link, used in backward pass to generate updates
# for leaf links
max_link_ref = num_links+1
# we have an incremented version of max_link_ref for siziing 1-indexed registers
max_link_incr = num_links+2

padded_matrix_dim = int(ceil(num_links / block_size)) * block_size

#---------------------------------------------

bw.writeLine("import GetPut::*;")
bw.writeLine("import FIFO::*;")
bw.writeLine("import ClientServer::*;")
bw.writeLine("import ConnectalMemory::*;")
bw.writeLine("import ConnectalConfig::*;")
bw.writeLine("import ConnectalMemTypes::*;")
bw.writeLine("import MemReadEngine::*;")
bw.writeLine("import MemWriteEngine::*;")
bw.writeLine("import Vector::*;")
bw.writeLine("import BuildVector::*;")
bw.writeLine("import IfcGradientPipeline::*;")
bw.writeLine("import GradientPipelineTypes::*;")
bw.writeLine("import BRAM::*;")
bw.writeLine("import BRAMFIFO::*;")
bw.writeLine("import BviIfcFProc_rev1::*;")
bw.writeLine("import BviIfcBProc_rev2::*;")
bw.writeLine("import BuildVector::*;")
bw.writeLine("import Pipe::*;")
bw.writeLine("import Ehr::*;")
bw.writeLine("import ConvertersFixFp::*;")
bw.writeLine("")
bw.writeLine("interface GradientPipeline;")
bw.writeLine("    interface Gradient start;")
bw.writeLine("    interface Vector#(1, MemReadClient#(256)) dmaReadClient;")
bw.writeLine("    interface Vector#(1, MemWriteClient#(256)) dmaWriteClient;")
bw.writeLine("endinterface")
bw.writeLine("")
bw.writeLine("typedef enum {GradientStop, GradientRunning} State deriving(Eq, Bits,FShow);")
bw.writeLine("")

bw.writeLine("module mkGradientPipeline#(GradientPipelineIndication indication)(GradientPipeline);")
bw.writeLine("    // rnea_acc[0] left empty for ease of indexing")
bw.writeLine("    // s/9/num_links+2 because num_links+1 is an input to bproc and is 1-indexed")

bw.writeLineWithFormatSpecReplace("    Vector#({max_link_incr},Reg#(RNEAIntermediate)) rnea_acc <- replicateM(mkReg(unpack(0)));")
bw.writeLineWithFormatSpecReplace("    Vector#({num_links_incr},Reg#(Trigo)) trigo_acc <- replicateM(mkReg(unpack(0))); // 1-indexed")
bw.writeLine("    // s/9/num_links+2 because num_links+1 is an input to bproc and is 1-indexed")
bw.writeLineWithFormatSpecReplace("    FIFO#(Vector#({max_link_incr},Trigo)) trigo_values <- mkSizedFIFO(4);")

bw.writeLine("")
bw.writeLine("    // storing intermediate dfi{dq,dqd}s outputed by fproc as matrix of regs to be able to arbitrarily index into them")
bw.writeLine("    // previously used to be: Vector#({num_links_incr},Reg#(Intermediate2)) intermediate_acc <- replicateM(mkReg(unpack(0)));    // 1-indexed")

bw.writeLine("    // 8-array of 6-vectors")
bw.writeLineWithFormatSpecReplace("    Reg#(Vector#({num_links_incr}, Vector#(6, Bit#(32)))) f_inter_acc <- mkReg(unpack(0));")
bw.writeLineWithFormatSpecReplace("    // 8-array of (6x7)-matrices")
bw.writeLineWithFormatSpecReplace("    Reg#(Vector#({num_links_incr}, Vector#(6, Vector#({num_links}, Bit#(32))))) dfidq_inter_acc <- mkReg(unpack(0));")
bw.writeLineWithFormatSpecReplace("    Reg#(Vector#({num_links_incr}, Vector#(6, Vector#({num_links}, Bit#(32))))) dfidqd_inter_acc <- mkReg(unpack(0));")
bw.writeLine("")
bw.writeLineWithFormatSpecReplace("    // s/{max_link_incr}/num_links+2 because num_links+1 is an input to bproc and is 1-indexed")
bw.writeLineWithFormatSpecReplace("    FIFO#(Vector#({max_link_incr},Intermediate2)) intermediate_values <- mkSizedFIFO(3);")

bw.writeLine("")
bw.writeLine("    // explicitly storing minv")
bw.writeLineWithFormatSpecReplace("    Vector#({padded_matrix_dim}, Vector#({padded_matrix_dim}, Reg#(Bit#(32)))) minv <- replicateM(replicateM(mkReg(0)));")
bw.writeLine("")

###
# FPGAKnotIn with zero padding included because this is where we get it from DMA
FPGAKnotIn_size_bits = to_bits(get_FPGAKnotIn_size_bytes(num_links))
bw.writeLine("    Reg#(Bit#({bits})) int_inputs <- mkReg(0);".format(bits=FPGAKnotIn_size_bits))
###

bw.writeLine("    Reg#(Bit#(16)) inputs_counter <- mkReg(0);")
bw.writeLine("    Reg#(Bit#(32)) nbP <- mkReg(64);")
bw.writeLineWithFormatSpecReplace("    FIFO#(Vector#({num_links_incr},FPGALink3)) inputs <- mkSizedFIFO(4); // 1-indexed")
bw.writeLine("")
bw.writeLine("    //commented to allow host-side fpconv")
bw.writeLine("`ifndef HOST_FPCONV")
bw.writeLine("    Server#(Vector#(8,Bit#(32)),Vector#(8,Bit#(32))) fix_fps <- mkFixFps();")
bw.writeLine("    Server#(Vector#(8,Bit#(32)),Vector#(8,Bit#(32))) fp_fixs <- mkFpFixs();")
bw.writeLine("`endif")
bw.writeLine("")
bw.writeLine("    MemReadEngine#(256, 256, Depth, Depth) re <- mkMemReadEngineBuff(valueOf(Depth) * 512);")
bw.writeLine("    MemWriteEngine#(256, 256, Depth, Depth)  we <- mkMemWriteEngineBuff(valueOf(Depth) * 512);")
bw.writeLine("")
bw.writeLine("    Reg#(SGLId) rdPointer <- mkReg(0);")
bw.writeLine("    Reg#(SGLId) wrPointer <- mkReg(0);")
bw.writeLine("    Reg#(State) state <- mkReg(GradientStop);")
bw.writeLine("")
bw.writeLine("    Reg#(Maybe#(Bit#(32))) read_idx_forward <- mkReg(Invalid);")
bw.writeLine("")
bw.writeLine("    Reg#(Bit#(32)) cyc_count <- mkReg(0);")
bw.writeLine("")
bw.writeLine("    FProc fproc <- mkFProc();")
bw.writeLine("    BProc bproc <- mkBProc();")
bw.writeLine("")
bw.writeLine("    rule tic;")
bw.writeLine("        cyc_count <= cyc_count + 1;")
bw.writeLine("    endrule")
bw.writeLine("")



###
bw.writeLine("    rule feedInput if (read_idx_forward matches tagged Valid .s);")
bw.writeLine("        if (read_idx_forward matches tagged Valid .offset) begin")
bw.writeLine("            re.readServers[0].request.put(MemengineCmd{")
bw.writeLine("            sglId:rdPointer,")
bw.writeLine("            base: 0,")

FPGAKnotIn_size_bytes = get_FPGAKnotIn_size_bytes(num_links)
bw.writeLine("            len: {bytes}*nbP,".format(bytes=FPGAKnotIn_size_bytes))

bw.writeLine("            burstLen: 128,")
bw.writeLine("            tag:0});")
bw.writeLine("            $display(\"Request data for the 64 points\", offset);")
bw.writeLine("        read_idx_forward <= tagged Invalid;")
bw.writeLine("        end")
bw.writeLine("    endrule")
bw.writeLine("")
###

bw.writeLine("    //commented to allow host-side fpconv")
bw.writeLine("")
bw.writeLine("`ifndef HOST_FPCONV")
bw.writeLine("    rule converFpToFix;")
bw.writeLine("           let complete_input_bits <- toGet(re.readServers[0].data).get();")
bw.writeLine("           //$display(\"Start fp to fix converison of \", fshow(complete_input_bits.data));")
bw.writeLine("           fp_fixs.request.put(unpack(complete_input_bits.data));")
bw.writeLine("    endrule")
bw.writeLine("`endif")
bw.writeLine("")
bw.writeLine("    Reg#(Bit#(32)) counter_input <- mkReg(0);")
bw.writeLine("")
bw.writeLine("    Reg#(FPGALink3) zeroLink <- mkReg(unpack(0));")
bw.writeLine("")
bw.writeLine("    rule getInput;")
bw.writeLine("`ifndef HOST_FPCONV")
bw.writeLine("        let complete_input_bitspacked <- fp_fixs.response.get();")
bw.writeLine("        let complete_input_bits = pack(complete_input_bitspacked);")
bw.writeLine("        //$display(\"Receive chunk converted to fix:\", fshow(complete_input_bits));")
bw.writeLine("`else")
bw.writeLine("        let complete_input_bitspacked <- toGet(re.readServers[0].data).get();")
bw.writeLine("        let complete_input_bits = complete_input_bitspacked.data;")
bw.writeLine("`endif")
bw.writeLine("")

###
# FPGAKnotIn zero extend
FPGAKnotIn_size_bits_zext_val = FPGAKnotIn_size_bits - 256
fmap = {
        "knot_in_bits": FPGAKnotIn_size_bits,
        "zext_bits"   : FPGAKnotIn_size_bits_zext_val,
}
bw.writeLine("        Bit#({knot_in_bits}) new_int = (int_inputs >> 256) | (zeroExtend(complete_input_bits) << {zext_bits}); ".format_map(fmap))

bw.writeLine("        //display(\"Current input:\", fshow(new_int));")
bw.writeLine("        let new_count = inputs_counter + 256;")
bw.writeLine("")
# FPGAKnotIn zero extend
bw.writeLine("        if (inputs_counter + 256 > {zext_bits}) begin".format(zext_bits=FPGAKnotIn_size_bits_zext_val))
###

bw.writeLine("            //$display(\"Complete input received from host\", fshow(counter_input));")
bw.writeLine("            counter_input <= counter_input + 1;")

bw.writeLineWithFormatSpecReplace("            Vector#({num_links},FPGALink3) new_data = unpack(truncate(new_int));")
bw.writeLineWithFormatSpecReplace("            Vector#({num_links_incr},FPGALink3) new_data_1_indexed = cons(zeroLink, new_data);")

bw.writeLine("            inputs.enq(new_data_1_indexed);")
bw.writeLine("            new_count = 0; ")
bw.writeLine("        end ")
bw.writeLine("")
bw.writeLine("        int_inputs <= new_int; ")
bw.writeLine("        inputs_counter <= new_count;")
bw.writeLine("        $display(\"Chunk\", fshow(new_count));")
bw.writeLine("    endrule")
bw.writeLine("")
bw.writeLine("    //*********************")
bw.writeLine("")
bw.writeLine("    Ehr#(2,Bool) phase <- mkEhr(False);")
bw.writeLine("")

###
# we DON'T include zero padding in FPGAKnotOut here because this has
# nothing to do with the DMA
bits = to_bits(get_FPGAKnotOut_size_bytes(num_links) - FPGAKnotOut_zero_padding(num_links))
bw.writeLine("    FIFO#(Bit#({bits})) out_data <- mkSizedFIFO(10);".format(bits=bits))
bw.writeLine("    Reg#(Bit#(16)) gear_out <- mkReg(0);")
bw.writeLine("")
###

bw.writeLine("")
bw.writeLine("`include \"GradientPipelineFprocRules.bsv\"")
bw.writeLine("`include \"GradientPipelineBprocRules.bsv\"")
bw.writeLine("")

bw.writeLine("    Reg#(Bit#(16)) outstanding_dma <- mkReg(0);")
bw.writeLine("")
bw.writeLine("    rule send_writeDMA;")

###
# FPGAKnotOut with zero padding included because this is where we ship it off to DMA
bits = to_bits(get_FPGAKnotOut_size_bytes(num_links))
bw.writeLine("	      Bit#({bits}) data_out = zeroExtend(out_data.first());".format(bits=bits))
bw.writeLine("        Bit#(256) chunk = data_out[256*(gear_out+1)-1 : 256*(gear_out)];")
bw.writeLine("        if (gear_out == 0)  begin")
# FPGAKnotOut
bytes = get_FPGAKnotOut_size_bytes(num_links)
bw.writeLine("            we.writeServers[0].request.put(MemengineCmd{{sglId: wrPointer, base: {bytes}*offset_out, len: {bytes}, burstLen: 64, tag:0}});".format(bytes=bytes))
###

bw.writeLine("            outstanding_dma <= outstanding_dma + 1;")
bw.writeLine("            offset_out <= offset_out + 1;")
bw.writeLine("            $display(\"Row number\", fshow(offset_out), fshow(outstanding_dma));")
bw.writeLine("            gear_out <= gear_out + 1;")
bw.writeLine("        end ")
bw.writeLine("        else begin")
###
# number of chunks per knot
n_chunks_knot = to_bits(get_FPGAKnotOut_size_bytes(num_links)) // 256
bw.writeLine("            if (gear_out == " + str(n_chunks_knot-1) + ") begin ")
###
bw.writeLine("                gear_out <= 0;")
bw.writeLine("                out_data.deq();")
bw.writeLine("                //$display(\"End Row:\",  fshow(data_out)); ")
bw.writeLine("            end")
bw.writeLine("            else begin")
bw.writeLine("                gear_out <= gear_out + 1;")
bw.writeLine("            end")
bw.writeLine("        end")
bw.writeLine("        $display(\"Push to writeback not converted yet\", fshow(chunk));  ")
bw.writeLine("")
bw.writeLine("`ifndef HOST_FPCONV")
bw.writeLine("        fix_fps.request.put(unpack(chunk));")
bw.writeLine("`else")
bw.writeLine("        we.writeServers[0].data.enq(chunk);")
bw.writeLine("`endif")
bw.writeLine("    endrule")

bw.writeLine("")
bw.writeLine("`ifndef HOST_FPCONV")
bw.writeLine("    rule converFixToFp;")
bw.writeLine("        let complete_input_bitspacked <- fix_fps.response.get();")
bw.writeLine("        we.writeServers[0].data.enq(pack(complete_input_bitspacked));")
bw.writeLine("        //$display(\"Fully converted and sent back \", fshow(complete_input_bitspacked));")
bw.writeLine("    endrule")
bw.writeLine("`endif")
bw.writeLine("")
bw.writeLine("    Reg#(Bool) processing <- mkReg(False);")
bw.writeLine("    rule finish if (offset_out == nbP && outstanding_dma == 0 && processing);")
bw.writeLine("        $display(\"Finish\");")
bw.writeLine("        processing <= False;")
bw.writeLine("        offset_out <= 0;")

###
# FPGAKnotOut
bytes = get_FPGAKnotOut_size_bytes(num_links)
bw.writeLine("	      we.writeServers[0].request.put(MemengineCmd{{sglId:wrPointer, base: {bytes}*nbP , len: 32, burstLen:32, tag:0}});".format(bytes=bytes))
###

bw.writeLine("        we.writeServers[0].data.enq(0);")
bw.writeLine("        outstanding_dma <= outstanding_dma + 1;")
bw.writeLine("    endrule ")
bw.writeLine("")
bw.writeLine("    rule write_done;")
bw.writeLine("        let rv1 <- we.writeServers[0].done.get;")
bw.writeLine("        outstanding_dma <= outstanding_dma - 1;")
bw.writeLine("        $display(\"Request write confirmation engine \", outstanding_dma);")
bw.writeLine("        //outstanding <= outstanding - 1;")
bw.writeLine("    endrule")
bw.writeLine("")
bw.writeLine("    interface Gradient start;")
bw.writeLine("        method Action start64(Bit#(32) dataIn, Bit#(32) dataOut, Bit#(32) nbPoints) if (read_idx_forward == tagged Invalid && !processing);")
bw.writeLine("            $display(\"Start a 64 Step gradient computation\");")
bw.writeLine("            rdPointer <= dataIn;")
bw.writeLine("            processing <= True;")
bw.writeLine("            wrPointer <= dataOut;")
bw.writeLine("            nbP <= nbPoints; ")
bw.writeLine("            read_idx_forward <= tagged Valid(0); ")
bw.writeLine("        endmethod")
bw.writeLine("    endinterface")
bw.writeLine("")
bw.writeLine("    interface MemReadClient dmaReadClient = vec(re.dmaClient);")
bw.writeLine("    interface MemWriteClient dmaWriteClient = vec(we.dmaClient);")
bw.writeLine("endmodule")
bw.writeLine("")
