set outputDir ./synth_results
file mkdir $outputDir
set num_links [lindex $argv 0]
set rbd_base $::env(RBD_BASE)
read_verilog [glob $rbd_base/hdl/verilog/rev2/common/*.v ]
read_verilog [glob $rbd_base/hdl/verilog/rev2/bproc/*.v ]
read_verilog [glob $rbd_base/hdl/verilog/rev2/fproc/*.v ]
synth_design -top fproc -part xcvu9p-flga2104-1-e
report_utilization -file $outputDir/fproc_post_synth_util_$num_links.xml -hierarchical -format xml
synth_design -top bproc -part xcvu9p-flga2104-1-e
report_utilization -file $outputDir/bproc_post_synth_util_$num_links.xml -hierarchical -format xml
