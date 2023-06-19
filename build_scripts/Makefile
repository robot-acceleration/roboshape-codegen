# This Makefile is for generating, pushing, building, running **all** the generator scripts.
# This is because we don't track the generated output verilog/bluespec files yet,
# so the Makefile can't run the flow for individual scripts and its associated outputs.
# eg: let's say you edit xgenpXc.py and want to generate all its outputs,
# push them to the accelerator repo, build the accel, and run it.
# This Makefile can't do this because `push` needs to know the names of all the output files associated with xgenpXc.py
# This Makefile will also break if you have a recently modified generator script which is broken, and you DO NOT want it to be generated or transferred.
# TODO: add input generator <-> output v/bs file association info
#
# If you want to do things with individual files/generator scripts, use the individual build scripts.

# the grep command lets you exclude commented out files in rbd_accel_generators.txt
SRCS = $(shell grep -v '^\#' rbd_accel_generators_rev3.txt)
BUILD_DIR = ./build

# lots of PHONYs because the target isn't an actual file,
# but an arbitrary name we use to execute the rule.

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

.PHONY: generate
generate : $(BUILD_DIR)
	#echo $(SRCS)
	./generate_file.sh $(SRCS)

.PHONY: push
push :
	./hacky_ci_push.sh all

.PHONY: build_rbd
build_rbd :
	./build_rbd_accel.sh

.PHONY: run_rbd
run_rbd :
	./run_rbd_accel.sh

.PHONY: clean
clean :
	-rm -rf $(BUILD_DIR)
