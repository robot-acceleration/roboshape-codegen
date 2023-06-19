## Accelerator generation

### Helper script workflow

```bash
# these should be put in a source script
# RBD_BASE and AUTOMATION_PREFIX will be different for you
export RBD_BASE=/home/radhika/robomorphic_stuff/rbd-accelerator
export AUTOMATION_PREFIX=/home/radhika/robomorphic_stuff/robomorphic-automation/scripts
export PYTHONPATH=$PYTHONPATH:$AUTOMATION_PREFIX
```

For running the entire flow:
```bash
$ cd build_scripts
$ make generate # generate all the verilog/bluespec from the py files
$ make push # push the generated files to rbd-accelerator repo
$ make build_rbd # builds the accelerator using the verilator target
$ make run_rbd # uses verilator to simulate the output
```

Haven't tested `make build_rbd` and `make run_rbd` using vivado.

---

0. Write the Python file for generating its corresponding Bluespec/Verilog output.
    * Put the .py file in the correct directory in `generators`, relative to its location in the rbd-accelerator repo.
    * eg: `fproc.py` goes in `generators/hdl/verilog/rev2/fproc`.
1. `$ ./generate_file.sh ./generators/[path to py file]`
    * eg: `./generate_file.sh ./generators/hdl/verilog/rev2/fproc/fproc.py`
    * This will generate `fproc.v` and send it to `build/hdl/verilog/rev2/fproc`.
2. `$ ./hacky_ci_push.sh ./build/hdl/verilog/rev2/fproc/fproc.v`
    * This will send `fproc.v` to `<rbd-accelerator repo>/hdl/verilog/rev2/fproc/fproc.v`
    * Also creates an `fproc.v.backup` from original rbd-accel HEAD in the same directory if you need it for reference.
    * Then kicks off a new verilator build and runs the simulation.
3. `$ ./hacky_ci_revert.sh ./build/hdl/verilog/rev2/fproc/fproc.v`
    * This will restore `fproc.v` in rbd-accelerator back to its original handwritten version from git head.
4. `$ ./hacky_ci_revert.sh all`
    * This will restore the entire rbd-accelerator back to git head and remove all the backup files. Use with care, it will erase any uncommitted changes in the repo.
