#!/usr/bin/python3
from URDFGenerator import URDFGenerator

do_chain_length_generate = False
do_handle_length_generate = False
do_torso_length_generate = True
do_quadruped_length_generate = False
do_quadruped_manip_length_generate = False
do_tine_length_generate = False

if do_chain_length_generate:
    # chain 1-12 links
    chain_length_sweep = list(range(1,13))
    for chain_length in chain_length_sweep:
        urdf_gen = URDFGenerator("chain", chain_length, filepath="./chain_length_sweep")
        urdf_gen.generate_file()

if do_handle_length_generate:
    # fork handle 1-6 links, tine length 3
    handle_length_sweep = list(range(1,7))
    for handle_length in handle_length_sweep:
        urdf_gen = URDFGenerator("fork", handle_length+6, [handle_length, 3, 3], filepath="./handle_length_sweep")
        urdf_gen.generate_file()

if do_torso_length_generate:
    # torso limb length 1-10 links
    torso_length_sweep = list(range(1,11))
    for torso_length in torso_length_sweep:
        urdf_gen = URDFGenerator("torso", 1+torso_length*2, [1, torso_length, torso_length], filepath="./torso_length_sweep")
        urdf_gen.generate_file()

if do_quadruped_length_generate:
    # quadruped limb length 1-10 links
    quadruped_length_sweep = list(range(1,11))
    for quadruped_length in quadruped_length_sweep:
        urdf_gen = URDFGenerator("quadruped", quadruped_length*4, [quadruped_length, quadruped_length, quadruped_length, quadruped_length], filepath="./quadruped_length_sweep")
        urdf_gen.generate_file()

if do_quadruped_manip_length_generate:
    # quadruped limb length 1-10 links + 7-link arm
    quadruped_length_sweep = list(range(1,11))
    for quadruped_length in quadruped_length_sweep:
        urdf_gen = URDFGenerator("quadruped_manip", 7+quadruped_length*4, [7, quadruped_length, quadruped_length, quadruped_length, quadruped_length], filepath="./quadruped_manip_length_sweep")
        urdf_gen.generate_file()

if do_tine_length_generate:
    # handle length = 6, tines sweep from 1-4
    tine_length_sweep = list(range(1, 5))
    for tine_length in tine_length_sweep:
        urdf_gen = URDFGenerator("arm_3_fingers", 6+tine_length*3, [6, tine_length, tine_length, tine_length], filepath="./tine_length_sweep")
        urdf_gen.generate_file()
