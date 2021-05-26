#! /bin/bash
iverilog -s testbench -o 2018A7PS0235G_Lab5.vvp testbench.v
vvp 2018A7PS0235G_Lab5.vvp
gtkwave 2018A7PS0235G_Lab5.vcd
