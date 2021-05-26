#! /bin/bash

if [ $# -eq 0 ] 
then
    echo "No args, please pass the Lab Number"
    read lab
fi 

if [ $# -eq 1 ] 
then
    lab=$1
fi 

touch .gtkwaverc
echo "use_big_fonts 1" > .gtkwaverc

output=2018A7PS0235G_Lab$lab

cp ./*.v ./$output.v.bak
mv ./*.v ./$output.v

iverilog -o $output.vvp $output.v
vvp $output.vvp
gtkwave $output.vcd