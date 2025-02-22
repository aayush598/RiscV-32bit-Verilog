@echo off
iverilog -o tb_top.out tb_top.v top.v
vvp tb_top.out
gtkwave waveform.vcd