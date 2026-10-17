#!/bin/bash
# Needs: iverilog (sudo apt install iverilog)
set -e
cd "$(dirname "$0")/.."
RTL="rtl/spi_slave.v rtl/adaremac_core.v rtl/mac_array.v rtl/adaremac_mac_lane.v rtl/adaremac_mul.v"
iverilog -g2005 -o sim/tb_mul.vvp  sim/tb_mul.v  rtl/adaremac_mul.v && vvp sim/tb_mul.vvp
iverilog -g2005 -o sim/tb_core.vvp sim/tb_core.v $RTL               && vvp sim/tb_core.vvp
