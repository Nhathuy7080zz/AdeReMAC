#!/bin/bash
# Kiem chung RTL (can Icarus Verilog >= 11):
#   1) tb_mul        : bo nhan AdaReMAC (65.536 cap, 3 che do)
#   2) tb_standalone : demo do vi kien truc chi dung Tang Nano 20K (rtl/microbench)
#   3) tb_fomo_top   : bo tang toc FOMO-Micro qua SPI, so bit-exact voi Python (run_fomo_sim.sh)
set -e
cd "$(dirname "$0")"
IV=${IVERILOG:-iverilog}; VVP=${VVP:-vvp}
MB=../rtl/microbench
$IV -g2005 -o tb_mul.vvp tb_mul.v ../rtl/adaremac_mul.v && $VVP -n tb_mul.vvp
$IV -g2005 -o tb_standalone.vvp tb_standalone.v $MB/top_standalone.v $MB/mac_array.v $MB/adaremac_mac_lane.v \
    $MB/report_tx.v $MB/uart_tx.v $MB/btn_sync.v ../rtl/adaremac_mul.v && $VVP -n tb_standalone.vvp
./run_fomo_sim.sh ${1:-1}
