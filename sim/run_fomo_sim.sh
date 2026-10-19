#!/bin/bash
# Mo phong bo tang toc FOMO-Micro (can Icarus Verilog >= 11). Chay tu thu muc sim/:
#   ./run_fomo_sim.sh [so_vector]
set -e
cd "$(dirname "$0")"
IV=${IVERILOG:-iverilog}; VVP=${VVP:-vvp}
$IV -g2012 -I ../rtl -DFOMO_MEM_DIR=\"../rtl/mem/\" -o tb_fomo_top.vvp tb_fomo_top.v \
    ../rtl/fomo_top_tn20k.v ../rtl/fomo_spi_if.v ../rtl/fomo_accel_core.v ../rtl/fomo_mac_lane.v ../rtl/adaremac_mul.v
$VVP -n tb_fomo_top.vvp +nvec=${1:-1}
