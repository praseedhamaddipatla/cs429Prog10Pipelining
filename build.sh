#!/bin/bash
set -e

mkdir -p sim vvp

SRCS="tinker.sv"
PASS=0
FAIL=0

run_tb() {
  local name=$1
  local tb_file=$2
  echo ""
  echo "=== $name ==="
  iverilog -g2012 -o vvp/${name}.vvp ${tb_file} ${SRCS}
  vvp vvp/${name}.vvp
  if [ $? -eq 0 ]; then PASS=$((PASS+1)); else FAIL=$((FAIL+1)); fi
}

run_tb "alu_tb"     test/alu_tb.sv
run_tb "regfile_tb" test/regfile_tb.sv
run_tb "mem_tb"     test/mem_tb.sv
run_tb "decoder_tb" test/decoder_tb.sv
run_tb "tinker"  test/tinker_tb.sv

echo ""
echo "=== build summary: $PASS suites ok, $FAIL suites failed ==="