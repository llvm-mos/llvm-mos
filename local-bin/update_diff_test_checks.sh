#!/bin/bash
set -x
for x; do
  bn=$(basename $x)
  cp /home/mysterymath/llvm-mos/build/test/CodeGen/MOS/Output/$bn.tmp \
     /home/mysterymath/llvm-mos/llvm/test/CodeGen/MOS/Inputs/${bn%%.*}-expected.s
done
