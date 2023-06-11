; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos --mcpu=mos4510 < %s | FileCheck %s

	map                             ; CHECK: encoding: [0x5c]
