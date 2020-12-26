//===-- MOSInstrInfo.td - MOS Instruction defs -------------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the MOS instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "MOSInstrFormats.td"

def BRK: MOSInst8< 
    (outs),
    (ins),
    "brk",
    [] >, 
    Requires<[Has6502]>;

def ASL_imm: MOSInst8< 
    (outs),
    (ins),
    "asl",
    [], 
    0x0a>, 
    Requires<[Has6502]>;


def LDA_imm: MOSInst16< 
  (outs),
  (ins imm8:$val),
  "lda\t$val",
  [] >,
  Requires<[Has6502]>
  {
      bits<8> val;
      let Inst{7-0} = 0xa9;
      let Inst{15-8} = val;
  }
  