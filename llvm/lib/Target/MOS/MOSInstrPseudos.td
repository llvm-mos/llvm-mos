//===-- MOSInstrPseudos.td - Describe MOS Pseudo-Instrs ---*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes the MOS pseudo-instructions.
//
// Pseudo-instructions allow deferring generation of specific instruction
// sequences from one pass until a later pass. This is useful whenever an early
// pass is required to make a decision that it doesn't have the information
// necessary to do well.
//
//===---------------------------------------------------------------------===//

class Pseudo : Instruction {
  let Namespace = "MOS";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

// Frame Pseudoinstructions

def LDstk : Pseudo {
  dag OutOperandList = (outs Anyc:$dst);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let mayLoad = true;
}

def STstk : Pseudo {
  dag InOperandList = (ins Anyc:$src, Imag16:$base, i16imm:$fi);

  let Defs = [NZ];

  let mayStore = true;
}

// Produces the low byte of the address of the given frame index. Produces a
// carry required by the high byte calculation.
def AddrLostk : Pseudo {
  dag OutOperandList = (outs Anyi8:$dst, Cc:$carry);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Produces the high byte of the address of the given frame index. Accepts the
// carry produced by the low byte calculation.
def AddrHistk : Pseudo {
  dag OutOperandList = (outs Anyi8:$dst);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset, Cc:$carry);

  let Defs = [C, NZ];

  let isReMaterializable = true;
}

// Soft stack

// These are the call-frame setup and teardown pseudoinstructions for the MOS
// target. They form a signalling mechanism used to pass the size of outgoing
// call frames all the way from calling-convention lowering (in IRTranslator)
// through to the stack frame prolog-epilog insertion. The later examines these
// pseudoinstructions to find the largest size that an outgoing call frame can
// be, then preallocates that much space to the stack frame if possible. This
// allows these pseudoinstructions to be removed wholesale in most cases, since
// we can instead guarantee that the bottom-most bytes of the stack will always
// have enough free space to set up the arguments of any outgoing call. This
// technique is called call frame reservation.

def ADJCALLSTACKDOWN : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [RS0];
  let Uses = [RS0];
}

def ADJCALLSTACKUP : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [RS0];
  let Uses = [RS0];
}

// Miscellaneous pseudoinstructions

def LDabs_offset : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def STabs_offset : Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList = (ins GPR:$src, i16imm:$addr, i16imm:$offset);

  let mayStore = true;
}

// $dst = *(addr + (unsigned char)idx);
// Sets NZ to undefined value, since expansion may clobber.
def LDidx : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, XY:$idx);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}
