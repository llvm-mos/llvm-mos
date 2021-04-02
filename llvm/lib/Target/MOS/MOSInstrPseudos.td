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

// The call-frame pseudoinstructions form a signalling mechanism to pass the
// size of outgoing call frames from calling-convention lowering (in
// IRTranslator) through to the stack frame prolog-epilog insertion. The latter
// examines these pseudoinstructions to find the largest size that an outgoing
// call frame can be, then preallocates that much space to the stack frame if
// possible. This allows these pseudoinstructions to be summarily removed in
// most cases, since we can instead guarantee that a reserved region at the
// bottom of the stack will always have enough free space to set up the
// arguments of any outgoing call.
class CallFramePseudo : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [RS0];
  let Uses = [RS0];
}

// Call frame setup pseudo.
def ADJCALLSTACKDOWN : CallFramePseudo;

// Call frame destroy pseudo.
def ADJCALLSTACKUP : CallFramePseudo;

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

// Loads a value (1, 8, or 16 bits) from the soft stack. Eliminated once the
// offset from the frame register is known.
def LDstk : Pseudo {
  dag OutOperandList = (outs Anyc:$dst);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let mayLoad = true;
}

// Stores a value (1, 8, or 16 bits) to the soft stack. Eliminated once the
// offset from the frame register is known.
def STstk : Pseudo {
  dag InOperandList = (ins Anyc:$src, Imag16:$base, i16imm:$fi);

  // All possible instruction sequences clobber NZ.
  let Defs = [NZ];

  let mayStore = true;
}

// Loads from an absolute location plus an absolute offset. The first operand
// will always be a frame index referring to to a statically allocated stack
// frame. This will be lowered to a regular LDabs upon FEI.
def LDabs_offset : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

// Stores to an absolute location plus an absolute offset. The first operand
// will always be a frame index referring to a statically allocated stack frame.
// This will be lowered to a regular STabs upon FEI.
def STabs_offset : Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList = (ins GPR:$src, i16imm:$addr, i16imm:$offset);

  let mayStore = true;
}

// Abstracts LDAidx, LDXidx, and LDYidx. This is almost a logical instruction,
// but the "LDX 1234,X" and "LDY 1234,Y" cases aren't natively supported, and
// more than one instruction is required to emit them.
def LDidx : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, XY:$idx);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}
