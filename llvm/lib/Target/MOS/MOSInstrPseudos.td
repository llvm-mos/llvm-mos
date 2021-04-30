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

class MOSPseudo : Instruction {
  let Namespace = "MOS";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

//===---------------------------------------------------------------------===//
// Soft stack
//===---------------------------------------------------------------------===//

// The call-frame pseudoinstructions form a signalling mechanism to pass the
// size of outgoing call frames from calling-convention lowering (in
// IRTranslator) through to the stack frame prolog-epilog insertion. The latter
// examines these pseudoinstructions to find the largest size that an outgoing
// call frame can be, then preallocates that much space to the stack frame if
// possible. This allows these pseudoinstructions to be summarily removed in
// most cases, since we can instead guarantee that a reserved region at the
// bottom of the stack will always have enough free space to set up the
// arguments of any outgoing call.
class MOSCallFramePseudo : MOSPseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [RS0];
  let Uses = [RS0];
}

// Call frame setup pseudo.
def ADJCALLSTACKDOWN : MOSCallFramePseudo;

// Call frame destroy pseudo.
def ADJCALLSTACKUP : MOSCallFramePseudo;

// Produces the low byte of the address of the given frame index. Produces a
// carry required by the high byte calculation.
def AddrLostk : MOSPseudo {
  dag OutOperandList = (outs Anyi8:$dst, Cc:$carry, Vc:$deadv);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Produces the high byte of the address of the given frame index. Accepts the
// carry produced by the low byte calculation.
def AddrHistk : MOSPseudo {
  dag OutOperandList = (outs Anyi8:$dst, Cc:$deadc, Vc:$deadv);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset, Cc:$carry);

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Loads a value (1, 8, or 16 bits) from the soft stack. Eliminated once the
// offset from the frame register is known.
def LDstk : MOSPseudo {
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let mayLoad = true;
}

// Stores a value (1, 8, or 16 bits) to the soft stack. Eliminated once the
// offset from the frame register is known.
def STstk : MOSPseudo {
  dag InOperandList = (ins unknown:$src, Imag16:$base, i16imm:$fi);

  // All possible instruction sequences clobber NZ.
  let Defs = [NZ];

  let mayStore = true;
}

// Loads from an absolute location plus an absolute offset. The first operand
// will always be a frame index referring to to a statically allocated stack
// frame. This will be lowered to a regular LDAbs upon FEI.
def LDabs_offset : MOSPseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

// Stores to an absolute location plus an absolute offset. The first operand
// will always be a frame index referring to a statically allocated stack frame.
// This will be lowered to a regular STAbs upon FEI.
def STabs_offset : MOSPseudo {
  dag OutOperandList = (outs);
  dag InOperandList = (ins GPR:$src, i16imm:$addr, i16imm:$offset);

  let mayStore = true;
}

// Abstracts LDAIdx, LDXIdx, and LDYIdx. This is almost a logical instruction,
// but the "LDX 1234,X" and "LDY 1234,Y" cases aren't natively supported, and
// more than one instruction is required to emit them.
def LDIdx : MOSPseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, XY:$idx);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

//===---------------------------------------------------------------------===//
// Control flow
//===---------------------------------------------------------------------===//

// Control flow cannot be generally added during instruction selection without
// encoding it in pseudoinstructions, since the passes involved generally aren't
// safe to add basic blocks while iterating through them. Instead, these
// pseudoinstructions are emitted, which are later lowered to new basic blocks
// and control flow in FinalizeISel via custom inserters.
class MOSControlFlowPseudo : MOSPseudo {
  let usesCustomInserter = true;
}

// V can only be copied to other locations via branches and immediate loads.
def CopyFromV : MOSControlFlowPseudo {
  dag OutOperandList = (outs CGPR:$dst);
  dag InOperandList = (ins);

  let Uses = [V];
}

//===---------------------------------------------------------------------===//
// NZ
//===---------------------------------------------------------------------===//

// Version of SBCImag8 that copies the N and Z flags out to virtual registers.
// This bundles the definition and use of N and Z together to maintain the invariant
// that neither can be live across pseudoinstructions.
def SBCNZImag8 : MOSPseudo {
  dag OutOperandList = (outs Ac:$dst,
                        Cc:$carryout, Anyi1:$nout, Vc:$vout, Anyi1:$zout);
  dag InOperandList = (ins Ac:$l, Imag8:$r, Cc:$carryin);
  let Constraints = "$dst = $l, $carryout = $carryin";
}
