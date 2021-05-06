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
// Post RA Pseudos
//===---------------------------------------------------------------------===//
// These pseudoinstructions defer decisionmaking until after register
// allocation. They're used whenever the best instruction sequence to emit
// depends only on the placement of their arguments and the set of live
// registers before and after the pseudo.
//===---------------------------------------------------------------------===//

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

def LDImm1 : MOSPseudo {
  dag OutOperandList = (outs Anyi1:$dst);
  dag InOperandList = (ins i1imm:$val);

  let Defs = [NZ];

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
  let isReMaterializable = true;
}

// Zero-extend a 1-bit value to 8 bits. May be emitted using the diamond pattern
// over a pair of immediate loads. Some argument combinations can be more
// directly emitted though.
def ZExt1 : MOSPseudo {
  dag OutOperandList = (outs Anyi8:$dst);
  dag InOperandList = (ins Anyi1:$src);

  let Defs = [NZ];
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

//===---------------------------------------------------------------------===//
// NZ
//===---------------------------------------------------------------------===//

// Version of SBCImag8 that copies the N and Z flags out to virtual registers.
// This bundles the definition and use of N and Z together to maintain the
// invariant that neither can be live across pseudoinstructions. At most one of
// N or Z can be set (the other must be $noreg), since copying N out usually
// clobbers Z, and vice versa.
def SBCNZImag8 : MOSPseudo {
  dag OutOperandList = (outs Ac:$dst,
                        Cc:$carryout, Anyi1:$nout, Vc:$vout, Anyi1:$zout);
  dag InOperandList = (ins Ac:$l, Imag8:$r, Cc:$carryin);
  let Constraints = "$dst = $l, $carryout = $carryin";
}

// Terminator version of CMPimm. This ensures that this instruction stays
// adjacent to the branch that consumes its N or Z value.
def CMPImmTerm : MOSPseudo {
  dag OutOperandList = (outs Cc:$carry);
  dag InOperandList = (ins GPR:$l, i8imm:$r);

  let Defs = [NZ];

  let isCompare = true;

  let isTerminator = true;
}

//===---------------------------------------------------------------------===//
// Control flow
//===---------------------------------------------------------------------===//

// Use the diamond control flow pattern to load one of two immediates based on
// the value of a flag. The output can be either Anyi1 or Anyi8.
def SelectImm : MOSPseudo {
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins Flag:$tst, i8imm:$true, i8imm:$false);

  let usesCustomInserter = true;
}
