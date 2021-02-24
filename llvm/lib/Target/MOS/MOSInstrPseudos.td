//===-- MOSInstrPseudos.td - MOS Pseudo-Instr Defs ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the pseudo-instructions for the MOS architecture.
//
// Pseudo-instructions allow the target to defer handling a situation until a
// later phase of the compiler. An earlier phase records the pseudo instruction,
// and the instruction is carried along until a later phase expands it into a
// real instruction sequence. It's often easier to make such expansions later,
// as more is known about the register allocation, instruction schedule, and
// stack frame. LLVM also has a habit of presuming that certain operations are
// "easy" for a target to emit without causing side effects. Wrapping these in
// pseudo instructions allows a later phase to figure out how to actually meet
// the constraints that LLVM expected the operation to have. These constraints
// are encoded directly into the uses and defs of the pseudo-instructions
// below. This ensures that any pseudo expasion that appears to have the same
// set of effects as the pseudo itself is correct, no matter how it goes about
// its business.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Soft Stack Pseudo-instructions
//===----------------------------------------------------------------------===//

class Pseudo : Instruction {
  let Namespace = "MOS";
  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isCodeGenOnly = true;
  let isPseudo = true;
}

// Decreases the stack pointer to accomodate $bytes bytes of outgoing arguments
// passed through the stack. $prior is the number of bytes handled implicitly by
// the call; this is always zero on this target.
def ADJCALLSTACKDOWN : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);
  dag OutOperandList = (outs);

  let Uses = [RS0];
  let Defs = [RS0];
}

// Increases the stack pointer to clean up $bytes bytes of outgoing arguments
// that were passed through the stack. $prior is the number of bytes handled
// implicitly by the call; this is always zero on this target.
def ADJCALLSTACKUP : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);
  dag OutOperandList = (outs);

  let Uses = [RS0];
  let Defs = [RS0];
}