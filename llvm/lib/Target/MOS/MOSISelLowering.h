//===-- MOSISelLowering.h - MOS DAG Lowering Interface ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MOS uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_ISEL_LOWERING_H
#define LLVM_MOS_ISEL_LOWERING_H

#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

namespace MOSISD {

/// MOS Specific DAG Nodes
enum NodeType {
  /// Start the numbering where the builtin ops leave off.
  FIRST_NUMBER = ISD::BUILTIN_OP_END,
  /// Return from subroutine.
  RET_FLAG,
};

} // end of namespace MOSISD

class MOSSubtarget;
class MOSTargetMachine;

/// Performs target lowering for the MOS.
class MOSTargetLowering : public TargetLowering {
public:
  explicit MOSTargetLowering(const MOSTargetMachine &TM,
                             const MOSSubtarget &STI);
protected:
  const MOSSubtarget &Subtarget;
};

} // end namespace llvm

#endif // LLVM_MOS_ISEL_LOWERING_H
