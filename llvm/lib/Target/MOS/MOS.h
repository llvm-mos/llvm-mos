//===-- MOS.h - Top-level interface for MOS representation ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// MOS back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_H
#define LLVM_MOS_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class MOSTargetMachine;
class FunctionPass;

FunctionPass *createMOSISelDag(MOSTargetMachine &TM,
                               CodeGenOpt::Level OptLevel);
FunctionPass *createMOSExpandPseudoPass();
FunctionPass *createMOSFrameAnalyzerPass();
FunctionPass *createMOSRelaxMemPass();
FunctionPass *createMOSDynAllocaSRPass();
FunctionPass *createMOSBranchSelectionPass();

void initializeMOSExpandPseudoPass(PassRegistry&);
void initializeMOSRelaxMemPass(PassRegistry&);

/// Contains the MOS backend.
namespace MOS {

/// An integer that identifies all of the supported MOS address spaces.
enum AddressSpace { DataMemory, ProgramMemory };

/// Checks if a given type is a pointer to program memory.
template <typename T> bool isProgramMemoryAddress(T *V) {
  return cast<PointerType>(V->getType())->getAddressSpace() == ProgramMemory;
}

inline bool isProgramMemoryAccess(MemSDNode const *N) {
  auto V = N->getMemOperand()->getValue();

  return (V != nullptr) ? isProgramMemoryAddress(V) : false;
}

} // end of namespace MOS

} // end namespace llvm

#endif // LLVM_MOS_H
