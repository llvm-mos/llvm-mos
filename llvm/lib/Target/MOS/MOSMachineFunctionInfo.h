//===-- MOSMachineFuctionInfo.h - MOS machine function info -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares MOS-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MACHINE_FUNCTION_INFO_H
#define LLVM_MOS_MACHINE_FUNCTION_INFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

/// Contains MOS-specific information for each MachineFunction.
class MOSMachineFunctionInfo : public MachineFunctionInfo {
  /// Indicates if a register has been spilled by the register
  /// allocator.
  bool HasSpills;

  /// Indicates if there are any fixed size allocas present.
  /// Note that if there are only variable sized allocas this is set to false.
  bool HasAllocas;

  /// Indicates if arguments passed using the stack are being
  /// used inside the function.
  bool HasStackArgs;

  /// Size of the callee-saved register portion of the
  /// stack frame in bytes.
  unsigned CalleeSavedFrameSize;

  /// FrameIndex for start of varargs area.
  int VarArgsFrameIndex;

public:
  MOSMachineFunctionInfo()
      : HasSpills(false), HasAllocas(false), HasStackArgs(false),
        CalleeSavedFrameSize(0), VarArgsFrameIndex(0) {}

  explicit MOSMachineFunctionInfo(MachineFunction &MF)
      : HasSpills(false), HasAllocas(false), HasStackArgs(false),
        CalleeSavedFrameSize(0), VarArgsFrameIndex(0) {}

  bool getHasSpills() const { return HasSpills; }
  void setHasSpills(bool B) { HasSpills = B; }

  bool getHasAllocas() const { return HasAllocas; }
  void setHasAllocas(bool B) { HasAllocas = B; }

  bool getHasStackArgs() const { return HasStackArgs; }
  void setHasStackArgs(bool B) { HasStackArgs = B; }

  unsigned getCalleeSavedFrameSize() const { return CalleeSavedFrameSize; }
  void setCalleeSavedFrameSize(unsigned Bytes) { CalleeSavedFrameSize = Bytes; }

  int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
  void setVarArgsFrameIndex(int Idx) { VarArgsFrameIndex = Idx; }
};

} // end llvm namespace

#endif // LLVM_MOS_MACHINE_FUNCTION_INFO_H
