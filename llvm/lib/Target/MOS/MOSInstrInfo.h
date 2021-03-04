//===-- MOSInstrInfo.h - MOS Instruction Information ------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_INSTR_INFO_H
#define LLVM_MOS_INSTR_INFO_H

#include "llvm/CodeGen/TargetInstrInfo.h"

#include "MOSRegisterInfo.h"

#define GET_INSTRINFO_HEADER
#include "MOSGenInstrInfo.inc"
#undef GET_INSTRINFO_HEADER

namespace llvm {

namespace MOS {

/// Specifies a target operand flag.
enum TOF {
  MO_NO_FLAG,
  MO_LO,
  MO_HI,
};

} // namespace MOS

/// Utilities related to the MOS instruction set.
class MOSInstrInfo : public MOSGenInstrInfo {
public:
  explicit MOSInstrInfo();

  std::pair<unsigned, unsigned>
  decomposeMachineOperandsTargetFlags(unsigned TF) const override;

  ArrayRef<std::pair<unsigned, const char *>>
  getSerializableDirectMachineOperandTargetFlags() const override;
};

} // end namespace llvm

#endif // LLVM_MOS_INSTR_INFO_H
