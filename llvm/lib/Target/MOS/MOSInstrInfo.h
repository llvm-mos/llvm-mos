//===-- MOSInstrInfo.h - MOS Instruction Information ------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
};

} // namespace MOS

/// Utilities related to the MOS instruction set.
class MOSInstrInfo : public MOSGenInstrInfo {
public:
  explicit MOSInstrInfo();
};

} // end namespace llvm

#endif // LLVM_MOS_INSTR_INFO_H
