//===-- MOSMCAsmInfo.h - MOS asm properties ---------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MOSMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_ASM_INFO_H
#define LLVM_MOS_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

/// Specifies the format of MOS assembly files.
class MOSMCAsmInfo : public MCAsmInfo {
public:
  explicit MOSMCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // end namespace llvm

#endif // LLVM_MOS_ASM_INFO_H
