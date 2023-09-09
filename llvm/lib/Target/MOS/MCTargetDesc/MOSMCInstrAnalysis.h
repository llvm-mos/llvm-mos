//===-- MOSMCInstrAnalysis.h - MOS instruction analysis ---------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MOSMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MC_INSTR_ANALYSIS_H
#define LLVM_MOS_MC_INSTR_ANALYSIS_H

#include "llvm/MC/MCInstrAnalysis.h"

namespace llvm {

class Triple;

class MOSMCInstrAnalysis : public MCInstrAnalysis {
public:
  explicit MOSMCInstrAnalysis(const MCInstrInfo *Info)
      : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override;

  std::optional<uint64_t>
  evaluateMemoryOperandAddress(const MCInst &Inst, const MCSubtargetInfo *STI,
                               uint64_t Addr, uint64_t Size) const override;
};

} // end namespace llvm

#endif // LLVM_MOS_MC_INSTR_ANALYSIS_H
