//===-- MOSTargetMachine.h - Define TargetMachine for MOS -------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_TARGET_MACHINE_H
#define LLVM_MOS_TARGET_MACHINE_H

#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#include "MOSFrameLowering.h"
#include "MOSISelLowering.h"
#include "MOSInstrInfo.h"
#include "MOSSubtarget.h"

namespace llvm {

/// A generic MOS implementation.
class MOSTargetMachine : public LLVMTargetMachine {
public:
  MOSTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   std::optional<Reloc::Model> RM,
                   std::optional<CodeModel::Model> CM, CodeGenOptLevel OL,
                   bool JIT);

  const MOSSubtarget *getSubtargetImpl() const { return &SubTarget; }
  const MOSSubtarget *getSubtargetImpl(const Function &F) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return this->TLOF.get();
  }

  TargetTransformInfo getTargetTransformInfo(const Function &F) const override;

  bool hasNoInitSection() const override { return true; }

  void registerPassBuilderCallbacks(PassBuilder &) override;

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  // The 6502 has only register-related scheduling concerns, so disable PostRA
  // scheduling by claiming to emit it ourselves, then never doing so.
  bool targetSchedulesPostRAScheduling() const override { return true; };

  StringRef getSectionPrefix(const GlobalObject *GO) const override;

  MachineFunctionInfo *
  createMachineFunctionInfo(BumpPtrAllocator &Allocator, const Function &F,
                            const TargetSubtargetInfo *STI) const override;

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  MOSSubtarget SubTarget;
  mutable StringMap<std::unique_ptr<MOSSubtarget>> SubtargetMap;
};

} // end namespace llvm

#endif // LLVM_MOS_TARGET_MACHINE_H
