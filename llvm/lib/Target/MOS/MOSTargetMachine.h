//===-- MOSTargetMachine.h - Define TargetMachine for MOS -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "MOSSelectionDAGInfo.h"
#include "MOSSubtarget.h"

namespace llvm {

/// A generic MOS implementation.
class MOSTargetMachine : public LLVMTargetMachine {
public:
  MOSTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Optional<Reloc::Model> RM,
                   Optional<CodeModel::Model> CM,
                   CodeGenOpt::Level OL, bool JIT);

  const MOSSubtarget *getSubtargetImpl() const;
  const MOSSubtarget *getSubtargetImpl(const Function &) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return this->TLOF.get();
  }

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  bool isMachineVerifierClean() const override {
    return false;
  }

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  MOSSubtarget SubTarget;
};

} // end namespace llvm

#endif // LLVM_MOS_TARGET_MACHINE_H
