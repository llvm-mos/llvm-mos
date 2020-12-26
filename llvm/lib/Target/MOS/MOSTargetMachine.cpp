//===-- MOSTargetMachine.cpp - Define TargetMachine for MOS ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "MOSTargetMachine.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"

#include "MOS.h"
#include "MOSTargetObjectFile.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

namespace llvm {

static const char *MOSDataLayout = "e-P1-p:16:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-n8-a:8";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    return "avr2";
  }

  return CPU;
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  return RM.hasValue() ? *RM : Reloc::Static;
}

MOSTargetMachine::MOSTargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Optional<Reloc::Model> RM,
                                   Optional<CodeModel::Model> CM,
                                   CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, MOSDataLayout, TT, getCPU(CPU), FS, Options,
                        getEffectiveRelocModel(RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, getCPU(CPU), FS, *this) {
  this->TLOF = make_unique<MOSTargetObjectFile>();
  initAsmInfo();
}

namespace {
/// MOS Code Generator Pass Configuration Options.
class MOSPassConfig : public TargetPassConfig {
public:
  MOSPassConfig(MOSTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  MOSTargetMachine &getMOSTargetMachine() const {
    return getTM<MOSTargetMachine>();
  }

  bool addInstSelector() override;
  void addPreSched2() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
};
} // namespace

TargetPassConfig *MOSTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MOSPassConfig(*this, PM);
}

extern "C" void LLVMInitializeMOSTarget() {
  // Register the target.
  RegisterTargetMachine<MOSTargetMachine> X(getTheMOSTarget());

  auto &PR = *PassRegistry::getPassRegistry();
  initializeMOSExpandPseudoPass(PR);
  initializeMOSRelaxMemPass(PR);
}

const MOSSubtarget *MOSTargetMachine::getSubtargetImpl() const {
  return &SubTarget;
}

const MOSSubtarget *MOSTargetMachine::getSubtargetImpl(const Function &) const {
  return &SubTarget;
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

bool MOSPassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createMOSISelDag(getMOSTargetMachine(), getOptLevel()));
  // Create the frame analyzer pass used by the PEI pass.
  addPass(createMOSFrameAnalyzerPass());

  return false;
}

void MOSPassConfig::addPreRegAlloc() {
  // Create the dynalloc SP save/restore pass to handle variable sized allocas.
  addPass(createMOSDynAllocaSRPass());
}

void MOSPassConfig::addPreSched2() {
  addPass(createMOSRelaxMemPass());
  addPass(createMOSExpandPseudoPass());
}

void MOSPassConfig::addPreEmitPass() {
  // Must run branch selection immediately preceding the asm printer.
  addPass(&BranchRelaxationPassID);
}

} // end of namespace llvm
