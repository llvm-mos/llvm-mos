//===-- MOSTargetMachine.cpp - Define TargetMachine for MOS ---------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "MOSTargetMachine.h"

#include "llvm/CodeGen/GlobalISel/CSEInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/Localizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Transforms/Scalar/IndVarSimplify.h"
#include "llvm/Transforms/Utils.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSCombiner.h"
#include "MOSIndexIV.h"
#include "MOSInsertCopies.h"
#include "MOSLateOptimization.h"
#include "MOSLowerSelect.h"
#include "MOSMachineScheduler.h"
#include "MOSNoRecurse.h"
#include "MOSPostRAScavenging.h"
#include "MOSStaticStackAlloc.h"
#include "MOSTargetObjectFile.h"
#include "MOSTargetTransformInfo.h"

using namespace llvm;

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMOSTarget() {
  // Register the target.
  RegisterTargetMachine<MOSTargetMachine> X(getTheMOSTarget());

  PassRegistry &PR = *PassRegistry::getPassRegistry();
  initializeGlobalISel(PR);
  initializeMOSCombinerPass(PR);
  initializeMOSInsertCopiesPass(PR);
  initializeMOSLateOptimizationPass(PR);
  initializeMOSLowerSelectPass(PR);
  initializeMOSNoRecursePass(PR);
  initializeMOSPostRAScavengingPass(PR);
  initializeMOSStaticStackAllocPass(PR);
}

static const char *MOSDataLayout =
    "e-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
  return (CPU.empty() || CPU == "generic") ? "mos6502" : CPU;
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
      SubTarget(TT, getCPU(CPU).str(), FS.str(), *this) {
  this->TLOF = std::make_unique<MOSTargetObjectFile>();

  initAsmInfo();

  setGlobalISel(true);
  // Prevents fallback to SelectionDAG by allowing direct aborts.
  setGlobalISelAbort(GlobalISelAbortMode::Enable);
}

const MOSSubtarget *
MOSTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  auto CPU = CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  auto FS = FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = std::make_unique<MOSSubtarget>(TargetTriple, CPU, FS, *this);
  }
  return I.get();
}

TargetTransformInfo
MOSTargetMachine::getTargetTransformInfo(const Function &F) {
  return TargetTransformInfo(MOSTTIImpl(this, F));
}

void MOSTargetMachine::registerPassBuilderCallbacks(PassBuilder &PB) {
  PB.registerPipelineParsingCallback(
      [](StringRef Name, LoopPassManager &PM,
         ArrayRef<PassBuilder::PipelineElement>) {
        if (Name == "mos-indexiv") {
          // Rewrite pointer artithmetic in loops to use 8-bit IV offsets.
          PM.addPass(MOSIndexIV());
          return true;
        }
        return false;
      });

  PB.registerLateLoopOptimizationsEPCallback(
      [](LoopPassManager &PM, OptimizationLevel Level) {
        if (Level != OptimizationLevel::O0) {
          PM.addPass(MOSIndexIV());

          // New induction variables may have been added.
          PM.addPass(IndVarSimplifyPass());
        }
      });
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

namespace {
/// MOS Code Generator Pass Configuration Options.
class MOSPassConfig : public TargetPassConfig {
public:
  MOSPassConfig(MOSTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  MOSTargetMachine &getMOSTargetMachine() const {
    return getTM<MOSTargetMachine>();
  }

  void addIRPasses() override;
  bool addPreISel() override;
  bool addIRTranslator() override;
  void addPreLegalizeMachineIR() override;
  bool addLegalizeMachineIR() override;
  void addPreRegBankSelect() override;
  bool addRegBankSelect() override;
  void addPreGlobalInstructionSelect() override;
  bool addGlobalInstructionSelect() override;

  // Register pressure is too high around calls to work without detailed
  // scheduling.
  bool alwaysRequiresMachineScheduler() const override { return true; }

  void addMachineSSAOptimization() override;

  // Register pressure is too high to work without optimized register
  // allocation.
  void addFastRegAlloc() override { addOptimizedRegAlloc(); }
  void addOptimizedRegAlloc() override;

  void addPreSched2() override;
  void addPreEmitPass() override;

  ScheduleDAGInstrs *
  createMachineScheduler(MachineSchedContext *C) const override;

  std::unique_ptr<CSEConfigBase> getCSEConfig() const override;
};
} // namespace

TargetPassConfig *MOSTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MOSPassConfig(*this, PM);
}

void MOSPassConfig::addIRPasses() {
  // Aggressively find provably non-recursive functions.
  addPass(createMOSNoRecursePass());
  TargetPassConfig::addIRPasses();
}

bool MOSPassConfig::addPreISel() {
  return false;
}

bool MOSPassConfig::addIRTranslator() {
  addPass(new IRTranslator(getOptLevel()));
  return false;
}

void MOSPassConfig::addPreLegalizeMachineIR() { addPass(createMOSCombiner()); }

bool MOSPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

void MOSPassConfig::addPreRegBankSelect() {
  addPass(createMOSCombiner());
  addPass(createMOSLowerSelectPass());
}

bool MOSPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

void MOSPassConfig::addPreGlobalInstructionSelect() {
  // This pass helps reduce the live ranges of constants to within a basic
  // block, which can greatly improve machine scheduling, as they can now be
  // moved around to keep register pressure low.
  addPass(new Localizer());
}

bool MOSPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}

void MOSPassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();
  addPass(createMOSInsertCopiesPass());
}

void MOSPassConfig::addOptimizedRegAlloc() {
  // Run the coalescer twice to coalesce RMW patterns revealed by the first coalesce.
  insertPass(&llvm::TwoAddressInstructionPassID, &llvm::RegisterCoalescerID);
  TargetPassConfig::addOptimizedRegAlloc();
}

void MOSPassConfig::addPreSched2() {
  addPass(createMOSPostRAScavengingPass());
  // Lower control flow pseudos.
  addPass(&FinalizeISelID);
  // Lower pseudos produced by control flow pseudos.
  addPass(&ExpandPostRAPseudosID);
  addPass(createMOSLateOptimizationPass());
  addPass(createMOSStaticStackAllocPass());
}

void MOSPassConfig::addPreEmitPass() { addPass(&BranchRelaxationPassID); }

ScheduleDAGInstrs *
MOSPassConfig::createMachineScheduler(MachineSchedContext *C) const {
  return new ScheduleDAGMILive(C, std::make_unique<MOSSchedStrategy>(C));
}

namespace {

class MOSCSEConfigFull : public CSEConfigFull {
public:
  virtual ~MOSCSEConfigFull() = default;
  virtual bool shouldCSEOpc(unsigned Opc) override;
};

bool MOSCSEConfigFull::shouldCSEOpc(unsigned Opc) {
  switch (Opc) {
  default:
    return CSEConfigFull::shouldCSEOpc(Opc);
  case MOS::G_DEC:
  case MOS::G_INC:
  case MOS::G_LSHRE:
  case MOS::G_SBC:
  case MOS::G_SHLE:
    return true;
  }
}

} // namespace

std::unique_ptr<CSEConfigBase> MOSPassConfig::getCSEConfig() const {
  if (TM->getOptLevel() == CodeGenOpt::None)
    return std::make_unique<CSEConfigConstantOnly>();
  return std::make_unique<MOSCSEConfigFull>();
}
