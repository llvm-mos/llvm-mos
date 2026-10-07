//===-- MOSCallTreeVerify.cpp - MOS Call Tree Verification ----------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS call tree verification pass.
//
// Every function reachable from an interrupt_norecurse("suffix") root must
// carry "rc-suffix"="suffix", or be a declaration marked "interrupt-safe".
// Otherwise it would use the main imaginary-register set and clobber the
// interrupted code's state. MOSCallTreeClone establishes this for IR calls, and
// MOSCallTreeLibcalls for libcalls created by the legalizer. This pass runs
// after MOSInternalize, when the libcalls actually used are known and unused
// speculative clones are gone, and checks the result:
//
//  * Calls to unsuffixed functions, and libcalls with no bitcode definition to
//    clone, are errors.
//  * Errors that MOSCallTreeClone deferred ("mos-call-tree-error") are reported
//    if the offending clone is reachable.
//  * Calls to "interrupt-safe" declarations must pass arguments and return
//    values only in A/X/Y.
//
// It also marks the root "isr-init-soft-stack" if any reachable function uses
// a soft stack frame, so the prologue initializes the private stack pointer.
//
//===----------------------------------------------------------------------===//

#include "MOSCallTreeVerify.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSCallGraphUtils.h"
#include "MOSFrameLowering.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Module.h"
#include "llvm/LTO/LTO.h"
#include "llvm/Pass.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mos-call-tree-verify"

using namespace llvm;

namespace {

class MOSCallTreeVerify : public ModulePass {
public:
  static char ID;

  MOSCallTreeVerify() : ModulePass(ID) {
    initializeMOSCallTreeVerifyPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<MachineModuleInfoWrapperPass>();
    AU.setPreservesAll();
  }

  bool runOnModule(Module &M) override;
};

// A call from one function to another, as seen in either IR or MIR.
struct CallEdge {
  // The callee, or nullptr if the symbol has no definition or declaration.
  Function *Callee;
  // The symbol name called.
  StringRef Name;
  // The MIR call instruction, if any.
  const MachineInstr *MI;
};

} // namespace

char MOSCallTreeVerify::ID = 0;

INITIALIZE_PASS(MOSCallTreeVerify, DEBUG_TYPE,
                "Verify suffixed MOS ISR call trees use their private "
                "register set",
                false, false)

ModulePass *llvm::createMOSCallTreeVerifyPass() {
  return new MOSCallTreeVerify();
}

// Collects the calls made by F. Uses the MIR when available, since it
// includes libcalls created by the legalizer.
static void collectCalls(Function &F, const MachineModuleInfo &MMI,
                         SmallVectorImpl<CallEdge> &Calls) {
  Module &M = *F.getParent();
  if (const MachineFunction *MF = MMI.getMachineFunction(F)) {
    for (const MachineBasicBlock &MBB : *MF) {
      for (const MachineInstr &MI : MBB) {
        if (!MI.isCall())
          continue;
        for (const MachineOperand &MO : MI.operands()) {
          if (MO.isGlobal()) {
            StringRef Name = MO.getGlobal()->getName();
            if (Function *Callee = mos::getSymbolFunction(M, Name))
              Calls.push_back(CallEdge{Callee, Name, &MI});
          } else if (MO.isSymbol()) {
            Calls.push_back(
                CallEdge{mos::getSymbolFunction(M, MO.getSymbolName()),
                         MO.getSymbolName(), &MI});
          }
        }
      }
    }
    return;
  }

  for (Instruction &I : instructions(F)) {
    auto *CB = dyn_cast<CallBase>(&I);
    if (!CB || CB->isInlineAsm())
      continue;
    auto *Callee = dyn_cast<Function>(
        CB->getCalledOperand()->stripPointerCastsAndAliases());
    if (Callee && !Callee->isIntrinsic())
      Calls.push_back(CallEdge{Callee, Callee->getName(), nullptr});
  }
}

// Returns whether a call passes arguments or return values in imaginary
// registers.
static bool usesImagRegs(const MachineInstr &MI) {
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.isImplicit() || !MO.getReg().isPhysical())
      continue;
    if (MOS::Imag8RegClass.contains(MO.getReg()) ||
        MOS::Imag16RegClass.contains(MO.getReg()))
      return true;
  }
  return false;
}

bool MOSCallTreeVerify::runOnModule(Module &M) {
  SmallVector<Function *, 4> Roots;
  for (Function &F : M)
    if (F.hasFnAttribute("interrupt-norecurse") &&
        F.hasFnAttribute("rc-suffix"))
      Roots.push_back(&F);
  if (Roots.empty())
    return false;

  const MachineModuleInfo &MMI =
      getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  StringSet<> LibcallNames;
  for (const char *Name :
       lto::LTO::getRuntimeLibcallSymbols(Triple(M.getTargetTriple())))
    LibcallNames.insert(Name);

  bool Changed = false;
  for (Function *Root : Roots) {
    StringRef Sfx = Root->getFnAttribute("rc-suffix").getValueAsString();

    auto error = [&](Function &F, const Twine &Msg) {
      M.getContext().diagnose(DiagnosticInfoUnsupported(
          F, "[MOS interrupt] " + Msg +
                 " (reachable from interrupt handler \"" + Root->getName() +
                 "\")"));
    };

    bool NeedsSoftStack = false;
    SmallPtrSet<Function *, 32> Visited;
    SmallVector<Function *, 32> Worklist = {Root};
    while (!Worklist.empty()) {
      Function *F = Worklist.pop_back_val();
      if (!Visited.insert(F).second)
        continue;

      // Functions MOSCallTreeClone found problems in are only reported once.
      if (F->hasFnAttribute("mos-call-tree-reported"))
        continue;
      Attribute Deferred = F->getFnAttribute("mos-call-tree-error");
      if (Deferred.isValid()) {
        M.getContext().diagnose(
            DiagnosticInfoUnsupported(*F, Deferred.getValueAsString()));
        continue;
      }

      if (const MachineFunction *MF = MMI.getMachineFunction(*F)) {
        const auto &TFL = static_cast<const MOSFrameLowering &>(
            *MF->getSubtarget().getFrameLowering());
        if (!TFL.usesStaticStack(*MF))
          NeedsSoftStack = true;
      }

      SmallVector<CallEdge, 8> Calls;
      collectCalls(*F, MMI, Calls);
      for (const CallEdge &Call : Calls) {
        Function *Callee = Call.Callee;
        if (!Callee || Callee->isDeclaration()) {
          if (Callee && Callee->hasFnAttribute("interrupt-safe")) {
            if (Call.MI && usesImagRegs(*Call.MI))
              error(*F, "arguments/return of interrupt_safe '" + Call.Name +
                            "' use imaginary registers; only A/X/Y allowed");
            continue;
          }
          if (LibcallNames.contains(Call.Name))
            error(*F, "libcall '" + Call.Name + "' required by '" +
                          F->getName() +
                          "' has no bitcode definition to clone; compile with "
                          "-flto and link compiler-rt as bitcode");
          else
            error(*F, "call to external function '" + Call.Name +
                          "': it is not available as bitcode (defined in "
                          "assembly or a non-LTO object); mark it "
                          "interrupt_safe if it only uses A/X/Y");
          continue;
        }

        Attribute CalleeSfx = Callee->getFnAttribute("rc-suffix");
        if (!CalleeSfx.isValid() || CalleeSfx.getValueAsString() != Sfx) {
          error(*F, "'" + Callee->getName() +
                        "' uses the main register set but is called from '" +
                        F->getName() + "'");
          continue;
        }
        Worklist.push_back(Callee);
      }
    }

    if (NeedsSoftStack && !Root->hasFnAttribute("isr-init-soft-stack")) {
      LLVM_DEBUG(dbgs() << "Soft stack init needed for " << Root->getName()
                        << "\n");
      Root->addFnAttr("isr-init-soft-stack");
      Changed = true;
    }
  }
  return Changed;
}
