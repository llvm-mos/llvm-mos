//===-- MOSCallTreeClone.cpp - MOS Call Tree Cloning Pass -----------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS call tree cloning pass.
//
// A function carrying the "interrupt-rc-suffix"="<suffix>" attribute (set by
// the clang front-end for the MOS interrupt_norecurse("suffix") attribute) is
// the root of an interrupt service routine that uses a private
// imaginary-register set: every emitted __rcN symbol is suffixed with <suffix>.
// Each suffix may have only one root, and the root must be norecurse.
//
// Every function the ISR reaches must also use the private register set,
// otherwise it would clobber the interrupted code's registers. Functions shared
// between mainline code and the ISR tree therefore need two compiled copies.
// This pass produces the per-suffix copies by cloning the reachable call tree
// and every defined runtime libcall. The clones are stamped with the
// backend-facing attribute "rc-suffix"="<suffix>" that the rest of the back-end
// uses to rename imaginary-register symbols.
//
// Libcall clones keep partition "contingent" so that MOSInternalize can DCE
// any that turn out to be unused (zero final code-size cost for pre-cloning).
// They are marked "mos-call-tree-libcall-clone". A later small
// MachineFunctionPass (MOSCallTreeLibcalls) rewrites the external-symbol
// operands the legalizer emits for libcalls in suffixed functions to point at
// the suffixed clones.
//
// The walk has two phases. Phase A starts at the roots; anything it finds
// wrong is definitely reachable from the ISR and is diagnosed immediately.
// Phase B starts at the speculative libcall clones, so the helpers they call
// are cloned too. Problems found only in phase B may be in code that is never
// used, so they are recorded as "mos-call-tree-error" on the offending clone
// and reported by MOSCallTreeVerify only if the clone is actually reachable.
//
//===----------------------------------------------------------------------===//

#include "MOSCallTreeClone.h"

#include "MOS.h"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/ADT/Twine.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/IR/Module.h"
#include "llvm/LTO/LTO.h"
#include "llvm/Support/Debug.h"
#include "llvm/Transforms/Utils/Cloning.h"

#define DEBUG_TYPE "mos-call-tree-clone"

using namespace llvm;

namespace {

class MOSCallTreeClone : public ModulePass {
public:
  static char ID;

  MOSCallTreeClone() : ModulePass(ID) {
    initializeMOSCallTreeClonePass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
};

} // namespace

char MOSCallTreeClone::ID = 0;

INITIALIZE_PASS(MOSCallTreeClone, DEBUG_TYPE,
                "Clone call trees of suffixed MOS ISRs for their private "
                "register sets",
                /* CFGOnly = */ false, /* Analysis = */ false)

ModulePass *llvm::createMOSCallTreeClonePass() {
  return new MOSCallTreeClone();
}

PreservedAnalyses MOSCallTreeClonePass::run(Module &M,
                                            ModuleAnalysisManager &AM) {
  MOSCallTreeClone Pass;
  bool Changed = Pass.runOnModule(M);
  return Changed ? PreservedAnalyses::none() : PreservedAnalyses::all();
}

// Reports an unsupported-feature diagnostic against the function F. The
// message is prefixed with "[MOS interrupt] " so users can find the cause.
static void reportUnsupported(Function &F, const Twine &Msg) {
  F.getContext().diagnose(DiagnosticInfoUnsupported(
      F, "[MOS interrupt] " + Msg + " (function \"" + F.getName() + "\")"));
}

// Returns whether an inline asm string contains a jsr or jmp instruction.
static bool asmHasCallOrJump(StringRef Asm) {
  std::string Lower = Asm.lower();
  return StringRef(Lower).contains("jsr") || StringRef(Lower).contains("jmp");
}

bool MOSCallTreeClone::runOnModule(Module &M) {
  // Collect roots grouped by suffix, preserving discovery order so diagnostics
  // are deterministic.
  SmallVector<std::string, 4> SuffixOrder;
  StringMap<SmallVector<Function *, 4>> RootsBySuffix;
  for (Function &F : M) {
    Attribute SuffixAttr = F.getFnAttribute("interrupt-rc-suffix");
    if (!SuffixAttr.isValid())
      continue;
    // A reentrant handler could clobber its own register set when re-entered,
    // and would need its private soft stack initialized before the first
    // interrupt. The front-end rejects this; catch hand-written IR too.
    if (F.hasFnAttribute("interrupt")) {
      reportUnsupported(F, "a private register set requires "
                           "\"interrupt-norecurse\"; plain \"interrupt\" "
                           "handlers cannot have a register set suffix");
      continue;
    }
    StringRef Sfx = SuffixAttr.getValueAsString();
    auto ItInserted = RootsBySuffix.try_emplace(Sfx);
    if (ItInserted.second)
      SuffixOrder.emplace_back(Sfx);
    ItInserted.first->second.push_back(&F);
  }

  if (RootsBySuffix.empty())
    return false;

  // Set of known runtime libcall names; the legalizer may emit calls to these
  // even though they don't appear in the IR. We pre-clone any with a definition
  // so they're available when the legalizer needs them.
  StringSet<> LibcallNameSet;
  for (const char *Name :
       lto::LTO::getRuntimeLibcallSymbols(Triple(M.getTargetTriple())))
    LibcallNameSet.insert(Name);

  bool Changed = false;
  LLVM_DEBUG(dbgs() << "**** MOS Call Tree Clone Pass ****\n");

  for (const std::string &SfxStorage : SuffixOrder) {
    StringRef Sfx(SfxStorage);
    LLVM_DEBUG(dbgs() << "  Suffix: '" << Sfx << "'\n");

    auto &Roots = RootsBySuffix[Sfx];
    // Two roots sharing a register set could preempt one another (e.g. NMI
    // during IRQ) and clobber each other's registers and static stack frames.
    if (Roots.size() > 1) {
      reportUnsupported(*Roots[1], "register set suffix '" + Twine(Sfx) +
                                       "' is used by both \"" +
                                       Roots[0]->getName() + "\" and \"" +
                                       Roots[1]->getName() +
                                       "\"; give each handler a distinct "
                                       "suffix");
      continue;
    }
    Function *Root = Roots.front();

    // Stamp the backend-facing "rc-suffix" attribute on the root. Roots keep
    // their original symbol names since interrupt vectors reference them.
    Root->addFnAttr("rc-suffix", Sfx);
    Changed = true;

    // Memoization: original function -> per-suffix clone.
    DenseMap<Function *, Function *> Clones;

    // Clone a function for this suffix if not already cloned. Returns the
    // clone (and records it in Clones) or nullptr on collision.
    auto cloneFn = [&](Function *Orig) -> Function * {
      auto It = Clones.find(Orig);
      if (It != Clones.end())
        return It->second;

      std::string CloneName = (Orig->getName() + Sfx).str();
      if (M.getFunction(CloneName)) {
        reportUnsupported(*Orig, "clone name '" + Twine(CloneName) +
                                     "' already exists in module; suffix '" +
                                     Sfx +
                                     "' collides with an existing symbol");
        return nullptr;
      }

      ValueToValueMapTy VMap;
      Function *Clone = CloneFunction(Orig, VMap);
      Clone->setName(CloneName);
      Clone->setLinkage(GlobalValue::InternalLinkage);
      Clone->addFnAttr("rc-suffix", Sfx);
      // A clone is an ordinary function in the tree, never a handler itself.
      Clone->removeFnAttr("interrupt");
      Clone->removeFnAttr("interrupt-norecurse");
      Clone->removeFnAttr("interrupt-rc-suffix");
      // Clones are private to this suffix; comdat would let the linker merge
      // them back with the original, defeating the purpose.
      if (Clone->hasComdat())
        Clone->setComdat(nullptr);

      Clones[Orig] = Clone;
      Changed = true;
      return Clone;
    };

    // Pre-clone defined libcalls, preserving partition "contingent" so that
    // MOSInternalize can DCE the unused ones post-legalization. These clones
    // are speculative: they exist only so MOSCallTreeLibcalls can redirect
    // legalizer-created libcall references in suffixed functions to them.
    SmallVector<Function *, 16> LibcallDefs;
    for (Function &F : M) {
      if (F.isDeclaration())
        continue;
      if (!LibcallNameSet.contains(F.getName()))
        continue;
      LibcallDefs.push_back(&F);
    }
    SmallVector<Function *, 16> LibcallClones;
    for (Function *F : LibcallDefs) {
      if (Function *Clone = cloneFn(F)) {
        Clone->setPartition("contingent");
        Clone->addFnAttr("mos-call-tree-libcall-clone");
        LibcallClones.push_back(Clone);
      }
    }

    SmallPtrSet<Function *, 32> Visited;
    SmallVector<Function *, 32> VisitOrder;
    SmallVector<Function *, 32> Worklist;

    // Rewrites the direct calls in every function on the worklist (and
    // transitively in their callees) to per-suffix clones. When Speculative,
    // errors are recorded on the offending function instead of reported.
    auto walk = [&](bool Speculative) {
      while (!Worklist.empty()) {
        Function *F = Worklist.pop_back_val();
        if (!Visited.insert(F).second)
          continue;
        VisitOrder.push_back(F);

        auto error = [&](const Twine &Msg) {
          if (!Speculative) {
            reportUnsupported(*F, Msg);
            // Keep MOSCallTreeVerify from reporting this function again.
            F->addFnAttr("mos-call-tree-reported");
            return;
          }
          if (!F->hasFnAttribute("mos-call-tree-error"))
            F->addFnAttr("mos-call-tree-error",
                         ("[MOS interrupt] " + Msg + " (function \"" +
                          F->getName() + "\")")
                             .str());
        };

        for (BasicBlock &BB : *F) {
          for (Instruction &I : BB) {
            auto *CB = dyn_cast<CallBase>(&I);
            if (!CB)
              continue;

            // Inline asm is handled by the dedicated warning scan below; it is
            // not a function-pointer call.
            if (CB->isInlineAsm())
              continue;

            // Look through aliases and casts: the target is still known.
            auto *Callee = dyn_cast<Function>(
                CB->getCalledOperand()->stripPointerCastsAndAliases());
            if (!Callee) {
              error("indirect call (function pointer) in the call tree of "
                    "interrupt handler \"" +
                    Root->getName() + "\" is not supported");
              continue;
            }

            // Intrinsics (memcpy intrinsic, etc.) are not subject to
            // register renaming.
            if (Callee->isIntrinsic())
              continue;

            if (Callee->isDeclaration()) {
              // Asserted by the user to touch only A/X/Y. Such a routine
              // can't call back into C code, which would use imaginary
              // registers; saying so keeps MOSNonReentrant from assuming the
              // ISR may recurse through it.
              if (Callee->hasFnAttribute("interrupt-safe")) {
                Callee->addFnAttr(Attribute::NoCallback);
                continue;
              }
              // Runtime libcalls without bitcode are reported by
              // MOSCallTreeVerify, but only if actually reachable.
              if (LibcallNameSet.contains(Callee->getName()))
                continue;
              error("call to external function '" + Twine(Callee->getName()) +
                    "' in the call tree of interrupt handler \"" +
                    Root->getName() +
                    "\": it is not available as bitcode (defined in assembly "
                    "or a non-LTO object); mark it interrupt_safe if it only "
                    "uses A/X/Y");
              continue;
            }

            // Already a same-suffix function (the root, or a clone we just
            // made). Only an alias needs to be looked through.
            Attribute CalleeSfx = Callee->getFnAttribute("rc-suffix");
            if (CalleeSfx.isValid() && CalleeSfx.getValueAsString() == Sfx) {
              if (CB->getCalledOperand() != Callee)
                CB->setCalledOperand(Callee);
              continue;
            }

            // Otherwise we need a per-suffix clone of this callee.
            Function *Clone = cloneFn(Callee);
            if (!Clone)
              continue;
            CB->setCalledOperand(Clone);
            Worklist.push_back(Clone);
          }
        }
      }
    };

    // Phase A: everything reachable from the root.
    Worklist.push_back(Root);
    walk(/*Speculative=*/false);
    SmallVector<Function *, 32> DefinitelyReachable = VisitOrder;

    // Phase B: helpers called by the speculative libcall clones.
    for (Function *Clone : LibcallClones)
      Worklist.push_back(Clone);
    walk(/*Speculative=*/true);

    // Warning: scan the inline asm in the tree. The asm text isn't rewritten,
    // so references to imaginary registers (__rc...) would silently point at
    // the main register set, and textual jsr/jmp targets are not redirected to
    // their per-suffix clones.
    for (Function *F : DefinitelyReachable) {
      for (BasicBlock &BB : *F) {
        for (Instruction &I : BB) {
          auto *CB = dyn_cast<CallBase>(&I);
          if (!CB || !CB->isInlineAsm())
            continue;
          StringRef Asm =
              cast<InlineAsm>(CB->getCalledOperand())->getAsmString();
          if (Asm.contains("__rc"))
            F->getContext().diagnose(DiagnosticInfoInlineAsm(
                I,
                "[MOS interrupt] inline assembly in the call tree of "
                "interrupt handler \"" +
                    Root->getName() +
                    "\" references imaginary-register symbols (__rc...); "
                    "these textual references are not rewritten to use the "
                    "handler's suffixed symbols",
                DS_Warning));
          if (asmHasCallOrJump(Asm))
            F->getContext().diagnose(DiagnosticInfoInlineAsm(
                I,
                "[MOS interrupt] inline assembly in the call tree of "
                "interrupt handler \"" +
                    Root->getName() +
                    "\" contains jsr/jmp; textual call targets are not "
                    "redirected to the handler's per-suffix clones",
                DS_Warning));
        }
      }
    }
  }

  return Changed;
}
