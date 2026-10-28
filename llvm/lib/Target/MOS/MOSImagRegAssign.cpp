//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Assign imaginary registers after local spilling.
///
/// MOSImagRegSpill leaves an SSA program whose imaginary register demands can
/// be satisfied. This pass chooses assignments respecting interference and
/// records them in VirtRegMap. It does not insert further spills.
///
/// This pass handles the MIR that MOSImagRegSpill accepts, along with the
/// transfer code that pass inserts: early-clobber scratch definitions,
/// REG_SEQUENCE, and subregister uses. Instructions are assigned in order using
/// local interference graphs.
///
//===----------------------------------------------------------------------===//

#include "MOSImagRegAssign.h"
#include "MOS.h"
#include "MOSInterferenceGraph.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveRegUnits.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/MCRegister.h"
#include "llvm/Support/ErrorHandling.h"
#include <optional>

#define DEBUG_TYPE "mos-imag-reg-assign"

using namespace llvm;

namespace {

class MOSImagRegAssign : public MachineFunctionPass {
public:
  static char ID;

  MOSImagRegAssign() : MachineFunctionPass(ID) {
    initializeMOSImagRegAssignPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LiveIntervalsWrapperPass>();
    AU.addRequired<MachineRegisterClassInfoWrapperPass>();
    AU.addRequired<VirtRegMapWrapperLegacy>();
    AU.addPreserved<LiveIntervalsWrapperPass>();
    AU.addPreserved<SlotIndexesWrapperPass>();
    AU.addPreserved<VirtRegMapWrapperLegacy>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

private:
  // Assign the registers defined by a basic block.
  void assignMBB(MachineBasicBlock &MBB);

  // Assign the registers defined by a nonempty, contiguous range. The builder
  // must be capturing Graph at the range's beginning.
  void assignMIs(iterator_range<MachineBasicBlock::iterator> Instructions);

  // Assign the registers mentioned in the current interference graph.
  void assignGraph();

  VirtRegMap *VRM = nullptr;
  LiveIntervals *LIS = nullptr;
  const RegisterClassInfo *RCI = nullptr;
  std::optional<MOSInterferenceGraphBuilder> Builder;
  std::optional<MOSInterferenceGraph> Graph;
};

} // namespace

bool MOSImagRegAssign::runOnMachineFunction(MachineFunction &MF) {
  VRM = &getAnalysis<VirtRegMapWrapperLegacy>().getVRM();
  LIS = &getAnalysis<LiveIntervalsWrapperPass>().getLIS();
  RCI = &getAnalysis<MachineRegisterClassInfoWrapperPass>().getRCI();
  Builder.emplace(*LIS);

  assignMBB(MF.front());

  Builder.reset();
  return false;
}

void MOSImagRegAssign::assignMBB(MachineBasicBlock &MBB) {
  Graph.emplace(*MBB.getParent(), *RCI, VRM);
  Builder->beginCapture(*Graph);
  for (auto I = MBB.begin(), E = MBB.end(); I != E;) {
    if (I->isDebugOrPseudoInstr()) {
      ++I;
      continue;
    }
    auto End = std::next(I);
    if (I->isTerminator()) {
      // Terminators remain contiguous and receive assignments together.
      End = E;
    }
    assignMIs(make_range(I, End));
    I = End;
  }
  Builder->endCapture();
  Graph.reset();
}

void MOSImagRegAssign::assignMIs(
    iterator_range<MachineBasicBlock::iterator> Instructions) {
  for (MachineInstr &MI : Instructions)
    if (!MI.isDebugOrPseudoInstr())
      Builder->stepForward(MI);
  assignGraph();
  Builder->trimToLive();
}

void MOSImagRegAssign::assignGraph() {
  SmallVector<MOSInterferenceGraph::VertexID> AssignmentOrder;
  if (!Graph->simplify(nullptr, &AssignmentOrder))
    report_fatal_error("MOS guaranteed imaginary assignment failed", false);
  for (MOSInterferenceGraph::VertexID ID : AssignmentOrder) {
    const MOSInterferenceGraph::Vertex &V = Graph->vertex(ID);
    ArrayRef<MCPhysReg> Candidates = V.candidates();
    // Choose a location that does not overlap with neighbors assigned so far.
    LiveRegUnits BlockedUnits = V.neighborAssignedUnits();
    auto Phys = llvm::find_if(Candidates, [&](MCPhysReg Phys) {
      return BlockedUnits.available(Phys);
    });
    if (Phys == Candidates.end())
      report_fatal_error("MOS guaranteed imaginary assignment failed", false);
    VRM->assignVirt2Phys(V.Reg, *Phys);
  }
}

char MOSImagRegAssign::ID = 0;
INITIALIZE_PASS_BEGIN(MOSImagRegAssign, DEBUG_TYPE,
                      "MOS Imaginary Register Assignment", false, false)
INITIALIZE_PASS_DEPENDENCY(LiveIntervalsWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MachineRegisterClassInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(VirtRegMapWrapperLegacy)
INITIALIZE_PASS_END(MOSImagRegAssign, DEBUG_TYPE,
                    "MOS Imaginary Register Assignment", false, false)

MachineFunctionPass *llvm::createMOSImagRegAssignPass() {
  return new MOSImagRegAssign;
}
