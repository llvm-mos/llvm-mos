//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Interference graph construction and squeeze-based simplification.
///
/// Vertices use reusable indices and sparse adjacency bitsets. The builder
/// connects each new resident to the current residents. Within an instruction,
/// killed uses are released before definitions are introduced, and dead results
/// are released afterwards. Edges persist until their vertices are removed.
///
/// Simplification sums the maximum number of candidate locations each neighbor
/// could block, accounting for register aliasing. A worklist removes vertices
/// whose candidate count exceeds that bound and updates their neighbors. The
/// blocking bounds are cached by register-class pair across graph edits.
///
//===----------------------------------------------------------------------===//

#include "MOSInterferenceGraph.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterClassInfo.h"

using namespace llvm;

MOSInterferenceGraph::MOSInterferenceGraph(MachineFunction &MF,
                                           const RegisterClassInfo &RCI)
    : TRI(*MF.getSubtarget<MOSSubtarget>().getRegisterInfo()),
      MRI(MF.getRegInfo()), RCI(RCI) {}

bool MOSInterferenceGraph::canSimplify(
    SmallVectorImpl<Register> *RemainingRegs) const {
  // Squeeze bounds how many candidate locations the remaining neighbors can
  // block. Removing a vertex with more candidates than this bound guarantees
  // that it can be assigned after all of those neighbors have been assigned.
  SmallBitVector Remaining = OccupiedVertices;
  SmallVector<unsigned> Squeezes(Vertices.size(), 0);
  SmallVector<VertexID> Worklist;
  for (VertexID ID : OccupiedVertices.set_bits()) {
    for (VertexID Other : Vertices[ID].Interferences)
      Squeezes[ID] += maxBlockedLocations(ID, Other);
    if (Squeezes[ID] < RCI.getOrder(regClass(ID)).size())
      Worklist.push_back(ID);
  }

  // Candidate sets stay fixed while squeeze decreases. Enqueue each vertex
  // only when it first becomes removable; it stays removable until popped.
  while (!Worklist.empty()) {
    VertexID ID = Worklist.pop_back_val();
    Remaining.reset(ID);
    for (VertexID Other : Vertices[ID].Interferences) {
      if (!Remaining.test(Other))
        continue;
      unsigned &Squeeze = Squeezes[Other];
      unsigned OldSqueeze = Squeeze;
      Squeeze -= maxBlockedLocations(Other, ID);
      unsigned Candidates = RCI.getOrder(regClass(Other)).size();
      if (OldSqueeze >= Candidates && Squeeze < Candidates)
        Worklist.push_back(Other);
    }
  }
  if (RemainingRegs) {
    RemainingRegs->clear();
    for (VertexID ID : Remaining.set_bits())
      if (Register Reg = Vertices[ID].Reg)
        RemainingRegs->push_back(Reg);
  }
  return Remaining.none();
}

void MOSInterferenceGraph::remove(Register Reg) {
  assert(Reg.isVirtual() && "expected a virtual register");
  auto It = VirtRegVertices.find(Reg);
  if (It != VirtRegVertices.end())
    removeVertex(It->second);
}

const TargetRegisterClass *MOSInterferenceGraph::regClass(VertexID ID) const {
  const Vertex &V = Vertices[ID];
  return V.ScratchRC ? V.ScratchRC : TRI.getImagRegClass(V.Reg, MRI);
}

unsigned MOSInterferenceGraph::maxBlockedLocations(VertexID ID,
                                                   VertexID Other) const {
  auto Key = std::make_pair(regClass(ID), regClass(Other));
  auto It = MaxBlockedLocations.find(Key);
  if (It != MaxBlockedLocations.end())
    return It->second;

  unsigned Max = 0;
  for (MCPhysReg OtherPhys : RCI.getOrder(Key.second)) {
    unsigned Blocked =
        llvm::count_if(RCI.getOrder(Key.first), [&](MCPhysReg Phys) {
          return TRI.regsOverlap(Phys, OtherPhys);
        });
    Max = std::max(Max, Blocked);
  }
  MaxBlockedLocations.try_emplace(Key, Max);
  return Max;
}

MOSInterferenceGraph::VertexID
MOSInterferenceGraph::getOrCreateVertex(Register Reg) {
  assert(Reg.isVirtual() && "expected a virtual register");
  auto [It, Inserted] = VirtRegVertices.try_emplace(Reg);
  if (Inserted)
    It->second = createVertex(Reg);
  return It->second;
}

MOSInterferenceGraph::VertexID
MOSInterferenceGraph::createVertex(Register Reg) {
  VertexID ID;
  if (FreeVertices.empty()) {
    ID = Vertices.size();
    Vertices.push_back({Reg, nullptr, {}});
    OccupiedVertices.resize(Vertices.size());
  } else {
    ID = FreeVertices.pop_back_val();
    Vertices[ID].Reg = Reg;
  }
  OccupiedVertices.set(ID);
  return ID;
}

MOSInterferenceGraph::VertexID
MOSInterferenceGraph::createScratchVertex(const TargetRegisterClass &RC) {
  VertexID ID = createVertex(Register());
  Vertices[ID].ScratchRC = &RC;
  return ID;
}

void MOSInterferenceGraph::removeVertex(VertexID ID) {
  assert(OccupiedVertices.test(ID) && "vertex already removed");
  Vertex &V = Vertices[ID];
  for (VertexID Other : V.Interferences)
    Vertices[Other].Interferences.reset(ID);
  if (V.Reg)
    VirtRegVertices.erase(V.Reg);
  V = {};
  OccupiedVertices.reset(ID);
  FreeVertices.push_back(ID);
}

void MOSInterferenceGraph::addInterferences(
    VertexID ID, const SparseBitVector<> &LiveVertices) {
  for (VertexID Other : LiveVertices) {
    if (ID == Other)
      continue;
    Vertices[ID].Interferences.set(Other);
    Vertices[Other].Interferences.set(ID);
  }
}

SmallVector<Register>
MOSInterferenceGraphBuilder::nonResidentUses(const MachineInstr &MI) const {
  SmallSetVector<Register, 8> Regs;
  for (const MachineOperand &Use : MI.all_uses()) {
    Register Reg = Use.getReg();
    if (Reg.isVirtual() && !Use.isDebug() && !Use.isUndef() &&
        !State.ResidentValues.contains(Reg))
      Regs.insert(Reg);
  }
  return SmallVector<Register>(Regs.begin(), Regs.end());
}

SmallVector<Register>
MOSInterferenceGraphBuilder::spillCandidates(const MachineInstr &MI) const {
  SmallVector<Register> Regs;
  for (Register Reg : State.ResidentValues)
    if (!MI.readsRegister(Reg, nullptr) && !MI.definesRegister(Reg, nullptr))
      Regs.push_back(Reg);
  return Regs;
}

void MOSInterferenceGraphBuilder::evict(Register Reg) {
  assert(Reg.isVirtual() && State.ResidentValues.contains(Reg) &&
         "expected a resident virtual register");
  releaseReg(Reg);
}

void MOSInterferenceGraphBuilder::reload(Register Reg) {
  assert(Reg.isVirtual() && !State.ResidentValues.contains(Reg) &&
         "expected a nonresident virtual register");
  makeLive(Reg);
  State.SavedResidentValues.insert(Reg);
}

void MOSInterferenceGraphBuilder::beginCapture(MOSInterferenceGraph &Graph) {
  assert(!this->Graph && "a capture is already active");
  this->Graph = &Graph;
  SparseBitVector<> LiveVertices;
  for (Register Reg : State.ResidentValues)
    LiveVertices.set(Graph.getOrCreateVertex(Reg));
  for (VertexID ID : LiveVertices)
    Graph.addInterferences(ID, LiveVertices);
}

void MOSInterferenceGraphBuilder::reserveHeadroom(
    const TargetRegisterClass &RC) {
  assert(Graph && "headroom requires an active capture");
  // Scratch is needed here only; it does not acquire interference with later
  // definitions when the builder steps forward.
  addInterferences(Graph->createScratchVertex(RC));
}

void MOSInterferenceGraphBuilder::stepForward(const MachineInstr &MI) {
  for (const MachineOperand &Use : MI.all_uses())
    assert((!Use.getReg().isVirtual() ||
            State.ResidentValues.contains(Use.getReg())) &&
           "virtual inputs must already be resident");
  for (const MachineOperand &Use : MI.all_uses())
    if (Use.getReg().isVirtual() && Use.isKill())
      releaseReg(Use.getReg());
  for (const MachineOperand &Def : MI.all_defs())
    if (Def.getReg().isVirtual())
      makeLive(Def.getReg());
  for (const MachineOperand &Def : MI.all_defs())
    if (Def.getReg().isVirtual() && Def.isDead())
      releaseReg(Def.getReg());
}

void MOSInterferenceGraphBuilder::trimToLive() {
  assert(Graph && "trimming requires an active capture");
  SmallBitVector DeadVertices = Graph->OccupiedVertices;
  for (Register Reg : State.ResidentValues)
    DeadVertices.reset(Graph->VirtRegVertices.at(Reg));
  for (VertexID ID : DeadVertices.set_bits())
    Graph->removeVertex(ID);
}

void MOSInterferenceGraphBuilder::endCapture() {
  assert(Graph && "no capture is active");
  Graph = nullptr;
}

void MOSInterferenceGraphBuilder::addInterferences(VertexID ID) {
  SparseBitVector<> LiveVertices;
  for (Register Reg : State.ResidentValues)
    LiveVertices.set(Graph->getOrCreateVertex(Reg));
  Graph->addInterferences(ID, LiveVertices);
}

void MOSInterferenceGraphBuilder::makeLive(Register Reg) {
  assert(Reg.isVirtual() && !State.ResidentValues.contains(Reg) &&
         "expected a new resident virtual register");
  if (Graph)
    addInterferences(Graph->getOrCreateVertex(Reg));
  State.ResidentValues.insert(Reg);
}

void MOSInterferenceGraphBuilder::releaseReg(Register Reg) {
  State.ResidentValues.erase(Reg);
  State.SavedResidentValues.erase(Reg);
}
