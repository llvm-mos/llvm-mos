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
/// whose candidate count exceeds that bound and updates their neighbors.
/// Earlier definitions remain as blockers throughout. Anonymous temporary
/// vertices undergo the same simplification as virtual registers. Blocking
/// bounds are cached by register-class pair across graph edits.
///
//===----------------------------------------------------------------------===//

#include "MOSInterferenceGraph.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/STLExtras.h"
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
    if (RetainedVertices.test(ID))
      continue;
    for (VertexID Other : Vertices[ID].Interferences)
      Squeezes[ID] += maxBlockedLocations(ID, Other);
    if (Squeezes[ID] < RCI.getOrder(Vertices[ID].RC).size())
      Worklist.push_back(ID);
  }

  // Candidate sets stay fixed while squeeze decreases. Enqueue each vertex
  // only when it first becomes removable; it stays removable until popped.
  while (!Worklist.empty()) {
    VertexID ID = Worklist.pop_back_val();
    Remaining.reset(ID);
    for (VertexID Other : Vertices[ID].Interferences) {
      if (!Remaining.test(Other) || RetainedVertices.test(Other))
        continue;
      unsigned &Squeeze = Squeezes[Other];
      unsigned OldSqueeze = Squeeze;
      Squeeze -= maxBlockedLocations(Other, ID);
      unsigned Candidates = RCI.getOrder(Vertices[Other].RC).size();
      if (OldSqueeze >= Candidates && Squeeze < Candidates)
        Worklist.push_back(Other);
    }
  }
  bool Simplified = Remaining == RetainedVertices;
  if (RemainingRegs) {
    RemainingRegs->clear();
    if (!Simplified) {
      for (VertexID ID : Remaining.set_bits()) {
        if (Register Reg = Vertices[ID].Reg)
          RemainingRegs->push_back(Reg);
      }
    }
  }
  return Simplified;
}

void MOSInterferenceGraph::retainAll() {
  RetainedVertices = OccupiedVertices;
}

void MOSInterferenceGraph::remove(Register Reg) {
  assert(Reg.isVirtual() && "expected a virtual register");
  auto It = VirtRegVertices.find(Reg);
  if (It != VirtRegVertices.end())
    removeVertex(It->second);
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
  RetainedVertices.reset(ID);
  FreeVertices.push_back(ID);
}

unsigned MOSInterferenceGraph::maxBlockedLocations(VertexID ID,
                                                   VertexID Other) const {
  auto Key = std::make_pair(Vertices[ID].RC, Vertices[Other].RC);
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
  if (Inserted) {
    It->second = createVertex(*TRI.getImagRegClass(Reg, MRI));
    Vertices[It->second].Reg = Reg;
  }
  return It->second;
}

MOSInterferenceGraph::VertexID
MOSInterferenceGraph::createVertex(const TargetRegisterClass &RC) {
  VertexID ID;
  if (FreeVertices.empty()) {
    ID = Vertices.size();
    Vertices.emplace_back();
    OccupiedVertices.resize(Vertices.size());
    RetainedVertices.resize(Vertices.size());
  } else {
    ID = FreeVertices.pop_back_val();
  }
  Vertices[ID].RC = &RC;
  OccupiedVertices.set(ID);
  return ID;
}

void MOSInterferenceGraph::addInterference(VertexID ID, VertexID Other) {
  assert(OccupiedVertices.test(ID) && OccupiedVertices.test(Other));
  if (ID == Other)
    return;
  Vertices[ID].Interferences.set(Other);
  Vertices[Other].Interferences.set(ID);
}

void MOSInterferenceGraph::addInterferences(
    VertexID ID, const SparseBitVector<> &LiveVertices) {
  for (VertexID Other : LiveVertices)
    addInterference(ID, Other);
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
  for (VertexID ID : LiveVertices) {
    Graph.addInterferences(ID, LiveVertices);
    Graph.RetainedVertices.set(ID);
  }
}

MOSInterferenceGraph::VertexID
MOSInterferenceGraphBuilder::addTemporaryVertex(const TargetRegisterClass &RC) {
  assert(Graph && "temporary storage requires an active capture");
  VertexID Temporary = Graph->createVertex(RC);
  addInterferences(Temporary);
  LiveTemporaries.push_back(Temporary);
  return Temporary;
}

void MOSInterferenceGraphBuilder::killTemporaryVertex(VertexID ID) {
  assert(Graph && "temporary storage requires an active capture");
  auto It = llvm::find(LiveTemporaries, ID);
  assert(It != LiveTemporaries.end() && "temporary is not live");
  LiveTemporaries.erase(It);
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
  for (VertexID ID : LiveTemporaries)
    DeadVertices.reset(ID);
  for (VertexID ID : DeadVertices.set_bits())
    Graph->removeVertex(ID);
}

void MOSInterferenceGraphBuilder::endCapture() {
  assert(Graph && "no capture is active");
  assert(LiveTemporaries.empty() && "cannot end capture with live temporaries");
  Graph = nullptr;
}

void MOSInterferenceGraphBuilder::addInterferences(VertexID ID) {
  SparseBitVector<> LiveVertices;
  for (Register Reg : State.ResidentValues)
    LiveVertices.set(Graph->getOrCreateVertex(Reg));
  for (VertexID Temporary : LiveTemporaries)
    LiveVertices.set(Temporary);
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
