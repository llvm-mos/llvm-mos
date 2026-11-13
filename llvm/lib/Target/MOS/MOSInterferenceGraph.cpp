//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Implementation for interference graph management.
///
/// Simplification uses the notion of "squeeze". A vertex's candidates are the
/// allocation order of its register class, and its squeeze is the sum, over its
/// neighbors remaining in the graph, of the most of its candidates that one
/// candidate of the neighbor can overlap. Each removal lowers its neighbors'
/// squeeze, and a non-retained vertex can only be removed once its squeeze is
/// less than its number of candidates, since its neighbors then cannot block
/// all of them. Retained vertices are removed at the very end, since they are
/// assumed to be allocated by other means.
///
//===----------------------------------------------------------------------===//

#include "MOSInterferenceGraph.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include <functional>

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
    for (VertexID Other : Vertices[ID].Edges)
      Squeezes[ID] += Vertices[ID].maxBlockedLocations(Vertices[Other]);
    if (Squeezes[ID] < RCI.getOrder(Vertices[ID].RC).size())
      Worklist.push_back(ID);
  }

  // Candidate sets stay fixed while squeeze decreases. Enqueue each vertex
  // only when it first becomes removable; it stays removable until popped.
  while (!Worklist.empty()) {
    VertexID ID = Worklist.pop_back_val();
    Remaining.reset(ID);
    for (VertexID Other : Vertices[ID].Edges) {
      if (!Remaining.test(Other) || RetainedVertices.test(Other))
        continue;
      unsigned &Squeeze = Squeezes[Other];
      unsigned OldSqueeze = Squeeze;
      Squeeze -= Vertices[Other].maxBlockedLocations(Vertices[ID]);
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

MOSInterferenceGraph::VertexID MOSInterferenceGraph::Vertex::id() const {
  // Use std::less to avoid UB.
  assert(!std::less<>()(this, Graph->Vertices.begin()) &&
         std::less<>()(this, Graph->Vertices.end()) &&
         "vertex is not in its graph");
  return this - Graph->Vertices.begin();
}

unsigned
MOSInterferenceGraph::Vertex::maxBlockedLocations(const Vertex &Other) const {
  auto Key = std::make_pair(RC, Other.RC);
  auto It = Graph->MaxBlockedLocations.find(Key);
  if (It != Graph->MaxBlockedLocations.end())
    return It->second;

  unsigned Max = 0;
  for (MCPhysReg OtherPhys : Graph->RCI.getOrder(Key.second)) {
    unsigned Blocked =
        llvm::count_if(Graph->RCI.getOrder(Key.first), [&](MCPhysReg Phys) {
          return Graph->TRI.regsOverlap(Phys, OtherPhys);
        });
    Max = std::max(Max, Blocked);
  }
  Graph->MaxBlockedLocations.try_emplace(Key, Max);
  return Max;
}

void MOSInterferenceGraph::Vertex::addEdge(Vertex &Other) {
  assert(this != &Other && "a vertex cannot have an edge to itself");
  assert(Graph == Other.Graph && "vertices must be in the same graph");
  VertexID ID = id();
  VertexID OtherID = Other.id();
  assert(Graph->OccupiedVertices.test(ID) &&
         Graph->OccupiedVertices.test(OtherID));
  Edges.set(OtherID);
  Other.Edges.set(ID);
}

void MOSInterferenceGraph::retainAll() { RetainedVertices = OccupiedVertices; }

MOSInterferenceGraph::Vertex &
MOSInterferenceGraph::getOrCreateVertex(Register Reg) {
  assert(Reg.isVirtual() && "expected a virtual register");
  auto [It, Inserted] = VirtRegVertices.try_emplace(Reg);
  if (Inserted) {
    Vertex &V = createVertex(*TRI.getImagRegClass(Reg, MRI));
    V.Reg = Reg;
    It->second = V.id();
  }
  return Vertices[It->second];
}

MOSInterferenceGraph::Vertex &
MOSInterferenceGraph::createVertex(const TargetRegisterClass &RC) {
  VertexID ID;
  if (FreeVertices.empty()) {
    ID = Vertices.size();
    Vertices.emplace_back(*this);
    OccupiedVertices.resize(Vertices.size());
    RetainedVertices.resize(Vertices.size());
  } else {
    ID = FreeVertices.pop_back_val();
  }
  Vertices[ID].RC = &RC;
  OccupiedVertices.set(ID);
  return Vertices[ID];
}

void MOSInterferenceGraph::removeVertex(Vertex &V) {
  VertexID ID = V.id();
  assert(OccupiedVertices.test(ID) && "vertex already removed");
  for (VertexID Other : V.Edges)
    Vertices[Other].Edges.reset(ID);
  if (V.Reg)
    VirtRegVertices.erase(V.Reg);
  V = Vertex(*this);
  OccupiedVertices.reset(ID);
  RetainedVertices.reset(ID);
  FreeVertices.push_back(ID);
}

void MOSInterferenceGraphBuilder::stepForward(const MachineInstr &MI) {
  SlotIndex Idx = LIS.getInstructionIndex(MI);
  for (const MachineOperand &Use : MI.all_uses())
    assert((!Use.getReg().isVirtual() || LiveRegs.contains(Use.getReg())) &&
           "virtual inputs must already be live");
  for (const MachineOperand &Use : MI.all_uses())
    if (Use.getReg().isVirtual() &&
        LIS.getInterval(Use.getReg()).Query(Idx).isKill())
      LiveRegs.erase(Use.getReg());
  for (const MachineOperand &Def : MI.all_defs())
    if (Def.getReg().isVirtual())
      defineReg(Def.getReg());
  for (const MachineOperand &Def : MI.all_defs())
    if (Def.getReg().isVirtual() &&
        LIS.getInterval(Def.getReg()).Query(Idx).isDeadDef())
      LiveRegs.erase(Def.getReg());
}

void MOSInterferenceGraphBuilder::beginCapture(MOSInterferenceGraph &Graph) {
  assert(!this->Graph && "a capture is already active");
  this->Graph = &Graph;
  SmallVector<VertexID> Earlier;
  for (Register Reg : LiveRegs) {
    Vertex &V = Graph.getOrCreateVertex(Reg);
    for (VertexID Other : Earlier)
      V.addEdge(Graph.Vertices[Other]);
    VertexID ID = V.id();
    Earlier.push_back(ID);
    Graph.RetainedVertices.set(ID);
  }
}

void MOSInterferenceGraphBuilder::addLiveReg(Register Reg) {
  assert(Graph && "adjusting liveness requires an active capture");
  defineReg(Reg);
}

void MOSInterferenceGraphBuilder::removeLiveReg(Register Reg) {
  assert(Graph && "adjusting liveness requires an active capture");
  assert(Reg.isVirtual() && LiveRegs.contains(Reg) &&
         "expected a live virtual register");
  LiveRegs.erase(Reg);
  Graph->removeVertex(Graph->Vertices[Graph->VirtRegVertices.at(Reg)]);
}

MOSInterferenceGraph::VertexID
MOSInterferenceGraphBuilder::addTemporaryVertex(const TargetRegisterClass &RC) {
  assert(Graph && "temporary storage requires an active capture");
  Vertex &Temporary = Graph->createVertex(RC);
  defineVertex(Temporary);
  return Temporary.id();
}

void MOSInterferenceGraphBuilder::killTemporaryVertex(VertexID ID) {
  assert(Graph && "temporary storage requires an active capture");
  auto It = llvm::find(LiveTemporaries, ID);
  assert(It != LiveTemporaries.end() && "temporary is not live");
  LiveTemporaries.erase(It);
  Graph->removeVertex(Graph->Vertices[ID]);
}

void MOSInterferenceGraphBuilder::trimToLive() {
  assert(Graph && "trimming requires an active capture");
  SmallBitVector DeadVertices = Graph->OccupiedVertices;
  for (Register Reg : LiveRegs)
    DeadVertices.reset(Graph->VirtRegVertices.at(Reg));
  for (VertexID ID : LiveTemporaries)
    DeadVertices.reset(ID);
  for (VertexID ID : DeadVertices.set_bits())
    Graph->removeVertex(Graph->Vertices[ID]);
}

void MOSInterferenceGraphBuilder::retainAll() {
  assert(Graph && "retention requires an active capture");
  Graph->retainAll();
}

void MOSInterferenceGraphBuilder::endCapture() {
  assert(Graph && "no capture is active");
  assert(LiveTemporaries.empty() && "cannot end capture with live temporaries");
  Graph = nullptr;
}

void MOSInterferenceGraphBuilder::defineReg(Register Reg) {
  assert(Reg.isVirtual() && !LiveRegs.contains(Reg) &&
         "expected a virtual register that is not yet live");
  if (Graph)
    defineVertex(Graph->getOrCreateVertex(Reg));
  else
    LiveRegs.insert(Reg);
}

void MOSInterferenceGraphBuilder::defineVertex(Vertex &V) {
  // Live registers already have vertices; creating one could invalidate V.
  for (Register Reg : LiveRegs)
    V.addEdge(Graph->Vertices[Graph->VirtRegVertices.at(Reg)]);
  for (VertexID Temporary : LiveTemporaries)
    V.addEdge(Graph->Vertices[Temporary]);
  if (V.Reg)
    LiveRegs.insert(V.Reg);
  else
    LiveTemporaries.push_back(V.id());
}
