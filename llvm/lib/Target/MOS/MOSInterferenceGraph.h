//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Interference graphs and instruction walks for imaginary register spilling.
///
/// MOSInterferenceGraph records storage conflicts and checks whether imaginary
/// registers can be assigned without spilling. A successful check guarantees
/// an assignment; failure is conservative. It does not select assignments.
///
/// MOSInterferenceGraphBuilder tracks register residency while walking MIR and
/// can capture interference in a graph. Callers can model spills and reloads
/// without editing MIR, reserve scratch storage, and discard expired portions
/// of the graph as the walk advances.
///
/// Currently these interfaces support ordinary virtual uses and definitions
/// within a block. PHI congruence, operand ties, and physical imaginary register
/// constraints are not yet modeled.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINTERFERENCEGRAPH_H
#define LLVM_LIB_TARGET_MOS_MOSINTERFERENCEGRAPH_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

namespace llvm {

class MachineFunction;
class MachineInstr;
class MachineRegisterInfo;
class RegisterClassInfo;
class MOSRegisterInfo;

/// Captured interference among SSA virtual registers. Each register receives
/// an imaginary storage class, independent of hardware operand constraints.
/// Anonymous vertices reserve scratch storage without introducing MIR
/// registers. Simplification checks whether these interferences guarantee an
/// assignment; the graph does not choose assignments or emit transfers.
class MOSInterferenceGraph {
public:
  MOSInterferenceGraph(MachineFunction &MF, const RegisterClassInfo &RCI);

  /// Test whether conservative simplification can eliminate the entire graph.
  /// If supplied, RemainingRegs receives the virtual registers left after
  /// simplification; anonymous scratch vertices are omitted.
  bool canSimplify(SmallVectorImpl<Register> *RemainingRegs = nullptr) const;

  /// Remove Reg and its incident edges. The caller must establish that Reg
  /// needs no storage anywhere still represented by the graph.
  void remove(Register Reg);

private:
  friend class MOSInterferenceGraphBuilder;

  // Stable until removal. IDs are reused; no references may survive removal.
  using VertexID = unsigned;
  struct Vertex {
    // Zero denotes anonymous scratch; class is specified by ScratchRC.
    Register Reg;
    const TargetRegisterClass *ScratchRC = nullptr;
    SparseBitVector<> Interferences;
  };

  const TargetRegisterClass *regClass(VertexID ID) const;
  // Maximum number of ID's candidates any single assignment of Other blocks.
  unsigned maxBlockedLocations(VertexID ID, VertexID Other) const;
  VertexID getOrCreateVertex(Register Reg);
  VertexID createVertex(Register Reg);
  VertexID createScratchVertex(const TargetRegisterClass &RC);
  void removeVertex(VertexID ID);
  // LiveVertices may include ID; no self-edge is added.
  void addInterferences(VertexID ID, const SparseBitVector<> &LiveVertices);

  const MOSRegisterInfo &TRI;
  MachineRegisterInfo &MRI;
  const RegisterClassInfo &RCI;
  SmallVector<Vertex> Vertices;
  SmallBitVector OccupiedVertices;
  SmallVector<VertexID> FreeVertices;
  DenseMap<Register, VertexID> VirtRegVertices;
  // Bounds depend on classes and aliasing, not graph topology. Ordered pairs:
  // an Imag16 blocks two Imag8 locations, while an Imag8 blocks only one pair.
  mutable DenseMap<
      std::pair<const TargetRegisterClass *, const TargetRegisterClass *>,
      unsigned>
      MaxBlockedLocations;
};

/// Track register residency while walking instructions, optionally recording
/// interference in a graph. MIR and kill flags describe the original SSA names;
/// evict/reload model transfers that the spiller will emit after the walk.
class MOSInterferenceGraphBuilder {
public:
  struct SpillState {
    SmallDenseSet<Register, 8> ResidentValues;
    // Resident values whose spill slots already hold their contents.
    // Nonresident values are always saved; reloading them restores membership
    // in this set.
    SmallDenseSet<Register, 8> SavedResidentValues;
  };

  const SpillState &spillState() const { return State; }
  SmallVector<Register> nonResidentUses(const MachineInstr &MI) const;
  /// Resident values neither read nor defined by MI, eligible for eviction
  /// before it. May be queried immediately before or after stepForward(MI).
  SmallVector<Register> spillCandidates(const MachineInstr &MI) const;
  void evict(Register Reg);
  void reload(Register Reg);

  /// Begin adding to Graph, recording interference among current residents.
  void beginCapture(MOSInterferenceGraph &Graph);
  /// Reserve scratch against the residents at the current instruction boundary.
  void reserveHeadroom(const TargetRegisterClass &RC);
  /// Advance from immediately before MI to immediately after it.
  ///
  /// When capturing, records any additional interference required by MI. All
  /// virtual inputs must already be resident. Ties, subregister operands, early
  /// clobbers, and physical imaginary constraints are not yet supported.
  void stepForward(const MachineInstr &MI);
  /// Discard captured vertices that are no longer resident, including scratch.
  void trimToLive();
  /// Stop recording interference, preserving residency and the captured graph.
  void endCapture();

private:
  using VertexID = MOSInterferenceGraph::VertexID;
  void addInterferences(VertexID ID);
  void makeLive(Register Reg);
  void releaseReg(Register Reg);

  SpillState State;
  MOSInterferenceGraph *Graph = nullptr;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSINTERFERENCEGRAPH_H
