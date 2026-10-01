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
/// without editing MIR, model temporary storage, and discard expired portions
/// of the graph as the walk advances.
///
/// Currently these interfaces support ordinary virtual uses and definitions
/// within a block. PHI congruence, operand ties, and physical imaginary
/// register constraints are not yet modeled.
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

/// Captured interference among SSA virtual registers. Each vertex requires one
/// location from its imaginary register class, independently of hardware
/// operand constraints. A vertex can name a virtual register or describe
/// anonymous temporary storage. Simplification checks whether these
/// interferences guarantee an assignment; the graph does not choose assignments
/// or emit transfers.
class MOSInterferenceGraph {
public:
  // Stable until removal. IDs are reused; no references may survive removal.
  using VertexID = unsigned;

  MOSInterferenceGraph(MachineFunction &MF, const RegisterClassInfo &RCI);

  /// Test whether conservative simplification can eliminate all new vertices
  /// while retaining earlier definitions. Their assignments are unknown, so
  /// their full squeeze remains throughout simplification. Success guarantees
  /// that any legal assignment of the retained vertices can be extended.
  /// On failure, RemainingRegs receives the virtual registers left, including
  /// retained registers that the caller can choose to spill. Anonymous vertices
  /// are omitted. On success, RemainingRegs is cleared.
  bool canSimplify(SmallVectorImpl<Register> *RemainingRegs = nullptr) const;

  /// Retain all current vertices: subsequent simplification must accommodate
  /// their assignments without changing them.
  void retainAll();

  /// Remove Reg and its incident edges. The caller must establish that Reg
  /// needs no storage anywhere still represented by the graph.
  void remove(Register Reg);
  /// Remove a vertex and its incident edges, including an anonymous temporary.
  /// The builder must first kill a live temporary.
  void removeVertex(VertexID ID);

private:
  friend class MOSInterferenceGraphBuilder;

  struct Vertex {
    // Zero denotes anonymous temporary storage. RC specifies the imaginary
    // storage class for both named and anonymous vertices.
    Register Reg;
    const TargetRegisterClass *RC = nullptr;
    SparseBitVector<> Interferences;
  };

  // Maximum number of ID's candidates any single assignment of Other blocks.
  unsigned maxBlockedLocations(VertexID ID, VertexID Other) const;
  VertexID getOrCreateVertex(Register Reg);
  VertexID createVertex(const TargetRegisterClass &RC);
  void addInterference(VertexID ID, VertexID Other);
  // LiveVertices may include ID; no self-edge is added.
  void addInterferences(VertexID ID, const SparseBitVector<> &LiveVertices);

  const MOSRegisterInfo &TRI;
  MachineRegisterInfo &MRI;
  const RegisterClassInfo &RCI;
  SmallVector<Vertex> Vertices;
  SmallBitVector OccupiedVertices;
  // Definitions preceding the current instruction region. Their assignments
  // must be preserved; simplification may only remove the new vertices.
  SmallBitVector RetainedVertices;
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
  void evict(Register Reg);
  void reload(Register Reg);

  /// Begin adding to Graph, recording interference among current residents and
  /// retaining them as earlier definitions whose assignments must be preserved.
  void beginCapture(MOSInterferenceGraph &Graph);
  /// Define an anonymous temporary of RC at the current position. It interferes
  /// with current residents and live temporaries, and remains live until
  /// killTemporaryVertex. Later definitions interfere with it while live.
  /// Requires an active capture; does not change spillState or retention.
  MOSInterferenceGraph::VertexID
  addTemporaryVertex(const TargetRegisterClass &RC);
  /// End a temporary's lifetime without removing its vertex or recorded edges.
  /// The caller can then discard it with Graph.removeVertex or keep its
  /// interference for subsequent graph queries.
  void killTemporaryVertex(MOSInterferenceGraph::VertexID ID);
  /// Advance from immediately before MI to immediately after it.
  ///
  /// When capturing, records any additional interference required by MI. All
  /// virtual inputs must already be resident. Ties, subregister operands, early
  /// clobbers, and physical imaginary constraints are not yet supported.
  void stepForward(const MachineInstr &MI);
  /// Discard captured vertices that are neither resident nor live temporaries.
  void trimToLive();
  /// Stop recording interference, preserving residency and the captured graph.
  /// All temporaries must have been killed.
  void endCapture();

private:
  using VertexID = MOSInterferenceGraph::VertexID;
  void addInterferences(VertexID ID);
  void makeLive(Register Reg);
  void releaseReg(Register Reg);

  SpillState State;
  // Graph-local IDs. Killed temporaries may remain in the captured graph.
  SmallVector<VertexID, 2> LiveTemporaries;
  MOSInterferenceGraph *Graph = nullptr;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSINTERFERENCEGRAPH_H
