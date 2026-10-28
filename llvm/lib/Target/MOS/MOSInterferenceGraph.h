//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Interference graph management for imaginary register allocation.
///
/// MOSInterferenceGraph records the conflict relationship graph between
/// imaginary storage for virtual registers. This provides the
/// means to check whether registers can be assigned without spilling, and an
/// order in which to assign them.
///
/// MOSInterferenceGraphBuilder tracks register residency while walking MIR. It
/// uses this to capture interferences between registers in a
/// MOSInterferenceGraph.
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

class LiveIntervals;
class LiveRegUnits;
class MachineFunction;
class MachineInstr;
class MachineRegisterInfo;
class RegisterClassInfo;
class MOSRegisterInfo;
class VirtRegMap;

/// Interference graph for the imaginary storage needed by registers within some
/// contiguous region of MIR instructions.
///
/// Each register that could require an imaginary storage location is a vertex
/// in the graph, and there is an edge in the graph between two vertices if the
/// structure of the MIR instructions that the graph covers implies that the two
/// registers cannot be assigned to an overlapping location simultaneously.
///
/// The graph also provides for anonymous temporary vertices. These do not
/// correspond to any actual register, but they instead provide means to
/// guarantee that if a register were to be inserted into the MIR later, it
/// could receive an assignment.
///
/// Some vertices may already have been assigned to some unknown location by the
/// time the graph is being analyzed. In such cases, the vertex may be marked as
/// retained, which means that the graph should not consider the vertex within
/// its purview to assign.
///
/// A vertex is also retained if the graph is given a VirtRegMap that assigns
/// its register. Its location is then known, and simplification needs to
/// accommodate only that location.
///
/// The main operation done on the graph is simplification. This means to
/// logically remove one vertex and its edges at a time from the graph. To
/// safely remove a vertex, we first guarantee that given any assignment of the
/// remainder of the graph, we can trivially assign the vertex to a location
/// that does not overlap with any of its neighbors. This ensures that if
/// vertices are assigned in the reverse order of removal, vertices can always
/// successfully be assigned.
class MOSInterferenceGraph {
public:
  // Stable until removal. IDs of removed vertices are reused by new ones.
  using VertexID = unsigned;

  MOSInterferenceGraph(MachineFunction &MF, const RegisterClassInfo &RCI,
                       const VirtRegMap *VRM = nullptr);

  // Vertices point to their graph, so it cannot be copied or moved.
  MOSInterferenceGraph(const MOSInterferenceGraph &) = delete;
  MOSInterferenceGraph &operator=(const MOSInterferenceGraph &) = delete;

  /// Try to simplify the graph, returning whether it succeeds.
  ///
  /// Failure means that no vertex remaining in the graph can be safely removed.
  /// The algorithm is conservative: success guarantees assignability, but
  /// failure does not prove the graph un-assignable.
  ///
  /// RemainingRegs is set to the virtual registers left in the graph when
  /// simplification ends. AssignmentOrder is set to the removed non-retained
  /// vertices in the reverse of their removal order. Given any assignment of
  /// the other vertices, they can be assigned in this order.
  bool simplify(SmallVectorImpl<Register> *RemainingRegs = nullptr,
                SmallVectorImpl<VertexID> *AssignmentOrder = nullptr) const;

  struct Vertex {
    explicit Vertex(const MOSInterferenceGraph &Graph) : Graph(&Graph) {}

    /// Zero denotes anonymous temporary storage.
    Register Reg;

    /// RC specifies the imaginary storage class for both named and anonymous
    /// vertices.
    const TargetRegisterClass *RC = nullptr;

    SparseBitVector<> Edges;

    /// The locations the vertex may be assigned, in allocation order.
    ArrayRef<MCPhysReg> candidates() const;

    /// The register units that the graph's VirtRegMap currently assigns to
    /// this vertex's neighbors.
    LiveRegUnits neighborAssignedUnits() const;

  private:
    friend class MOSInterferenceGraph;
    friend class MOSInterferenceGraphBuilder;

    VertexID id() const;

    // Maximum number of this vertex's candidates any single assignment of
    // Other blocks.
    unsigned maxBlockedLocations(const Vertex &Other) const;

    // The location the graph's VirtRegMap assigns this vertex's register, if
    // any.
    MCRegister assignment() const;

    // Whether this vertex is retained: marked so, or assigned by the graph's
    // VirtRegMap.
    bool isRetained() const;

    // Adds an edge between this vertex and Other. Other must not be this
    // vertex.
    void addEdge(Vertex &Other);

    const MOSInterferenceGraph *Graph;
  };

  const Vertex &vertex(VertexID ID) const { return Vertices[ID]; }

private:
  friend class MOSInterferenceGraphBuilder;

  /// Retain all current vertices: subsequent simplification must accommodate
  /// their assignments without changing them.
  void retainAll();

  // Creating a vertex may invalidate references to the others.
  Vertex &getOrCreateVertex(Register Reg);
  Vertex &createVertex(const TargetRegisterClass &RC);

  /// Remove a vertex and its incident edges. If the vertex is live, the builder
  /// must first end its lifetime.
  void removeVertex(Vertex &V);

  const MOSRegisterInfo &TRI;
  MachineRegisterInfo &MRI;
  const RegisterClassInfo &RCI;
  const VirtRegMap *VRM;
  SmallVector<Vertex> Vertices;
  SmallBitVector OccupiedVertices;

  // Definitions preceding the current instruction region. Their assignments
  // must be preserved; simplification may only remove the new vertices.
  // Vertices with VirtRegMap assignments are retained even if not marked here.
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

/// Track register liveness while walking instructions, optionally recording
/// interference in a graph. Live intervals describe the original SSA names;
/// callers adjust liveness directly to model transfers not yet in the MIR.
class MOSInterferenceGraphBuilder {
public:
  explicit MOSInterferenceGraphBuilder(LiveIntervals &LIS) : LIS(LIS) {}

  const SmallDenseSet<Register, 8> &liveRegs() const { return LiveRegs; }

  /// Advance from immediately before MI to immediately after it.
  ///
  /// When capturing, records any additional interference required by MI. All
  /// virtual inputs must already be live. Ties and physical imaginary
  /// constraints are not yet supported.
  ///
  void stepForward(const MachineInstr &MI);

  /// Begin adding to Graph, recording interference among the current live
  /// registers and retaining them as earlier definitions whose assignments must
  /// be preserved.
  void beginCapture(MOSInterferenceGraph &Graph);

  // These operations require an active capture.

  /// Make Reg live at the current position, recording its interference with
  /// the current live registers and live temporaries.
  void addLiveReg(Register Reg);

  /// Remove a live Reg, along with its vertex and recorded interference.
  void removeLiveReg(Register Reg);

  /// Define an anonymous temporary of RC at the current position. It interferes
  /// with the current live registers and live temporaries, and remains live
  /// until killTemporaryVertex. Later definitions interfere with it while live.
  /// Does not change liveRegs or retention.
  MOSInterferenceGraph::VertexID
  addTemporaryVertex(const TargetRegisterClass &RC);

  /// End a temporary's lifetime and remove its vertex from the graph.
  void killTemporaryVertex(MOSInterferenceGraph::VertexID ID);

  /// Discard captured vertices that are neither live registers nor live
  /// temporaries.
  void trimToLive();

  /// Retain all vertices in the active capture: subsequent simplification must
  /// accommodate their assignments without changing them.
  void retainAll();

  /// Stop recording interference, preserving liveness and the captured graph.
  /// All temporaries must have been killed.
  void endCapture();

private:
  using Vertex = MOSInterferenceGraph::Vertex;
  using VertexID = MOSInterferenceGraph::VertexID;

  // Start Reg's lifetime at the current position. When capturing, records its
  // interference with the current live registers and live temporaries.
  void defineReg(Register Reg);

  // Start V's lifetime at the current position, recording its interference
  // with the current live registers and live temporaries.
  void defineVertex(Vertex &V);

  LiveIntervals &LIS;
  SmallDenseSet<Register, 8> LiveRegs;

  // Graph-local IDs.
  SmallVector<VertexID, 2> LiveTemporaries;

  MOSInterferenceGraph *Graph = nullptr;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSINTERFERENCEGRAPH_H
