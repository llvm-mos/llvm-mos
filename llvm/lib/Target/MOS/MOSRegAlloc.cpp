//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Allocate and assign final physical registers and exit SSA form.
///
/// MOSImagRegAssign supplies an SSA program with imaginary register assignments
/// in VirtRegMap and explicit spills. This pass chooses hardware placements and
/// emits the transfers needed to satisfy uses, while respecting those imaginary
/// assignments wherever imaginary storage is required. The resulting MIR has
/// physical operands.
///
/// Liveness comes from LiveIntervals; kill and dead flags are not trusted on
/// input, and are computed for the physical operands afterwards.
///
/// This pass finds a cheapest use of hardware registers and assigned imaginary
/// registers with Dijkstra's algorithm. A position is the point just before one
/// of the block's instructions, or the block's end. A state is a hardware
/// allocation at a position. An action is either a transfer, which leads to
/// another allocation at the same position, or an assignment of the operands of
/// the instruction after the position, which leads to an allocation at the next
/// position; it costs what the emitted instruction costs with those operands.
/// States are closed in order of path cost. Each closed state refers back to
/// the closed state it was reached from, and the solution is read off those
/// references once the goal, at the last position, is closed.
///
/// Transfers store hardware registers to imaginary storage, reload them, and
/// move flags to and from the LSBs of byte registers. Every value has an
/// imaginary assignment, so these suffice to reach any allocation.
///
/// Currently, allocation is limited to a single non-looping basic block, and
/// REG_SEQUENCE is not yet supported.
///
//===----------------------------------------------------------------------===//

#include "MOSRegAlloc.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrCost.h"
#include "MOSInstrInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/ADT/Hashing.h"
#include "llvm/ADT/PriorityQueue.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <array>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <optional>
#include <tuple>
#include <utility>

#define DEBUG_TYPE "mos-regalloc"

using namespace llvm;

namespace {

/// What the code before a position guarantees about hardware register
/// contents. An entry names the live SSA value its register is guaranteed to
/// hold; a zero register guarantees nothing. Live values absent from hardware
/// are guaranteed to be in imaginary storage.
struct Alloc {
  enum { A, X, Y, C, V, NumRegs };
  /// Full physical registers corresponding to the entries in Regs.
  static constexpr MCPhysReg HardwareRegs[] = {MOS::A, MOS::X, MOS::Y, MOS::C,
                                               MOS::V};
  std::array<Register, NumRegs> Regs = {};

  /// One bit per hardware register, in the same order as Regs. A set bit means
  /// imaginary storage is also guaranteed to contain the value named by the
  /// corresponding entry; a clear bit means it is not guaranteed to. Empty
  /// entries have clear bits; hardware copies of the same value have matching
  /// bits.
  std::bitset<NumRegs> ImagRegsValid;

  /// Whether imaginary storage is guaranteed to contain the live value Value.
  bool isImagRegValid(Register Value) const {
    const auto *I = llvm::find(Regs, Value);
    return I == Regs.end() || ImagRegsValid[I - Regs.begin()];
  }

  /// Record that imaginary storage is guaranteed to contain Value.
  void setImagRegValid(Register Value) {
    for (unsigned I = 0; I != NumRegs; ++I)
      if (Regs[I] == Value)
        ImagRegsValid.set(I);
  }

  /// Whether going from this allocation to To loses a value: one that this
  /// guarantees only in hardware registers is in none of To's, so it could
  /// never be read again.
  bool losesValue(const Alloc &To) const {
    for (unsigned I = 0; I != NumRegs; ++I)
      if (Regs[I] && !ImagRegsValid[I] && !llvm::is_contained(To.Regs, Regs[I]))
        return true;
    return false;
  }

  bool operator==(const Alloc &Other) const {
    return Regs == Other.Regs && ImagRegsValid == Other.ImagRegsValid;
  }
};

/// A state of the search: a hardware allocation at a position. A position is
/// the point just before one of the block's non-debug instructions, or the
/// block's end. An allocation describes the hardware registers at one position,
/// and transfers are inserted at positions.
struct State {
  /// The position, as a number. Position 0 is just before the block's first
  /// non-debug instruction, each later position has the next number, and the
  /// last is the block's end.
  unsigned Pos;
  Alloc A;

  bool operator==(const State &Other) const {
    return Pos == Other.Pos && A == Other.A;
  }
};

} // namespace

template <> struct llvm::DenseMapInfo<State> {
  static unsigned getHashValue(const State &Value) {
    return hash_combine(
        Value.Pos,
        hash_combine_range(
            map_range(Value.A.Regs, [](Register Reg) { return Reg.id(); })),
        Value.A.ImagRegsValid.to_ulong());
  }

  static bool isEqual(const State &LHS, const State &RHS) { return LHS == RHS; }
};

namespace {

/// What the pass records about a position.
struct Position {
  /// The instruction the position is just before. Null at the block's end.
  MachineInstr *NextMI;
  /// The SSA values live at the position, in order of definition.
  SmallVector<Register, 8> LiveValues;
};

/// An action: one transfer or completed instruction assignment. MI names an
/// original instruction; a null MI instead selects TransferOpcode. OpRegs is
/// indexed by operand number, with noreg entries for non-register operands,
/// and records physical registers after applying the original operand's
/// subregister index. Original opcodes and non-register operands come from MI.
struct Action {
  MachineInstr *MI = nullptr;
  unsigned TransferOpcode = 0;
  SmallVector<Register, 8> OpRegs;
};

/// The last edge of a path: an action, and the state it was taken at.
struct ParentEdge {
  State Parent;
  Action Act;
};

/// An open node: a state, and a path that reaches it from the initial state.
/// A state gets another node whenever a cheaper path to it is found; its
/// earlier nodes are then out of date.
struct OpenNode {
  State S;
  /// The cost of the path, as a number in the function's cost mode.
  int64_t PathCost;
  /// The last edge of the path; the rest is the parent's path. Only the node
  /// of the initial state has none.
  std::optional<ParentEdge> Edge;
  /// Order in which the paths were found. Among equal path costs the earliest
  /// found is closed first. The priority queue does not define the order of
  /// equal elements, so without this the solution chosen could differ between
  /// standard libraries.
  unsigned Seq;
};

/// Orders the queue of open nodes so that its top is the next node to close.
struct LaterOpenNode {
  bool operator()(const OpenNode &A, const OpenNode &B) const {
    return std::tie(A.PathCost, A.Seq) > std::tie(B.PathCost, B.Seq);
  }
};

class MOSRegAlloc : public MachineFunctionPass {
public:
  static char ID;

  MOSRegAlloc() : MachineFunctionPass(ID) {
    initializeMOSRegAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override {
    if (MF.size() != 1 || !MF.front().succ_empty())
      report_fatal_error(
          "MOSRegAlloc requires a single non-looping basic block", false);
    STI = &MF.getSubtarget<MOSSubtarget>();
    TII = STI->getInstrInfo();
    TRI = STI->getRegisterInfo();
    MRI = &MF.getRegInfo();
    VRM = &getAnalysis<VirtRegMapWrapperLegacy>().getVRM();
    LIS = &getAnalysis<LiveIntervalsWrapperPass>().getLIS();
    CostMode = MOSInstrCost::getModeFor(MF);
    return allocMBB(MF.front());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LiveIntervalsWrapperPass>();
    AU.addRequired<VirtRegMapWrapperLegacy>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }

  MachineFunctionProperties getSetProperties() const override {
    return MachineFunctionProperties().setNoPHIs().setNoVRegs();
  }

  MachineFunctionProperties getClearedProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }

private:
  bool allocMBB(MachineBasicBlock &MBB) {
    if (llvm::any_of(MBB,
                     [](const MachineInstr &MI) { return MI.isRegSequence(); }))
      report_fatal_error("MOSRegAlloc does not yet support REG_SEQUENCE",
                         false);

    collectPositions(MBB);
    State Goal = solveMBB(MBB);
    applySolution(MBB, Goal);

    Positions.clear();
    StateCost.clear();
    OpenNodes.clear();
    ClosedStateParentEdge.clear();
    return true;
  }

  /// List the block's positions in order, each with the instruction after it
  /// and the SSA values live at it. LiveIntervals can tell whether one value
  /// is live at a position, but cannot list the values live there.
  void collectPositions(MachineBasicBlock &MBB) {
    Positions.clear();
    SmallSetVector<Register, 16> LiveValues;
    for (MachineInstr &MI : MBB) {
      if (MI.isDebugOrPseudoInstr())
        continue;
      Positions.push_back({&MI, {LiveValues.begin(), LiveValues.end()}});

      for (const MachineOperand &Use : MI.all_uses())
        if (Use.getReg().isVirtual() && isKill(MI, Use.getReg()))
          LiveValues.remove(Use.getReg());
      for (const MachineOperand &Def : MI.all_defs())
        if (Def.getReg().isVirtual() && !isDeadDef(MI, Def.getReg()))
          LiveValues.insert(Def.getReg());
    }
    // The block's end.
    Positions.push_back({nullptr, {LiveValues.begin(), LiveValues.end()}});
  }

  /// Close states in order of path cost, from the initial state, the empty
  /// allocation at the block's entry, until the goal state at the block's end
  /// is closed. Return the goal state.
  State solveMBB(MachineBasicBlock &MBB) {
    StateCost.clear();
    OpenNodes.clear();
    ClosedStateParentEdge.clear();
    NextSeq = 0;

    relax({0, {}}, 0, std::nullopt);
    while (!OpenNodes.empty()) {
      OpenNode Node = OpenNodes.top();
      OpenNodes.pop();
      // An out-of-date node comes after the cheaper node of its state, so it
      // finds the state closed.
      if (!ClosedStateParentEdge.try_emplace(Node.S, std::move(Node.Edge))
               .second)
        continue;

      // Only the block's end has no instruction.
      MachineInstr *NextMI = Positions[Node.S.Pos].NextMI;
      if (!NextMI) {
        assert(Node.S.A == Alloc() &&
               "Expected an empty allocation at basic-block exit");
        LLVM_DEBUG({
          dbgs() << "Allocation cost for " << MBB.getParent()->getName() << ": "
                 << Node.PathCost << '\n';
          dbgs() << "Closed " << ClosedStateParentEdge.size() << " states\n";
        });
        return Node.S;
      }

      expandTransfers(Node);
      expandInstr(Node, *NextMI);
    }
    // Every value has an imaginary assignment, so the goal is reachable.
    report_fatal_error("MOSRegAlloc found no allocation");
  }

  /// Open a node for To with the path of the given cost that ends with Edge,
  /// unless a path to To at least as cheap is already known. That includes
  /// every closed state: no later path is cheaper than the one it was closed
  /// with.
  void relax(const State &To, int64_t PathCost,
             std::optional<ParentEdge> Edge) {
    auto [I, Inserted] = StateCost.try_emplace(To, PathCost);
    if (!Inserted) {
      if (PathCost >= I->second)
        return;
      I->second = PathCost;
    }
    OpenNodes.push({To, PathCost, std::move(Edge), NextSeq++});
  }

  /// Rewrite original instructions and insert transfers along the path that
  /// Goal was closed with.
  void applySolution(MachineBasicBlock &MBB, const State &Goal) {
    // Follow the parent edges back to the initial state, applying their
    // actions from last to first. A transfer goes just before the action after
    // it. There always is one: no transfer is taken at the block's end, where
    // the search stops.
    MachineInstr *InsertBefore = nullptr;
    State S = Goal;
    while (const std::optional<ParentEdge> &Edge =
               ClosedStateParentEdge.find(S)->second) {
      const Action &Act = Edge->Act;
      if (Act.MI) {
        MachineInstr &MI = *Act.MI;
        assert(Act.OpRegs.size() == MI.getNumOperands());
        for (auto [I, MO] : llvm::enumerate(MI.operands())) {
          if (!MO.isReg())
            continue;
          MO.setReg(Act.OpRegs[I]);
          // OpRegs already includes the original subregister selection.
          MO.setSubReg(0);
        }
        InsertBefore = &MI;
      } else {
        const MCInstrDesc &Desc = TII->get(Act.TransferOpcode);
        auto Transfer =
            BuildMI(MBB, InsertBefore, InsertBefore->getDebugLoc(), Desc);
        for (auto [OpIdx, Phys] : llvm::enumerate(Act.OpRegs))
          Transfer.addReg(Phys, getDefRegState(OpIdx < Desc.getNumDefs()));
        InsertBefore = Transfer;
      }
      S = Edge->Parent;
    }

    // A value can move between hardware and imaginary storage. Until debug
    // locations track these moves, do not retain virtual-register locations.
    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      if (!MI.isDebugInstr() ||
          !llvm::any_of(MI.operands(), [](const MachineOperand &MO) {
            return MO.isReg() && MO.getReg().isVirtual();
          }))
        continue;
      if (MI.isDebugValue())
        MI.setDebugValueUndef();
      else
        MI.eraseFromParent();
    }

    // Flag the lifetimes of the selected physical copies.
    recomputeLivenessFlags(MBB);
    VRM->clearAllVirt();
    MRI->clearVirtRegs();
  }

  /// Open the states at the next position, one for each assignment of MI, the
  /// instruction after the state of Node, that loses no value.
  void expandInstr(const OpenNode &Node, MachineInstr &MI) {
    const Alloc &From = Node.S.A;

    // The inputs that MI kills are in no register after it.
    Alloc Kept = From;
    for (unsigned I = 0; I != Alloc::NumRegs; ++I) {
      if (!From.Regs[I] || !isKill(MI, From.Regs[I]))
        continue;
      Kept.Regs[I] = {};
      Kept.ImagRegsValid.reset(I);
    }

    for (const SmallVector<Register, 8> &OpRegs : instrAssignments(From, MI)) {
      // A definition replaces what its register held. A value defined in a
      // hardware register is not in imaginary storage; a physical or dead
      // definition only empties its register.
      Alloc To = Kept;
      for (const MachineOperand &Def : MI.all_defs()) {
        Register Value = Def.getReg();
        if (!Value.isVirtual() || isDeadDef(MI, Value))
          Value = Register();
        for (unsigned I = 0; I != Alloc::NumRegs; ++I) {
          if (!TRI->regsOverlap(OpRegs[Def.getOperandNo()],
                                Alloc::HardwareRegs[I]))
            continue;
          To.Regs[I] = Value;
          To.ImagRegsValid.reset(I);
        }
      }
      if (Kept.losesValue(To))
        continue;
      int64_t Cost = getInstrCost(MI, OpRegs).value(CostMode);
      relax({Node.S.Pos + 1, To}, Node.PathCost + Cost,
            ParentEdge{Node.S, {&MI, 0, OpRegs}});
    }
  }

  /// The assignments of MI in the allocation From: every way of giving each
  /// of its register operands one of its operandRegs with no two operands in
  /// conflict. Each is indexed like Action::OpRegs.
  SmallVector<SmallVector<Register, 8>, 4>
  instrAssignments(const Alloc &From, const MachineInstr &MI) const {
    // The ways of assigning the operands handled so far. Each operand extends
    // every one of them with every register it can take.
    SmallVector<SmallVector<Register, 8>, 4> Assignments(
        1, SmallVector<Register, 8>(MI.getNumOperands()));
    SmallVector<SmallVector<Register, 8>, 4> NextAssignments;
    for (const MachineOperand &MO : MI.operands()) {
      if (!MO.isReg() || !MO.getReg())
        continue;
      SmallVector<Register, 6> Regs = operandRegs(From, MO);
      NextAssignments.clear();
      for (const SmallVector<Register, 8> &OpRegs : Assignments) {
        for (Register Phys : Regs) {
          if (llvm::any_of(llvm::enumerate(OpRegs), [&](const auto &Other) {
                return Other.value() &&
                       operandsConflict(MO, MI.getOperand(Other.index())) &&
                       TRI->regsOverlap(Phys, Other.value());
              }))
            continue;
          NextAssignments.push_back(OpRegs);
          NextAssignments.back()[MO.getOperandNo()] = Phys;
        }
      }
      std::swap(Assignments, NextAssignments);
    }
    return Assignments;
  }

  /// The registers that MO, an operand with a register, can be given in the
  /// allocation From, whatever MI's other operands get. A physical operand
  /// keeps its register.
  SmallVector<Register, 6> operandRegs(const Alloc &From,
                                       const MachineOperand &MO) const {
    Register Value = MO.getReg();
    if (Value.isPhysical())
      return {Value};
    assert((MO.isUse() || !MO.getSubReg()) &&
           "expected a whole virtual definition in SSA");
    SmallVector<Register, 6> Regs = physRegsForValue(Value);

    // A use reads a register that holds its value, and may name only part of
    // the value.
    if (MO.isUse())
      llvm::erase_if(Regs, [&](Register Phys) {
        return !physRegHoldsValue(From, Phys, Value);
      });
    if (MO.getSubReg())
      for (Register &Phys : Regs)
        Phys = TRI->getSubReg(Phys, MO.getSubReg());

    // The register must be in the operand's class. A definition must also be
    // free at the slot where MI writes it.
    const MachineInstr &MI = *MO.getParent();
    const TargetRegisterClass *RC =
        TII->getRegClass(MI.getDesc(), MO.getOperandNo());
    SlotIndex DefSlot =
        LIS->getInstructionIndex(MI).getRegSlot(MO.isEarlyClobber());
    llvm::erase_if(Regs, [&](Register Phys) {
      return !Phys || (RC && !RC->contains(Phys)) ||
             (MO.isDef() && !isPhysRegFree(DefSlot, Phys));
    });
    return Regs;
  }

  /// Whether From guarantees that Phys, a register that Value fits in, holds
  /// it.
  bool physRegHoldsValue(const Alloc &From, Register Phys,
                         Register Value) const {
    if (std::optional<unsigned> RegIndex = hardwareRegIndex(Phys))
      return From.Regs[*RegIndex] == Value;
    // Phys is in the imaginary register of Value.
    return From.isImagRegValid(Value);
  }

  /// Cost MI with the registers in OpRegs for its register operands.
  MOSInstrCost getInstrCost(const MachineInstr &MI,
                            ArrayRef<Register> OpRegs) const {
    // Change copies, so that MI and its register use lists stay as they are.
    SmallVector<MachineOperand, 8> Operands(MI.operands());
    for (auto [I, MO] : llvm::enumerate(Operands)) {
      MO.clearParent();
      if (!MO.isReg())
        continue;
      MO.setReg(OpRegs[I]);
      // OpRegs already includes the original subregister selection.
      MO.setSubReg(0);
    }
    return TII->getInstrCost(MI.getOpcode(), Operands);
  }

  /// Whether two register operands of one instruction must use disjoint
  /// storage. Prescribed overlaps between physical operands are already part
  /// of the MIR; distinct virtual placements must respect the instruction's
  /// read/write ordering.
  static bool operandsConflict(const MachineOperand &A,
                               const MachineOperand &B) {
    if (&A == &B || (A.getReg().isPhysical() && B.getReg().isPhysical()))
      return false;
    if (A.isDef() && B.isDef())
      return true;
    if (A.isUse() && B.isUse())
      return false;

    return (A.isDef() ? A : B).isEarlyClobber();
  }

  /// Open the states one transfer away from the state of Node. A transfer
  /// copies a live value from a register that holds it to one that does not.
  /// One end of every transfer is a byte register, A, X or Y: a byte is stored
  /// to or reloaded from imaginary storage, and a flag is copied between C or
  /// V and a byte's LSB. So a flag reaches imaginary storage through a byte
  /// register, which keeps that placement decision visible to the search, and
  /// bytes move between byte registers through imaginary storage.
  void expandTransfers(const OpenNode &Node) {
    const Alloc &From = Node.S.A;
    // The slot just before the next instruction, where a transfer goes.
    SlotIndex Idx = LIS->getInstructionIndex(*Positions[Node.S.Pos].NextMI);
    assert(llvm::all_of(From.Regs,
                        [&](Register Value) {
                          return !Value || LIS->getInterval(Value).liveAt(Idx);
                        }) &&
           "Dead value in allocation");
    const auto IsByteReg = [&](Register Phys) {
      std::optional<unsigned> RegIndex = hardwareRegIndex(Phys);
      return RegIndex && *RegIndex <= Alloc::Y;
    };

    for (Register Value : Positions[Node.S.Pos].LiveValues) {
      SmallVector<Register, 6> Regs = physRegsForValue(Value);
      for (Register SrcPhys : Regs) {
        if (!physRegHoldsValue(From, SrcPhys, Value))
          continue;
        for (Register DstPhys : Regs) {
          if (physRegHoldsValue(From, DstPhys, Value) ||
              IsByteReg(SrcPhys) == IsByteReg(DstPhys) ||
              !isPhysRegFree(Idx, DstPhys))
            continue;

          // A copy to a hardware register replaces what it held; a store
          // changes no hardware register.
          Alloc To = From;
          if (std::optional<unsigned> DstIndex = hardwareRegIndex(DstPhys)) {
            To.Regs[*DstIndex] = Value;
            To.ImagRegsValid[*DstIndex] = From.isImagRegValid(Value);
          } else {
            To.setImagRegValid(Value);
          }
          if (From.losesValue(To))
            continue;

          int64_t Cost =
              getTransferCost(MOS::COPY, {DstPhys, SrcPhys}).value(CostMode);
          relax({Node.S.Pos, To}, Node.PathCost + Cost,
                ParentEdge{Node.S, {nullptr, MOS::COPY, {DstPhys, SrcPhys}}});
        }
      }
    }
  }

  /// Cost a transfer using its selected physical operands.
  MOSInstrCost getTransferCost(unsigned TransferOpcode,
                               ArrayRef<Register> OpRegs) const {
    SmallVector<MachineOperand, 4> Operands;
    unsigned NumDefs = TII->get(TransferOpcode).getNumDefs();
    for (auto [I, Reg] : llvm::enumerate(OpRegs))
      Operands.push_back(MachineOperand::CreateReg(Reg, I < NumDefs));
    return TII->getInstrCost(TransferOpcode, Operands);
  }

  /// Whether a write at Slot may use Phys: no part of it holds physical
  /// register contents that are live there. These constrain writes even though
  /// no Alloc entry names them. Reserved registers are never free.
  bool isPhysRegFree(SlotIndex Slot, Register Phys) const {
    return !MRI->isReserved(Phys) &&
           llvm::none_of(TRI->regunits(Phys), [&](MCRegUnit Unit) {
             return LIS->getRegUnit(Unit).liveAt(Slot);
           });
  }

  /// Whether MI's read of the SSA value ends its live range.
  bool isKill(const MachineInstr &MI, Register Value) const {
    return LIS->getInterval(Value).Query(LIS->getInstructionIndex(MI)).isKill();
  }

  /// Whether MI defines the SSA value without it being live afterwards.
  bool isDeadDef(const MachineInstr &MI, Register Value) const {
    return LIS->getInterval(Value)
        .Query(LIS->getInstructionIndex(MI))
        .isDeadDef();
  }

  /// Return the physical register or subregister that can hold Value at
  /// RegIndex in Alloc::Regs (Alloc::A, X, Y, C, or V), or zero if it cannot
  /// fit. Boolean values use the LSB subregister of A, X, or Y, or the whole C
  /// or V. Imag16 values cannot fit in any single hardware register.
  Register hardwareReg(Register Value, unsigned RegIndex) const {
    switch (TRI->getRegSizeInBits(Value, *MRI).getFixedValue()) {
    case 1:
      return RegIndex <= Alloc::Y
                 ? Register(TRI->getSubReg(Alloc::HardwareRegs[RegIndex],
                                           MOS::sublsb))
                 : Register(Alloc::HardwareRegs[RegIndex]);
    case 8:
      return RegIndex <= Alloc::Y ? Register(Alloc::HardwareRegs[RegIndex])
                                  : Register();
    default:
      return Register();
    }
  }

  /// The index in Alloc::Regs of the hardware register that Phys is or is the
  /// LSB of, or nothing if Phys is in an imaginary register.
  std::optional<unsigned> hardwareRegIndex(Register Phys) const {
    for (unsigned I = 0; I != Alloc::NumRegs; ++I)
      if (TRI->regsOverlap(Phys, Alloc::HardwareRegs[I]))
        return I;
    return std::nullopt;
  }

  /// The registers that Value fits in: the hardware registers of its size, or
  /// their LSBs for a flag, and its imaginary register.
  SmallVector<Register, 6> physRegsForValue(Register Value) const {
    SmallVector<Register, 6> Regs;
    for (unsigned I = 0; I != Alloc::NumRegs; ++I)
      if (Register Phys = hardwareReg(Value, I))
        Regs.push_back(Phys);
    Regs.push_back(VRM->getPhys(Value));
    return Regs;
  }

  // The block's positions, indexed by position number.
  SmallVector<Position, 0> Positions;

  // The cost of the cheapest path found so far to each state reached.
  DenseMap<State, int64_t> StateCost;
  // The open set. It keeps out-of-date nodes; they are skipped when their
  // turn comes.
  PriorityQueue<OpenNode, SmallVector<OpenNode, 0>, LaterOpenNode> OpenNodes;
  // The closed set: each closed state, with the last edge of the path it was
  // closed with. Only the initial state has none.
  DenseMap<State, std::optional<ParentEdge>> ClosedStateParentEdge;
  unsigned NextSeq = 0;

  const MOSSubtarget *STI = nullptr;
  const MOSInstrInfo *TII = nullptr;
  const MOSRegisterInfo *TRI = nullptr;
  MachineRegisterInfo *MRI = nullptr;
  VirtRegMap *VRM = nullptr;
  LiveIntervals *LIS = nullptr;
  MOSInstrCost::Mode CostMode;
};

} // namespace

char MOSRegAlloc::ID = 0;
INITIALIZE_PASS_BEGIN(MOSRegAlloc, DEBUG_TYPE, "MOS Register Allocation", false,
                      false)
INITIALIZE_PASS_DEPENDENCY(LiveIntervalsWrapperPass)
INITIALIZE_PASS_DEPENDENCY(VirtRegMapWrapperLegacy)
INITIALIZE_PASS_END(MOSRegAlloc, DEBUG_TYPE, "MOS Register Allocation", false,
                    false)

MachineFunctionPass *llvm::createMOSRegAllocPass() { return new MOSRegAlloc; }
