//===-- MOSExpandPseudoInsts.cpp - Expand pseudo instructions -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSTargetMachine.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define MOS_EXPAND_PSEUDO_NAME "MOS pseudo instruction expansion pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual MOS instructions.
class MOSExpandPseudo : public MachineFunctionPass {
public:
  static char ID;

  MOSExpandPseudo() : MachineFunctionPass(ID) {
    initializeMOSExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override { return false; };

  StringRef getPassName() const override { return MOS_EXPAND_PSEUDO_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const MOSRegisterInfo *TRI;
  const TargetInstrInfo *TII;

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode,
                              unsigned DstReg) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode), DstReg);
  }

  template <unsigned OP> bool expand(Block &MBB, BlockIt MBBI);

  bool expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI);
  bool expandMBB(Block &MBB);
  bool expandMI(Block &MBB, BlockIt MBBI);
  bool expandLogic(unsigned Op, Block &MBB, BlockIt MBBI);
  bool expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI);
  MachineRegisterInfo &getRegInfo(Block &MBB) {
    return MBB.getParent()->getRegInfo();
  }

  bool isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const;

  /// Scavenges a free GPR8 register for use.
  unsigned scavengeGPR8(MachineInstr &MI);
};

char MOSExpandPseudo::ID = 0;

} // end of anonymous namespace

INITIALIZE_PASS(MOSExpandPseudo, "mos-expand-pseudo", MOS_EXPAND_PSEUDO_NAME,
                false, false)
namespace llvm {

FunctionPass *createMOSExpandPseudoPass() { return new MOSExpandPseudo(); }

} // end of namespace llvm
