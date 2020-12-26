//===-- MOSRelaxMemOperations.cpp - Relax out of range loads/stores -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass which relaxes out of range memory operations into
// equivalent operations which handle bigger addresses.
//
//===----------------------------------------------------------------------===//

#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define MOS_RELAX_MEM_OPS_NAME "MOS memory operation relaxation pass"

namespace {

class MOSRelaxMem : public MachineFunctionPass {
public:
  static char ID;

  MOSRelaxMem() : MachineFunctionPass(ID) {
    initializeMOSRelaxMemPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return MOS_RELAX_MEM_OPS_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const TargetInstrInfo *TII;

  template <unsigned OP> bool relax(Block &MBB, BlockIt MBBI);

  bool runOnBasicBlock(Block &MBB);
  bool runOnInstruction(Block &MBB, BlockIt MBBI);

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }
};

char MOSRelaxMem::ID = 0;

bool MOSRelaxMem::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  TII = STI.getInstrInfo();

  for (Block &MBB : MF) {
    bool BlockModified = runOnBasicBlock(MBB);
    Modified |= BlockModified;
  }

  return Modified;
}

bool MOSRelaxMem::runOnBasicBlock(Block &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}



} // end of anonymous namespace

INITIALIZE_PASS(MOSRelaxMem, "mos-relax-mem",
                MOS_RELAX_MEM_OPS_NAME, false, false)

namespace llvm {

FunctionPass *createMOSRelaxMemPass() { return new MOSRelaxMem(); }

} // end of namespace llvm
