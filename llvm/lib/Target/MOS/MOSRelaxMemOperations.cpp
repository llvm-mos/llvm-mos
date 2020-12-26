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
    Modified |= runOnInstruction(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

template <>
bool MOSRelaxMem::relax<MOS::STDWPtrQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  MachineOperand &Ptr = MI.getOperand(0);
  MachineOperand &Src = MI.getOperand(2);
  int64_t Imm = MI.getOperand(1).getImm();

  // We can definitely optimise this better.
  if (Imm > 63) {
    // Push the previous state of the pointer register.
    // This instruction must preserve the value.
    buildMI(MBB, MBBI, MOS::PUSHWRr)
      .addReg(Ptr.getReg());

    // Add the immediate to the pointer register.
    buildMI(MBB, MBBI, MOS::SBCIWRdK)
      .addReg(Ptr.getReg(), RegState::Define)
      .addReg(Ptr.getReg())
      .addImm(-Imm);

    // Store the value in the source register to the address
    // pointed to by the pointer register.
    buildMI(MBB, MBBI, MOS::STWPtrRr)
      .addReg(Ptr.getReg())
      .addReg(Src.getReg(), getKillRegState(Src.isKill()));

    // Pop the original state of the pointer register.
    buildMI(MBB, MBBI, MOS::POPWRd)
      .addReg(Ptr.getReg(), getKillRegState(Ptr.isKill()));

    MI.removeFromParent();
  }

  return false;
}

bool MOSRelaxMem::runOnInstruction(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define RELAX(Op)                \
  case Op:                       \
    return relax<Op>(MBB, MI)

  switch (Opcode) {
    RELAX(MOS::STDWPtrQRr);
  }
#undef RELAX
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(MOSRelaxMem, "avr-relax-mem",
                MOS_RELAX_MEM_OPS_NAME, false, false)

namespace llvm {

FunctionPass *createMOSRelaxMemPass() { return new MOSRelaxMem(); }

} // end of namespace llvm
