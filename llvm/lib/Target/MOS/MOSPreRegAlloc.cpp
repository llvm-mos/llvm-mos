#include "MOSPreRegAlloc.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineOperand.h"

#define DEBUG_TYPE "mos-preregalloc"

using namespace llvm;

namespace {

struct MOSPreRegAlloc : public MachineFunctionPass {
  static char ID;

  MOSPreRegAlloc() : MachineFunctionPass(ID) {
    initializeMOSPreRegAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool couldContainA(Register Reg, const MachineRegisterInfo &MRI) {
  if (Reg.isPhysical())
    return Reg == MOS::A;
  return MRI.getRegClass(Reg)->contains(MOS::A);
}

bool MOSPreRegAlloc::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  bool Changed = false;

  for (auto &MBB : MF) {
    for (auto &MI : MBB) {
      switch (MI.getOpcode()) {
      case MOS::ASL:
      case MOS::ROL: {
        Register Dst = MI.getOperand(0).getReg();
        Register Src = MI.getOperand(2).getReg();
        if (couldContainA(Dst, MRI) || !couldContainA(Src, MRI))
          continue;

        Register DstCopy = MRI.createVirtualRegister(&MOS::AZPRegClass);
        MI.getOperand(0).setReg(DstCopy);
        MachineIRBuilder Builder(MBB, std::next(MI.getIterator()));
        Builder.buildCopy(Dst, DstCopy);
        Changed = true;
        break;
      }
      }
    }
  }

  return Changed;
}

} // namespace

MachineFunctionPass *llvm::createMOSPreRegAlloc() {
  return new MOSPreRegAlloc;
}

char MOSPreRegAlloc::ID = 0;

INITIALIZE_PASS(MOSPreRegAlloc, DEBUG_TYPE,
                "Adjust instructions before register allocation.", false, false)
