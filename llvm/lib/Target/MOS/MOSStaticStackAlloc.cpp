#include "MOSStaticStackAlloc.h"

#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSSubtarget.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Module.h"
#include "llvm/PassRegistry.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-static-stack-alloc"

using namespace llvm;

namespace {

class MOSStaticStackAlloc : public ModulePass {
public:
  static char ID;

  MOSStaticStackAlloc() : ModulePass(ID) {
    llvm::initializeMOSStaticStackAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

void MOSStaticStackAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<MachineModuleInfoWrapperPass>();
  AU.addPreserved<MachineModuleInfoWrapperPass>();
}

bool MOSStaticStackAlloc::runOnModule(Module &M) {
  MachineModuleInfo &MMI = getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  bool Changed = false;
  for (Function &F : M) {
    MachineFunction *MF = MMI.getMachineFunction(F);
    if (!MF)
      continue;

    const MOSFrameLowering &TFL =
        *MF->getSubtarget<MOSSubtarget>().getFrameLowering();

    uint64_t Size = TFL.staticSize(MF->getFrameInfo());
    if (!Size)
      continue;

    LLVM_DEBUG(dbgs() << "Found static stack for " << F.getName() << "\n");
    LLVM_DEBUG(dbgs() << "Size " << Size << "\n");

    Type *Typ = ArrayType::get(Type::getInt8Ty(M.getContext()), Size);
    GlobalVariable *Stack =
        new GlobalVariable(M, Typ, false, GlobalValue::PrivateLinkage,
                           UndefValue::get(Typ), Twine(F.getName()) + "_sstk");
    LLVM_DEBUG(dbgs() << "Allocated: " << *Stack << "\n");
    Changed = true;

    for (MachineBasicBlock &MBB : *MF) {
      for (MachineInstr &MI : MBB) {
        for (MachineOperand &MO : MI.operands()) {
          if (!MO.isTargetIndex())
            continue;
          MO.ChangeToGA(Stack, MO.getOffset(), MO.getTargetFlags());
        }
      }
    }
  }
  return Changed;
}

} // namespace

char MOSStaticStackAlloc::ID = 0;

INITIALIZE_PASS(MOSStaticStackAlloc, DEBUG_TYPE,
                "Allocate non-recursive stack to static memory", false, false)

ModulePass *llvm::createMOSStaticStackAllocPass() {
  return new MOSStaticStackAlloc();
}
