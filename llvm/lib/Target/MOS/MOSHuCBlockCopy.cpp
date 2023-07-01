//===-- MOSHuCBlockCopy.cpp - MOS Increment Decrement PHI --------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS pass to lower G_MEMCPY/G_MEMSET/G_MEMSET_INLINE
// opcodes to special HuC6280 variants prior to legalization, as information
// which is difficult to extract mid-legalization (whether operands point to
// opaque constants) is required.
//
//===----------------------------------------------------------------------===//

#include "MOSHuCBlockCopy.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSSubtarget.h"

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Transforms/Utils/SizeOpts.h"
#include <optional>

#define DEBUG_TYPE "mos-hucblockcopy"

using namespace llvm;

namespace {

class MOSHuCBlockCopy : public MachineFunctionPass {
public:
  static char ID;

  MOSHuCBlockCopy() : MachineFunctionPass(ID) {
    llvm::initializeMOSHuCBlockCopyPass(*PassRegistry::getPassRegistry());
    HuCIrqSafeBlockCopies = true;
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  // TODO: Make this configurable, ideally as an integer setting the maximum
  // transfer command limit.
  bool HuCIrqSafeBlockCopies;
};

} // namespace

static std::optional<MachineOperand>
getConstantOperand(MachineRegisterInfo &MRI, MachineInstr &StartMI,
                   int OpIdx, bool FullyConstant = false) {
  Register VReg = StartMI.getOperand(OpIdx).getReg();
  MachineInstr *MI;

  while ((MI = MRI.getVRegDef(VReg))) {
    switch (MI->getOpcode()) {
    case TargetOpcode::COPY:
      VReg = MI->getOperand(1).getReg();
      if (VReg.isPhysical())
        return std::nullopt;
      break;
    case TargetOpcode::G_INTTOPTR:
      VReg = MI->getOperand(1).getReg();
      break;
    case TargetOpcode::G_GLOBAL_VALUE:
    case TargetOpcode::G_CONSTANT: {
      auto Op = MI->getOperand(1);
      // MOSMCInstLower does not support CImmediates.
      if (Op.getType() == MachineOperand::MO_CImmediate) {
        return MachineOperand::CreateImm(Op.getCImm()->getZExtValue());    
      }
      if (FullyConstant && !Op.isImm())
        return std::nullopt;
      return MI->getOperand(1);
    }
    default:
      return std::nullopt;
    }
  }

  return std::nullopt;
}

static std::optional<uint64_t>
getUInt64FromConstantOper(MachineOperand &Operand) {
  if (Operand.isImm())
    return Operand.getImm();
  if (Operand.isCImm())
    return Operand.getCImm()->getZExtValue();
  return std::nullopt;
}

static MachineOperand offsetMachineOperand(MachineOperand Operand,
                                           int64_t Offset) {
  if (Operand.isImm())
    return MachineOperand::CreateImm(Operand.getImm() + Offset);
  if (Operand.isCImm())
    return MachineOperand::CreateCImm(Operand.getCImm() + Offset);
  if (Operand.isGlobal())
    return MachineOperand::CreateGA(Operand.getGlobal(),
                                    Operand.getOffset() + Offset);
  llvm_unreachable("Unsupported machine operand type!");
}

enum RegionOverlap {
  RO_Unknown,
  RO_MayOverlapBefore, /* Src < Dst */
  RO_MayOverlapAfter, /* Src > Dst */
  RO_DoesNotOverlap
};

bool MOSHuCBlockCopy::runOnMachineFunction(MachineFunction &MF) {
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  if (!STI.hasHUC6280())
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    for (auto I = MBB.begin(), E = MBB.end(); I != E; ++I) {
      MachineInstr &MI = *I;
      MachineInstr *MIPrev;

      std::optional<MachineOperand> Src = std::nullopt;
      Register DstReg = 0;
      std::optional<MachineOperand> Dst = std::nullopt;
      std::optional<MachineOperand> Len = std::nullopt;
      bool IsLoadStorePair = false;
      bool IsSet = false;
      RegionOverlap Overlap = RO_Unknown;
      uint8_t Descending = 0;
      
      // Match supported combinations.
      if (MI.getOpcode() == MOS::G_MEMCPY
        || MI.getOpcode() == MOS::G_MEMCPY_INLINE
        || MI.getOpcode() == MOS::G_MEMSET) {
        Register SrcReg = MI.getOperand(1).getReg();
        DstReg = MI.getOperand(0).getReg();
        Dst = getConstantOperand(MRI, MI, 0);
        Src = getConstantOperand(MRI, MI, 1);
        Len = getConstantOperand(MRI, MI, 2);
        Overlap = RO_DoesNotOverlap;
        
        if (MI.getOpcode() == MOS::G_MEMSET) {
          IsSet = true;

          if (MRI.getType(SrcReg).getSizeInBytes() != 1)
            continue;

          auto SrcValue = getUInt64FromConstantOper(Src.value());
          if (!SrcValue.has_value())
            continue;
        }
      } else if (MI.getOpcode() == MOS::G_MEMMOVE) {
        DstReg = MI.getOperand(0).getReg();
        Dst = getConstantOperand(MRI, MI, 0, true);
        Src = getConstantOperand(MRI, MI, 1, true);
        Len = getConstantOperand(MRI, MI, 2, true);
      } else if (MI.getOpcode() == MOS::G_STORE) {
        Register TmpReg = MI.getOperand(0).getReg();
        // Setting Dst/Len is safe here, as Src will only be set later.
        DstReg = MI.getOperand(1).getReg();
        Dst = getConstantOperand(MRI, MI, 1);
        Len = MachineOperand::CreateImm(MRI.getType(TmpReg)
                                           .getSizeInBytes());
        MIPrev = I->getPrevNode();
        // InstCombinePass combines 16/32/64-bit memcpy() calls into
        // a load/store pair; cover those.
        if (MIPrev && MIPrev->getOpcode() == MOS::G_LOAD
            && MIPrev->getOperand(0).getReg() == TmpReg) {
          if (MRI.getType(MIPrev->getOperand(1).getReg()).isPointer()) {
            Src = getConstantOperand(MRI, *MIPrev, 1);
          }
          IsLoadStorePair = true;
        } else {
          // This could also be a large constant store.
          Src = getConstantOperand(MRI, MI, 0);
          IsSet = true;
          if (!Src.has_value())
            continue;
          auto SrcValue = getUInt64FromConstantOper(Src.value());
          if (!SrcValue.has_value())
            continue;
          uint32_t SrcBytes = MRI.getType(MI.getOperand(0).getReg())
                                 .getSizeInBytes();
          uint32_t DstBytes = MRI.getType(DstReg).getSizeInBytes();
          if (SrcValue.value() != 0) {
            if (SrcBytes != DstBytes)
              continue;
            bool MismatchFound = false;
            // Support variants like 0x01010101, 0x02020202...
            for (uint32_t I = 1; I < SrcBytes; I++) {
              if (((SrcValue.value() >> (I * 8)) & 0xFF)
                  != (SrcValue.value() & 0xFF)) {
                MismatchFound = true;
                break;
              }
            }
            if (MismatchFound)
              continue;
          }
        }
      } else {
        continue;
      }

      if (!Src.has_value() || !Dst.has_value() || !Len.has_value())
        continue;

      if (IsSet) {
        // A TII-based memory set is always slower than the alternative.
        // Skip using it unless -Os, -Oz is set.
        if (!MF.getFunction().hasOptSize())
          continue;
      } else if (Overlap == RO_Unknown) {
        auto SrcValue = getUInt64FromConstantOper(Src.value());
        auto DstValue = getUInt64FromConstantOper(Dst.value());
        if (!SrcValue.has_value() || !DstValue.has_value())
          continue;
        
        Overlap = *SrcValue < *DstValue ? RO_MayOverlapBefore : RO_MayOverlapAfter;
      }

      Descending = Overlap == RO_MayOverlapBefore;

      // On HuC platforms, block copies can be emitted, and sets can be done
      // with them too. However, some requirements have to be considered:
      // 1) The source, destination, and length have to be constant; however,
      //    they can be opaque constants (such as symbols).
      // 2) A block copy instruction stalls all interrupts until it completes.
      //    As such, one instruction should only do some amount of transfers,
      //    to prevent stalling video interrupts mid-execution.
      // Each transfer is 7 bytes and (17 + 6n) cycles, where n is the length
      // of the transfer in bytes.
      uint64_t BytesPerTransfer = HuCIrqSafeBlockCopies ? 16 : UINT16_MAX;
      uint64_t SizeMin, SizeMax;
      // Note that non-indexed LDA/STA memory calls are 1 cycle slower on
      // HuC6280 compared to other 6502 derivatives.
      if (MF.getFunction().hasMinSize()) {
        // Copies:
        // => inline LDA/STA: 6n bytes
        // => TII: 7 bytes
        // => memcpy(): ~23 bytes
        // Sets:
        // => inline LDA/STA: 2 + 3n bytes
        // => (LDA/STA|STZ)/TII: 3-5 + 7 bytes
        // => __memset(): ~21? bytes
        SizeMin = IsSet ? 5 : 2;
        SizeMax = IsSet ? (BytesPerTransfer * 2 + 1)
                        : (BytesPerTransfer * 3);
      } else if (MF.getFunction().hasOptSize()) {
        // Try to strike a balance.
        SizeMin = IsSet ? 5 : 4;
        SizeMax = IsSet ? (BytesPerTransfer * 3 + 1)
                        : (BytesPerTransfer * 4);
      } else {
        // Copies:
        // => inline LDA/STA: 10n cycles
        // => TII: 17 + 6n cycles
        // Sets:
        // => inline LDA/STA: 2 + 5n cycles
        // => (LDA/STA|STZ)/TII: 22-24 + 6n cycles
        SizeMin = 5;
        SizeMax = BytesPerTransfer * 5;
      }
      uint64_t KnownLen = UINT16_MAX;

      // If we require IRQ-safe chunks, the length has to be known.
      auto LenValue = getUInt64FromConstantOper(Len.value());
      if (LenValue.has_value()) {
        KnownLen = LenValue.value();
        if (KnownLen < SizeMin || KnownLen > SizeMax) {
          continue;
        }
      } else if (BytesPerTransfer < UINT16_MAX || Descending) {
        continue;
      }

      // Proceed with the custom lowering.
      MachineIRBuilder Builder(MBB, MI);

      if (IsSet) {
        // Emit a G_STORE, then set Src = Dst, Dst = Dst + 1, Len = Len - 1.
        auto StoreReg = MRI.createGenericVirtualRegister(LLT::scalar(8));
        Builder.buildConstant(StoreReg,
                              getUInt64FromConstantOper(Src.value())
                                .value() & 0xFF);
        Builder.buildStore(StoreReg, DstReg,
                           *MF.getMachineMemOperand(MachinePointerInfo(),
                                                    MachineMemOperand::MOStore,
                                                    1, Align(1)));

        Src = Dst;
        Dst = offsetMachineOperand(Dst.value(), 1);
        Len = offsetMachineOperand(Len.value(), -1);
      }

      if (KnownLen <= BytesPerTransfer) {
        Builder.buildInstr(MOS::HuCMemcpy)
            .add(Src.value()).add(Dst.value()).add(Len.value())
            .addImm(Descending);
      } else {
        for (uint64_t TOfs = 0; TOfs < KnownLen; TOfs += BytesPerTransfer) {
          uint64_t TLen = std::min(KnownLen - TOfs, BytesPerTransfer);
          // Descending transfers must be done in backwards order.
          uint64_t AdjTOfs = Descending ? (KnownLen - TOfs - TLen) : TOfs;
          Builder.buildInstr(MOS::HuCMemcpy)
              .add(offsetMachineOperand(Src.value(), AdjTOfs))
              .add(offsetMachineOperand(Dst.value(), AdjTOfs))
              .add(MachineOperand::CreateImm(TLen))
              .addImm(Descending);
        }
      }

      --I;
      MI.eraseFromParent();
      if (IsLoadStorePair) {
        MIPrev->eraseFromParent();
      }
      
      Changed = true;
    }
  }

  return Changed;
}

char MOSHuCBlockCopy::ID = 0;

INITIALIZE_PASS_BEGIN(MOSHuCBlockCopy, DEBUG_TYPE,
                      "Emit HuC6280 block copies", false, false)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
INITIALIZE_PASS_END(MOSHuCBlockCopy, DEBUG_TYPE,
                    "Emit HuC6280 block copies", false, false)

MachineFunctionPass *llvm::createMOSHuCBlockCopyPass() {
  return new MOSHuCBlockCopy();
}
