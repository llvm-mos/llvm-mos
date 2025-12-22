//===-- MOSMCTargetDesc.cpp - MOS Target Descriptions ---------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MOS specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "MOSMCTargetDesc.h"
#include "MOSInstPrinter.h"
#include "MOSMCAsmInfo.h"
#include "MOSMCELFStreamer.h"
#include "MOSMCInstrAnalysis.h"
#include "MOSTargetStreamer.h"
#include "TargetInfo/MOSTargetInfo.h"

#include "llvm/ADT/SmallString.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "MOSGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "MOSGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "MOSGenRegisterInfo.inc"

using namespace llvm;

MCInstrInfo *llvm::createMOSMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitMOSMCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createMOSMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitMOSMCRegisterInfo(X, MOS::PC);
  return X;
}

/// Emit a DWARF expression that computes a normalized hardware stack address.
/// The 6502 hardware stack is at 0x0100-0x01FF. The S register is physically
/// 8-bit, but some debuggers report it as 16-bit (e.g., MAME reports 0x01FE).
/// This expression normalizes to always produce addresses in 0x0100-0x01FF:
///   result = ((S + Offset) & 0xFF) | 0x0100
static void emitNormalizedHardwareStackExpr(SmallVectorImpl<char> &Expr,
                                             unsigned DwarfS,
                                             int8_t Offset) {
  // DW_OP_breg<S> <offset> - S + offset
  Expr.push_back(static_cast<char>(dwarf::DW_OP_breg0 + DwarfS));
  Expr.push_back(static_cast<char>(Offset));

  // DW_OP_const1u 0xFF
  Expr.push_back(static_cast<char>(dwarf::DW_OP_const1u));
  Expr.push_back(static_cast<char>(0xFF));

  // DW_OP_and - keep low byte only
  Expr.push_back(static_cast<char>(dwarf::DW_OP_and));

  // DW_OP_const2u 0x0100 (little-endian: 0x00, 0x01)
  Expr.push_back(static_cast<char>(dwarf::DW_OP_const2u));
  Expr.push_back(static_cast<char>(0x00));
  Expr.push_back(static_cast<char>(0x01));

  // DW_OP_or - force into hardware stack page
  Expr.push_back(static_cast<char>(dwarf::DW_OP_or));
}

static MCAsmInfo *createMOSMCAsmInfo(const MCRegisterInfo &MRI,
                                      const Triple &TT,
                                      const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new MOSMCAsmInfo(TT, Options);

  // Initialize initial frame state for hardware stack.
  // 6502 JSR pushes (return_address - 1) onto hardware stack:
  //   - First pushes high byte to [SP], then decrements SP
  //   - Then pushes low byte to [SP], then decrements SP
  // After JSR, SP points to next free slot (two below pushed address).
  // Stack layout after JSR:
  //   [SP+1] = low byte of (return_address - 1)
  //   [SP+2] = high byte of (return_address - 1)
  //
  // We use normalized expressions to handle both 8-bit and 16-bit S values.
  // Each hardware stack address is computed independently (not CFA-relative)
  // to avoid subtraction underflow when S wraps within the stack page.
  // This must match ABISysV_mos::CreateFunctionEntryUnwindPlan().

  unsigned DwarfS = MRI.getDwarfRegNum(MOS::S, true);
  unsigned DwarfPC = MRI.getDwarfRegNum(MOS::PC, true);

  // CFA = normalized(S + 3) = ((S + 3) & 0xFF) | 0x0100
  SmallString<16> CfaExpr;
  emitNormalizedHardwareStackExpr(CfaExpr, DwarfS, 3);
  MCCFIInstruction CfaInst =
      MCCFIInstruction::createDefCfaExpression(nullptr, CfaExpr);
  MAI->addInitialFrameState(CfaInst);

  // PC = [normalized(S + 1)] - direct expression, NOT [CFA - 2]
  // The return address is at S+1 (low byte) and S+2 (high byte).
  SmallString<16> PcExpr;
  emitNormalizedHardwareStackExpr(PcExpr, DwarfS, 1);
  MCCFIInstruction PcInst =
      MCCFIInstruction::createExpression(nullptr, DwarfPC, PcExpr);
  MAI->addInitialFrameState(PcInst);

  return MAI;
}

static MCSubtargetInfo *createMOSMCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  // If we've received no advice on which CPU to use, let's use our own default.
  if (CPU.empty()) {
    CPU = "mos6502";
  }
  return createMOSMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createMOSMCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  switch (SyntaxVariant) {
  case 0:
    return new MOSInstPrinter(MAI, MII, MRI);
    break;
  case 1:
    return new MOSInstPrinterCA65(MAI, MII, MRI);
    break;
  case 2:
    return new MOSInstPrinterXA65(MAI, MII, MRI);
    break;
  default:
    return nullptr;
  }
}

static MCTargetStreamer *
createMOSObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new MOSTargetELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint) {
  return new MOSTargetAsmStreamer(S, OS);
}

static MCInstrAnalysis *createMOSMCInstrAnalysis(const MCInstrInfo *Info) {
  return new MOSMCInstrAnalysis(Info);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMOSTargetMC() {
  // Register the MC asm info with initial frame state for CFI.
  TargetRegistry::RegisterMCAsmInfo(getTheMOSTarget(), createMOSMCAsmInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(getTheMOSTarget(), createMOSMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(getTheMOSTarget(), createMOSMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(getTheMOSTarget(),
                                          createMOSMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(getTheMOSTarget(),
                                        createMOSMCInstPrinter);

  // Register the MC instruction analyzer.
  TargetRegistry::RegisterMCInstrAnalysis(getTheMOSTarget(),
                                          createMOSMCInstrAnalysis);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getTheMOSTarget(),
                                        createMOSMCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getTheMOSTarget(),
                                      createMOSMCELFStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(getTheMOSTarget(),
                                               createMOSObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getTheMOSTarget(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getTheMOSTarget(), createMOSAsmBackend);
}

constexpr StringRef ZPPrefixes[] = {
    ".zp",
    ".zeropage",
    ".directpage",
};

bool MOS::isZeroPageSectionName(StringRef Name) {
  if (Name.empty())
    return false;
  for (StringRef Prefix : ZPPrefixes)
    if (Name.starts_with(Prefix) &&
        (Name.size() == Prefix.size() || Name[Prefix.size()] == '.'))
      return true;
  return false;
}
