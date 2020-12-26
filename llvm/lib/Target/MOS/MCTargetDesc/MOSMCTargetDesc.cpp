//===-- MOSMCTargetDesc.cpp - MOS Target Descriptions ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides MOS specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "MOSELFStreamer.h"
#include "MOSMCAsmInfo.h"
#include "MOSMCELFStreamer.h"
#include "MOSMCTargetDesc.h"
#include "MOSTargetStreamer.h"
#include "InstPrinter/MOSInstPrinter.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

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
  InitMOSMCRegisterInfo(X, 0);

  return X;
}

static MCSubtargetInfo *createMOSMCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  return createMOSMCSubtargetInfoImpl(TT, CPU, FS);
}

static MCInstPrinter *createMOSMCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  if (SyntaxVariant == 0) {
    return new MOSInstPrinter(MAI, MII, MRI);
  }

  return nullptr;
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  return createELFStreamer(Context, std::move(MAB), std::move(OW),
                           std::move(Emitter), RelaxAll);
}

static MCTargetStreamer *
createMOSObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new MOSELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint,
                                                   bool isVerboseAsm) {
  return new MOSTargetAsmStreamer(S);
}

extern "C" void LLVMInitializeMOSTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<MOSMCAsmInfo> X(getTheMOSTarget());

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

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getTheMOSTarget(), createMOSMCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getTheMOSTarget(), createMCStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(getTheMOSTarget(),
                                               createMOSObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getTheMOSTarget(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getTheMOSTarget(), createMOSAsmBackend);
}

