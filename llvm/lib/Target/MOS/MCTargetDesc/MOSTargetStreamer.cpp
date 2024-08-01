//===-- MOSTargetStreamer.cpp - MOS Target Streamer Methods ---------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MOS specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "MOSTargetStreamer.h"

#include "MOSMCELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

namespace llvm {

MOSTargetStreamer::MOSTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void MOSTargetStreamer::finish() {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  if (hasBSS())
    stronglyReference("__do_zero_bss",
                      "Declaring this symbol tells the CRT that there is "
                      "something in .bss, so it may need to be zeroed.");

  if (hasZPBSS())
    stronglyReference("__do_zero_zp_bss",
                      "Declaring this symbol tells the CRT that there is "
                      "something in .zp.bss, so it may need to be zeroed.");

  if (hasData())
    stronglyReference(
        "__do_copy_data",
        "Declaring this symbol tells the CRT that there is something in .data, "
        "so it may need to be copied from LMA to VMA.");

  if (hasZPData())
    stronglyReference(
        "__do_copy_zp_data",
        "Declaring this symbol tells the CRT that there is something in "
        ".zp.data, so it may need to be copied from LMA to VMA.");

  if (hasInitArray())
    stronglyReference("__do_init_array",
                      "Declaring this symbol tells the CRT that there are "
                      "initialization routines to be run in .init_array");

  if (hasFiniArray())
    stronglyReference("__do_fini_array",
                      "Declaring this symbol tells the CRT that there are "
                      "finalization routines to be run in .fini_array");

  bool ReferencesStackPtr = llvm::any_of(
      Context.getSymbols(),
      [](const StringMapEntry<MCSymbolTableValue> &TableEntry) {
        return TableEntry.getKey() == "__rc0" || TableEntry.getKey() == "__rc1";
      });
  if (ReferencesStackPtr)
    stronglyReference("__do_init_stack",
                      "Declaring this symbol tells the CRT that the stack "
                      "pointer needs to be initialized.");
}

void MOSTargetStreamer::stronglyReference(StringRef Name, StringRef Comment) {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();
  MCSymbol *Sym = Context.getOrCreateSymbol(Name);
  OS.emitRawComment(Comment);
  stronglyReference(Sym);
}

static bool HasPrefix(StringRef Name, StringRef Prefix) {
  SmallString<32> PrefixDot = Prefix;
  PrefixDot += ".";
  return Name == Prefix || Name.starts_with(PrefixDot);
}

void MOSTargetAsmStreamer::changeSection(const MCSection *CurSection,
                                         MCSection *Section,
                                         uint32_t SubSection, raw_ostream &OS) {
  MCTargetStreamer::changeSection(CurSection, Section, SubSection, OS);
  HasBSS |= HasPrefix(Section->getName(), ".bss");
  HasZPBSS |= HasPrefix(Section->getName(), ".zp.bss");
  HasData |= HasPrefix(Section->getName(), ".data");
  HasZPData |= HasPrefix(Section->getName(), ".zp.data");
  HasZPData |= HasPrefix(Section->getName(), ".zp.rodata");
  HasInitArray |= HasPrefix(Section->getName(), ".init_array");
  HasFiniArray |= HasPrefix(Section->getName(), ".fini_array");
}

void MOSTargetAsmStreamer::stronglyReference(MCSymbol *Sym) {
  getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
}

/// Makes an e_flags value based on subtarget features.
static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned ELFArch = 0;
  if (Features[MOS::Feature65C02])
    ELFArch |= ELF::EF_MOS_ARCH_65C02;
  if (Features[MOS::Feature65CE02])
    ELFArch |= ELF::EF_MOS_ARCH_65CE02;
  if (Features[MOS::Feature65EL02])
    ELFArch |= ELF::EF_MOS_ARCH_65EL02;
  if (Features[MOS::Feature6502])
    ELFArch |= ELF::EF_MOS_ARCH_6502;
  if (Features[MOS::Feature6502BCD])
    ELFArch |= ELF::EF_MOS_ARCH_6502_BCD;
  if (Features[MOS::Feature6502X])
    ELFArch |= ELF::EF_MOS_ARCH_6502X;
  if (Features[MOS::FeatureR65C02])
    ELFArch |= ELF::EF_MOS_ARCH_R65C02;
  if (Features[MOS::FeatureSWEET16])
    ELFArch |= ELF::EF_MOS_ARCH_SWEET16;
  if (Features[MOS::FeatureW65C02])
    ELFArch |= ELF::EF_MOS_ARCH_W65C02;
  if (Features[MOS::FeatureW65816])
    ELFArch |= ELF::EF_MOS_ARCH_W65816;
  if (Features[MOS::FeatureHUC6280])
    ELFArch |= ELF::EF_MOS_ARCH_HUC6280;
  if (Features[MOS::Feature65DTV02])
    ELFArch |= ELF::EF_MOS_ARCH_65DTV02;
  if (Features[MOS::Feature4510])
    ELFArch |= ELF::EF_MOS_ARCH_4510;
  if (Features[MOS::Feature45GS02])
    ELFArch |= ELF::EF_MOS_ARCH_45GS02;
  if (Features[MOS::FeatureSPC700])
    ELFArch |= ELF::EF_MOS_ARCH_SPC700;
  return ELFArch;
}

MOSTargetELFStreamer::MOSTargetELFStreamer(MCStreamer &S,
                                           const MCSubtargetInfo &STI)
    : MOSTargetStreamer(S) {
  ELFObjectWriter &W = getStreamer().getWriter();
  unsigned EFlags = W.getELFHeaderEFlags();
  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());
  W.setELFHeaderEFlags(EFlags);
}

bool MOSTargetELFStreamer::hasBSS() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasBSS();
}
bool MOSTargetELFStreamer::hasZPBSS() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasZPBSS();
}
bool MOSTargetELFStreamer::hasData() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasData();
}
bool MOSTargetELFStreamer::hasZPData() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasZPData();
}
bool MOSTargetELFStreamer::hasInitArray() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasInitArray();
}
bool MOSTargetELFStreamer::hasFiniArray() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasFiniArray();
}

bool MOSTargetELFStreamer::emitDirectiveZeroPage(MCSymbol *Sym) {
  cast<MCSymbolELF>(Sym)->setOther(ELF::STO_MOS_ZEROPAGE);
  return true;
}

void MOSTargetELFStreamer::stronglyReference(MCSymbol *Sym) {
  auto *ES = cast<MCSymbolELF>(Sym);
  // There's an explicit check in emitSymbolAttribute to avoid accidentally
  // overriding weak->global due to a GCC corner case, but it should always be
  // safe for symbols under complete compiler control.
  if (ES->isBindingSet())
    ES->setBinding(ELF::STB_GLOBAL);
  else
    getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
}

} // end namespace llvm
