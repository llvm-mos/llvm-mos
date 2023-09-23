//===-- MOSFlags.cpp - MOS ELF e_flags tools ---------------------*- C++-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/BinaryFormat/MOSFlags.h"
#include "llvm/BinaryFormat/ELF.h"

namespace llvm {
namespace MOS {

#define ENUM_ENT(enum, altName)                                                \
  { #enum, altName, ELF::enum }

static const EnumEntry<unsigned> ElfHeaderMOSFlagsEntries[] = {
    ENUM_ENT(EF_MOS_ARCH_6502, "mos6502"),
    ENUM_ENT(EF_MOS_ARCH_6502_BCD, "mos6502bcd"),
    ENUM_ENT(EF_MOS_ARCH_6502X, "mos6502x"),
    ENUM_ENT(EF_MOS_ARCH_65C02, "mos65c02"),
    ENUM_ENT(EF_MOS_ARCH_R65C02, "mosr65c02"),
    ENUM_ENT(EF_MOS_ARCH_W65C02, "mosw65c02"),
    ENUM_ENT(EF_MOS_ARCH_65CE02, "mos65ce02"),
    ENUM_ENT(EF_MOS_ARCH_W65816, "mosw65816"),
    ENUM_ENT(EF_MOS_ARCH_65EL02, "mos65el02"),
    ENUM_ENT(EF_MOS_ARCH_HUC6280, "moshuc6280"),
    ENUM_ENT(EF_MOS_ARCH_SWEET16, "mossweet16"),
    ENUM_ENT(EF_MOS_ARCH_65DTV02, "mos65dtv02"),
    ENUM_ENT(EF_MOS_ARCH_4510, "mos4510"),
    ENUM_ENT(EF_MOS_ARCH_45GS02, "mos45gs02"),
    ENUM_ENT(EF_MOS_ARCH_SPC700, "mosspc700")};
const ArrayRef<EnumEntry<unsigned>> ElfHeaderMOSFlags{ElfHeaderMOSFlagsEntries};

std::string makeEFlagsString(unsigned EFlags) {
  std::string Str;
  raw_string_ostream Stream(Str);
  ScopedPrinter Printer(Stream);
  Printer.printFlags("Flags", EFlags, ElfHeaderMOSFlags);
  Stream.flush();
  return Str;
}

bool checkEFlagsCompatibility(unsigned EFlags, unsigned ModuleEFlags) {
  const unsigned Flags = EFlags | ModuleEFlags;
  // Mixing sweet16 with native is prohibited
  if ((Flags & ELF::EF_MOS_ARCH_SWEET16) &&
      (Flags & ~ELF::EF_MOS_ARCH_SWEET16))
    return false;
  // Mixing SPC700 with native is prohibited
  if ((Flags & ELF::EF_MOS_ARCH_SPC700) &&
      (Flags & ~ELF::EF_MOS_ARCH_SPC700))
    return false;
  return true;
}

} // namespace MOS
} // namespace llvm
