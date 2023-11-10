//===- MOS.cpp ------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "InputFiles.h"
#include "Symbols.h"
#include "Target.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/BinaryFormat/MOSFlags.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Endian.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::support::endian;
using namespace llvm::ELF;

namespace lld {
namespace elf {

namespace {
class MOS final : public TargetInfo {
public:
  MOS();
  uint32_t calcEFlags() const override;
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;
};
} // namespace

MOS::MOS() {
  defaultMaxPageSize = 1;
  defaultCommonPageSize = 1;
}

static uint32_t getEFlags(InputFile *file) {
  return cast<ObjFile<ELF32LE>>(file)->getObj().getHeader().e_flags;
}

uint32_t MOS::calcEFlags() const {
  uint32_t outputFlags = 0;

  for (InputFile *f : ctx.objectFiles) {
    const uint32_t flags = getEFlags(f);
    if (!llvm::MOS::checkEFlagsCompatibility(flags, outputFlags)) {
      error("Input file '" + f->getName() +
            "' uses bad MOS "
            "feature combination from rest of output file.\n"
            "Input file: " +
            llvm::MOS::makeEFlagsString(flags) +
            "Output file: " + llvm::MOS::makeEFlagsString(outputFlags));
    }

    outputFlags |= flags;
  }

  return outputFlags;
}

RelExpr MOS::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  switch (type) {
  default:
    return R_ABS;
  case R_MOS_PCREL_8:
  case R_MOS_PCREL_16:
    return R_PC;
  }
}

// Note: The high two bytes of the 16-bit address space are often used for
// ancillary things like bank numbers, so these are not considered when checking
// validity of 16-bit or narrower addresses.
void MOS::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  switch (rel.type) {
  case R_MOS_IMM8:
    checkIntUInt(loc, val, 8, rel);
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MOS_IMM16:
    checkIntUInt(loc, val, 16, rel);
    write16le(loc, static_cast<unsigned short>(val));
    break;
  case R_MOS_PCREL_8:
    checkInt(loc, val - 1, 8, rel);
    // MOS's PC relative addressing is off by one from the standard LLVM PC
    // relative convention.
    *loc = static_cast<unsigned char>(val - 1);
    break;
  case R_MOS_PCREL_16:
    checkInt(loc, val - 2, 16, rel);
    // MOS's PC relative addressing is off by two from the standard LLVM PC
    // relative convention.
    write16le(loc, static_cast<unsigned short>(val - 2));
    break;
  case R_MOS_ADDR8:
    // The HuC6280's zero page is at 0x2000.
    if (config->eflags & ELF::EF_MOS_ARCH_HUC6280)
      val -= 0x2000;
    checkUInt(loc, val & 0xffff, 8, rel);
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MOS_ADDR13:
    checkInt(loc, val & 0xffff, 13, rel);
    write16le(loc, (read16le(loc) & ~0x1fff) | (val & 0x1fff));
    break;
  case R_MOS_ADDR16:
    write16le(loc, static_cast<unsigned short>(val));
    break;
  case R_MOS_ADDR16_LO:
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MOS_ADDR16_HI:
    *loc = static_cast<unsigned char>(val >> 8);
    break;
  case R_MOS_ADDR24:
    checkUInt(loc, val, 24, rel);
    write32le(loc, (read32le(loc) & ~0x00ffffff) | (val & 0x00ffffff));
    break;
  case R_MOS_ADDR24_SEGMENT:
    checkUInt(loc, val, 24, rel);
    write16le(loc, static_cast<unsigned short>(val));
    break;
  case R_MOS_ADDR24_SEGMENT_LO:
    checkUInt(loc, val, 24, rel);
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MOS_ADDR24_SEGMENT_HI:
    checkUInt(loc, val, 24, rel);
    *loc = static_cast<unsigned char>(val >> 8);
    break;
  case R_MOS_ADDR24_BANK:
    checkUInt(loc, val, 24, rel);
    *loc = static_cast<unsigned char>(val >> 16);
    break;
  case R_MOS_FK_DATA_4:
    write32le(loc, static_cast<unsigned long>(val));
    break;
  case R_MOS_FK_DATA_8:
    write64le(loc, static_cast<unsigned long long>(val));
    break;
  case R_MOS_ADDR_ASCIZ: {
    std::string valueStr = utostr(val);
    assert(valueStr.size() <= 8 && "R_MOS_ADDR_ASCIZ string too big!");
    std::copy(valueStr.begin(), valueStr.end(), loc);
    loc[valueStr.size()] = '\0';
    break;
  }
  default:
    error(getErrorLocation(loc) + "unrecognized relocation " +
          toString(rel.type));
  }
}

TargetInfo *getMOSTargetInfo() {
  static MOS target;
  return &target;
}

} // namespace elf
} // namespace lld
