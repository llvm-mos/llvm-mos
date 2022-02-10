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

  for (InputFile *f : objectFiles) {
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

void MOS::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  switch (rel.type) {
  case R_MOS_IMM8:
  case R_MOS_ADDR8:
  case R_MOS_ADDR16_LO:
  case R_MOS_ADDR24_SEGMENT_LO:
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MOS_PCREL_8:
    // MOS's PC relative addressing is off by one from the standard LLVM PC
    // relative convention.
    *loc = static_cast<unsigned char>(val - 1);
    break;
  case R_MOS_ADDR16_HI:
  case R_MOS_ADDR24_SEGMENT_HI:
    *loc = static_cast<unsigned char>(val >> 8);
    break;
  case R_MOS_ADDR24_BANK:
    *loc = static_cast<unsigned char>(val >> 16);
    break;
  case R_MOS_ADDR16:
  case R_MOS_ADDR24_SEGMENT:
    write16le(loc, static_cast<unsigned short>(val));
    break;
  case R_MOS_PCREL_16:
    // MOS's PC relative addressing is off by two from the standard LLVM PC
    // relative convention.
    write16le(loc, static_cast<unsigned short>(val - 2));
    break;
  case R_MOS_ADDR24:
    checkInt(loc, val, 24, rel);
    write32le(loc, (read32le(loc) & ~0x00ffffff) | (val & 0x00ffffff));
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
