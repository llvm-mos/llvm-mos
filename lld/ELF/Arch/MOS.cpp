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
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;
};
} // namespace

MOS::MOS() {
  defaultMaxPageSize = 1;
  defaultCommonPageSize = 1;
  noneRel = R_MOS_NONE;
}

RelExpr MOS::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  return R_ABS;
}

void MOS::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  switch (rel.type) {
  case R_MOS_IMM8:
  case R_MOS_ADDR8:
  case R_MOS_ADDR16_HI:
  case R_MOS_ADDR16_LO:
  case R_MOS_PCREL_8:
  case R_MOS_ADDR24_SEGMENT:
  case R_MOS_ADDR24_BANK_HI:
  case R_MOS_ADDR24_BANK_LO:
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MOS_ADDR16:
  case R_MOS_ADDR24_BANK:
    write16le(loc, static_cast<unsigned short>(val));
    break;
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
