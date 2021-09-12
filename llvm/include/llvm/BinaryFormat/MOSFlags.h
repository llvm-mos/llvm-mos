//===- MOSFlags.h - MOS ELF e_flags tools -----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This header contains shared EF_MOS_* information and verification tools for
// targeting MOS.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_BINARYFORMAT_MOSFLAGS_H
#define LLVM_BINARYFORMAT_MOSFLAGS_H

#include "llvm/Support/ScopedPrinter.h"

namespace llvm {

class FeatureBitset;

namespace MOS {

/// EnumEntries for general EF_MOS_* printing.
extern const ArrayRef<EnumEntry<unsigned>> ElfHeaderMOSFlags;

/// Makes a string describing all set EF_MOS_* bits.
std::string makeEFlagsString(unsigned EFlags);

/// Checks if functions generated for a subtarget are compatible with e_flags
/// combinations in a module under assembly.
bool checkEFlagsCompatibility(unsigned EFlags, unsigned ModuleEFlags);

} // namespace MOS
} // namespace llvm

#endif // LLVM_BINARYFORMAT_MOSFLAGS_H
