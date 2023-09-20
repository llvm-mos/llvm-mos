//===-- MOS.h - Top-level interface for MOS representation ------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// MOS back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOS_H
#define LLVM_LIB_TARGET_MOS_MOS_H

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Pass.h"

namespace llvm {

void initializeMOSCombinerPass(PassRegistry &);
void initializeMOSCopyOptPass(PassRegistry &);
void initializeMOSIncDecPhiPass(PassRegistry &);
void initializeMOSIndexIVPass(PassRegistry &);
void initializeMOSInsertCopiesPass(PassRegistry &);
void initializeMOSLateOptimizationPass(PassRegistry &);
void initializeMOSLowerSelectPass(PassRegistry &);
void initializeMOSNonReentrantPass(PassRegistry &);
void initializeMOSPostRAScavengingPass(PassRegistry &);
void initializeMOSStaticStackAllocPass(PassRegistry &);
void initializeMOSZeroPageAllocPass(PassRegistry &);

// The behind-by-one property of the std::reverse_iterator adaptor applied by
// reverse() does not properly handle instruction erasures. This range construct
// converts the forward iterators to native reverse iterators that are not
// behind-by-one and therefore handle erasures correctly when combined with
// make_early_inc_range().
inline auto mbb_reverse(MachineBasicBlock::iterator Begin,
                        MachineBasicBlock::iterator End) {
  return make_range(MachineBasicBlock::reverse_iterator(End),
                    MachineBasicBlock::reverse_iterator(Begin));
}
template <typename ContainerTy> inline auto mbb_reverse(ContainerTy &&C) {
  return mbb_reverse(C.begin(), C.end());
}

namespace MOS {

/// An integer that identifies all of the supported MOS address spaces.
enum AddressSpace {
  MemoryAS,
  ZeroPageAS,
  NumAddrSpaces,
};

template <typename T> AddressSpace getAddressSpace(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  unsigned AS = PT->getAddressSpace();
  if (AS < NumAddrSpaces)
    return static_cast<AddressSpace>(AS);
  return NumAddrSpaces;
}

}

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOS_H
