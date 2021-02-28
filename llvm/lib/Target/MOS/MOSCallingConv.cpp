//===-- MOSCallingConv.cpp - MOS Calling Convention ------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS calling convention.
//
//===----------------------------------------------------------------------===//

#include "MOSCallingConv.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/IR/DataLayout.h"

#include "MOSGenCallingConv.inc"
