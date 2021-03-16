//===-- MOS.cpp - MOS ToolChain -------------------------------------------===//
//
// Part of the LLVM-MOS Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOS.h"

using namespace clang::driver::toolchains;

MOS::MOS(const Driver &D, const llvm::Triple &Triple,
         const llvm::opt::ArgList &Args)
    : ToolChain(D, Triple, Args) {}
