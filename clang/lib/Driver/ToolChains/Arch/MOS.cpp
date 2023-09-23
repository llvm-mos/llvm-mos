//===--- MOS.cpp - MOS Helpers for Tools ------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOS.h"
#include "clang/Driver/Driver.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang::driver::tools;
using namespace llvm::opt;
using namespace llvm;

/// getMOSTargetCPU - Get the (LLVM) name of the MOS cpu we are targeting.
std::string mos::getMOSTargetCPU(const ArgList &Args) {
  if (Arg *A = Args.getLastArg(clang::driver::options::OPT_mcpu_EQ)) {
    StringRef CPUName = A->getValue();

    return llvm::StringSwitch<const char *>(CPUName)
        .Cases("mos6502", "6502", "mos6502")
        .Cases("mos6502x", "6502x", "mos6502x")
        .Cases("mos65c02", "65c02", "mos65c02")
        .Cases("mos65ce02", "65ce02", "mos65ce02")
        .Cases("mosr65c02", "r65c02", "mosr65c02")
        .Cases("mosw65c02", "w65c02", "mosw65c02")
        .Cases("mosw65816", "w65816", "mosw65816")
        .Cases("mos65el02", "65el02", "mos65el02")
        .Cases("moshuc6280", "huc6280", "moshuc6280")
        .Cases("mossweet16", "sweet16", "mossweet16")
        .Cases("mos65dtv02", "65dtv02", "mos65dtv02")
        .Cases("mos4510", "4510", "mos4510")
        .Cases("mos45gs02", "45gs02", "mos45gs02")
        .Cases("mosspc700", "spc700", "mosspc700")
        .Default("");
  }

  return "";
}

void mos::getMOSTargetFeatures(const Driver &D, const ArgList &Args,
                               std::vector<StringRef> &Features) {
  if (Args.hasArg(clang::driver::options::OPT_mcpu_EQ) &&
      getMOSTargetCPU(Args).empty()) {
    D.Diag(diag::err_drv_clang_unsupported)
        << Args.getLastArg(clang::driver::options::OPT_mcpu_EQ)
               ->getAsString(Args);
  }

  if (Arg *A = Args.getLastArg(options::OPT_fstatic_stack,
                               options::OPT_fno_static_stack)) {
    if (A->getOption().matches(options::OPT_fstatic_stack))
      Features.push_back("+static-stack");
    else
      Features.push_back("-static-stack");
  }
}
