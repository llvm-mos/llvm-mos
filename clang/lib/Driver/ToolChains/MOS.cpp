//===-- MOS.cpp - MOS ToolChain -------------------------------------------===//
//
// Part of the LLVM-MOS Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOS.h"

#include "CommonArgs.h"
#include "InputInfo.h"

#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/Options.h"
#include "llvm/Support/Path.h"

using namespace llvm::opt;
using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;

MOS::MOS(const Driver &D, const llvm::Triple &Triple,
         const llvm::opt::ArgList &Args)
    : ToolChain(D, Triple, Args) {
  // Look for binaries in both the installation and driver directory.
  getProgramPaths().push_back(getDriver().getInstalledDir());
  if (getDriver().getInstalledDir() != getDriver().Dir)
    getProgramPaths().push_back(getDriver().Dir);
}

Tool *MOS::buildLinker() const { return new tools::mos::Linker(*this); }

void MOS::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                    ArgStringList &CC1Args) const {
  if (DriverArgs.hasArg(options::OPT_nostdinc))
    return;

  if (!DriverArgs.hasArg(options::OPT_nobuiltininc)) {
    SmallString<128> Dir(getDriver().ResourceDir);
    llvm::sys::path::append(Dir, "include");
    addSystemInclude(DriverArgs, CC1Args, Dir.str());
  }
}

void MOS::addClangTargetOptions(const ArgList &DriverArgs,
                                ArgStringList &CC1Args,
                                Action::OffloadKind) const {
  CC1Args.push_back("-nostdsysteminc");
}

void mos::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                               const InputInfo &Output,
                               const InputInfoList &Inputs, const ArgList &Args,
                               const char *LinkingOutput) const {
  ArgStringList CmdArgs;

  auto &TC = static_cast<const toolchains::MOS &>(getToolChain());
  auto &D = TC.getDriver();

  AddLinkerInputs(TC, Inputs, Args, CmdArgs, JA);

  AddLTOOptions(TC, Args, Output, Inputs, CmdArgs);

  if (!D.SysRoot.empty())
    CmdArgs.push_back(Args.MakeArgString("--sysroot=" + D.SysRoot));

  TC.AddFilePathLibArgs(Args, CmdArgs);
  Args.AddAllArgs(CmdArgs, {options::OPT_L, options::OPT_T_Group,
                            options::OPT_e, options::OPT_s, options::OPT_t,
                            options::OPT_Z_Flag, options::OPT_r});

  if (!Args.hasArg(options::OPT_nostartfiles, options::OPT_nostdlib)) {
    // Prefixing a colon causes GNU LD-like linkers to search for this filename
    // as-is.
    CmdArgs.push_back("-l:crt0.o");
  }

  if (!Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib))
    CmdArgs.push_back("-lcrt");

  if (!Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nolibc,
                   options::OPT_nostdlib))
    CmdArgs.push_back("-lc");

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  C.addCommand(std::make_unique<Command>(JA, *this, ResponseFileSupport::None(),
                                         Args.MakeArgString(TC.GetLinkerPath()),
                                         CmdArgs, Inputs, Output));
}

void mos::Linker::AddLTOOptions(const toolchains::MOS &TC, const ArgList &Args,
                                const InputInfo &Output,
                                const InputInfoList &Inputs,
                                ArgStringList &CmdArgs) const {
  assert(!Inputs.empty() && "Must have at least one input.");
  addLTOOptions(TC, Args, CmdArgs, Output, Inputs[0],
                TC.getDriver().getLTOMode() == LTOK_Thin);
  addMOSCodeGenArgs(CmdArgs);
}
