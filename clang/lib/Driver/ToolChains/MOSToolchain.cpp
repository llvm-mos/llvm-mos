//===-- MOSToolchain.cpp - MOS ToolChain ----------------------------------===//
//
// Part of the LLVM-MOS Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSToolchain.h"

#include "CommonArgs.h"

#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/Options.h"
#include "llvm/Support/Path.h"

using namespace llvm::opt;
using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;

MOSToolChain::MOSToolChain(const Driver &D, const llvm::Triple &Triple,
                           const llvm::opt::ArgList &Args)
    : ToolChain(D, Triple, Args) {
  getProgramPaths().push_back(getDriver().Dir);
}

Tool *MOSToolChain::buildLinker() const { return new tools::mos::Linker(*this); }

void MOSToolChain::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                             ArgStringList &CC1Args) const {
  if (DriverArgs.hasArg(options::OPT_nostdinc))
    return;

  if (!DriverArgs.hasArg(options::OPT_nobuiltininc)) {
    SmallString<128> Dir(getDriver().ResourceDir);
    llvm::sys::path::append(Dir, "include");
    addSystemInclude(DriverArgs, CC1Args, Dir.str());
  }
}

void MOSToolChain::addClangTargetOptions(const ArgList &DriverArgs,
                                ArgStringList &CC1Args,
                                Action::OffloadKind) const {
  CC1Args.push_back("-nostdsysteminc");
  // Not yet implemented for GlobalISel.
  CC1Args.push_back("-fexperimental-assignment-tracking=disabled");
}

static bool hasLTOEmitAsm(const ArgList &Args) {
  for (Arg *A : Args) {
    if (!A->getOption().matches(options::OPT_Wl_COMMA) &&
        !A->getOption().matches(options::OPT_Xlinker))
      continue;
    if (A->containsValue("--lto-emit-asm"))
      return true;
  }
  return false;
}

void mos::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                               const InputInfo &Output,
                               const InputInfoList &Inputs, const ArgList &Args,
                               const char *LinkingOutput) const {
  ArgStringList CmdArgs;

  auto &TC = static_cast<const toolchains::MOSToolChain &>(getToolChain());
  auto &D = TC.getDriver();

  // Pass defaults before AddLinkerInputs, since that includes -Wl
  // options, which should override these.
  CmdArgs.push_back("--gc-sections");
  CmdArgs.push_back("--sort-section=alignment");

  AddLinkerInputs(TC, Inputs, Args, CmdArgs, JA);

  AddLTOOptions(TC, Args, Output, Inputs, CmdArgs);

  if (!D.SysRoot.empty())
    CmdArgs.push_back(Args.MakeArgString("--sysroot=" + D.SysRoot));

  TC.AddFilePathLibArgs(Args, CmdArgs);
  Args.addAllArgs(CmdArgs, {options::OPT_L, options::OPT_T_Group,
                            options::OPT_e, options::OPT_s, options::OPT_t,
                            options::OPT_Z_Flag, options::OPT_r});

  if (!Args.hasArg(options::OPT_nostartfiles, options::OPT_nostdlib)) {
    // Prefixing a colon causes GNU LD-like linkers to search for this filename
    // as-is. This contains the minimum necessary startup library.
    CmdArgs.push_back("-l:crt0.o");

    // libcrt0.a contains optional startup objects that are only pulled in if
    // referenced.
    CmdArgs.push_back("-lcrt0");
  }

  if (!Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib))
    CmdArgs.push_back("-lcrt");

  if (!Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nolibc,
                   options::OPT_nostdlib))
    CmdArgs.push_back("-lc");

  // No matter what's included in the link, the default linker script is
  // nonsense for the 6502. Accordingly, use one named "link.ld" if none is
  // specified.
  if (!Args.hasArg(options::OPT_T))
    CmdArgs.push_back("-Tlink.ld");

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  C.addCommand(std::make_unique<Command>(JA, *this, ResponseFileSupport::None(),
                                         Args.MakeArgString(TC.GetLinkerPath()),
                                         CmdArgs, Inputs, Output));
  if (!hasLTOEmitAsm(Args)) {
    for (StringRef PostLinkTool :
         Args.getAllArgValues(options::OPT_fpost_link_tool)) {
      ArgStringList PostLinkToolArgs;
      PostLinkToolArgs.push_back(
          Args.MakeArgString(Twine(Output.getFilename()) + ".elf"));

      std::string Path = PostLinkTool.str();
      if (!llvm::sys::fs::exists(Path))
        Path = getToolChain().GetProgramPath(Path.c_str());

      C.addCommand(std::make_unique<Command>(
          JA, *this, ResponseFileSupport::None(),
          Args.MakeArgString(Path.c_str()), PostLinkToolArgs, Inputs, Output));
    }
  }
}

void mos::Linker::AddLTOOptions(const toolchains::MOSToolChain &TC, const ArgList &Args,
                                const InputInfo &Output,
                                const InputInfoList &Inputs,
                                ArgStringList &CmdArgs) const {
  assert(!Inputs.empty() && "Must have at least one input.");
  addLTOOptions(TC, Args, CmdArgs, Output, Inputs[0],
                TC.getDriver().getLTOMode() == LTOK_Thin);
  addMOSCodeGenArgs(CmdArgs);
  unsigned ZPBytes = 0;
  StringRef LTOZP = Args.getLastArgValue(options::OPT_mlto_zp_EQ);
  if (!LTOZP.empty()) {
    if (LTOZP.getAsInteger(0, ZPBytes))
      TC.getDriver().Diag(diag::err_drv_invalid_zp) << LTOZP;
  }
  for (const std::string &Val :
       Args.getAllArgValues(options::OPT_mreserve_zp_EQ)) {
    if (Val.empty())
      continue;
    unsigned Amt;
    if (StringRef(Val).getAsInteger(0, Amt)) {
      TC.getDriver().Diag(diag::err_drv_invalid_zp) << Val;
      continue;
    }
    if (ZPBytes < Amt) {
      TC.getDriver().Diag(diag::err_drv_too_much_zp_reserved) << Val;
      continue;
    }
    ZPBytes -= Amt;
  }
  if (ZPBytes) {
    CmdArgs.push_back("-mllvm");
    CmdArgs.push_back(Args.MakeArgString("-zp-avail=" + Twine(ZPBytes)));
  }
}
