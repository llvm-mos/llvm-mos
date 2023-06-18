//===-- llvm-mlb.cpp - Mesen label file generator ------------------------===//
//
// Part of the LLVM-MOS Project, under the Apache License v2.0 with LLVM
// Exceptions. See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This program is a utility that generates a Mesen label file (.lbl) from the
// ELF output of a LLVM-MOS-SDK NES target build.
//
//===----------------------------------------------------------------------===//

#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Object/ELFObjectFile.h"
#include "llvm/Object/ELFTypes.h"
#include "llvm/Object/ObjectFile.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/InitLLVM.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Regex.h"
#include "llvm/Support/WithColor.h"
#include <system_error>

using namespace llvm;
using namespace llvm::object;

namespace {
  static cl::OptionCategory MLBCategory("MLB Options");

  enum class MLBPlatform {
    Auto, NES, PCE
  };

  cl::opt<MLBPlatform> Platform(
      "platform", cl::desc(""),
      cl::init(MLBPlatform::Auto),
      cl::values(clEnumValN(MLBPlatform::Auto, "Auto",
                            "Detect automatically based on filename."),
                 clEnumValN(MLBPlatform::NES, "NES", "NES"),
                 clEnumValN(MLBPlatform::PCE, "PCE", "PCE")),
      cl::NotHidden,
      cl::cat(MLBCategory));

  enum class MLBOutputFormat {
    MLB
  };

  cl::opt<MLBOutputFormat> OutputFormat(
      "format", cl::desc(""),
      cl::init(MLBOutputFormat::MLB),
      cl::values(clEnumValN(MLBOutputFormat::MLB, "MLB", "Mesen Label File.")),
      cl::NotHidden,
      cl::cat(MLBCategory));

  cl::list<std::string> ClInputFilenames(cl::Positional, cl::OneOrMore,
                                    cl::desc("<input ELF files>"),
                                    cl::cat(MLBCategory));

  cl::opt<std::string> ClOutputFilename("o", cl::init(""),
                                      cl::desc("Override output filename"),
                                      cl::value_desc("filename"),
                                      cl::cat(MLBCategory));
}

static void reportWarning(StringRef File, const Twine &Message) {
  outs().flush();
  WithColor::warning(errs(), "llvm-mlb")
      << "'" << File << "': " << Message << "\n";
}

[[noreturn]] static void reportError(StringRef File, const Twine &Message) {
  outs().flush();
  WithColor::error(errs(), "llvm-mlb")
      << "'" << File << "': " << Message << "\n";
  exit(1);
}

[[noreturn]] static void reportError(Error E, StringRef File) {
  outs().flush();
  WithColor::error(errs(), "llvm-mlb")
      << "'" << File << "': " << std::move(E) << "\n";
  exit(1);
}

template <typename T, typename... Ts>
static T unwrapOrError(Expected<T> EO, Ts &&...Args) {
  if (EO)
    return std::move(*EO);
  reportError(EO.takeError(), std::forward<Ts>(Args)...);
}

static Regex PRGROMRegex("^__prg_rom(.*)_(lma|offset)$");
static Regex ROMRegex("^__rom(.*)_(lma|offset)$");

int main(int argc, char **argv) {
  InitLLVM X(argc, argv);

  cl::HideUnrelatedOptions({&MLBCategory, &getColorCategory()});
  cl::ParseCommandLineOptions(argc, argv, "label file export tool\n");

  for (auto &InputFilename : ClInputFilenames) {
    OwningBinary<Binary> OBinary =
        unwrapOrError(createBinary(InputFilename), InputFilename);
    Binary &Binary = *OBinary.getBinary();

    auto *O = dyn_cast<ELF32LEObjectFile>(&Binary);
    if (!O)
      reportError(InputFilename, "expected an ELF object file");
    const ELFFile<ELF32LE> &ELFFile = O->getELFFile();

    StringRef OutputBase = InputFilename;
    OutputBase.consume_back(".elf");
    bool IsInputExtNES = OutputBase.consume_back(".nes");
    bool IsInputExtPCE = OutputBase.consume_back(".pce");
    if (Platform == MLBPlatform::Auto) {
      // Try detecting by filename extension.
      Platform = IsInputExtNES ? MLBPlatform::NES
        : (IsInputExtPCE ? MLBPlatform::PCE
        : MLBPlatform::Auto);

      if (Platform == MLBPlatform::Auto) {
        // When all else fails, assume NES for now.
        Platform = MLBPlatform::NES;
      }
    }
    std::string OutputFilename = ClOutputFilename;
    if (OutputFilename.empty()) {
      OutputFilename = OutputBase;
      OutputFilename += ".mlb";
    }
    std::error_code EC;
    raw_fd_ostream OS(OutputFilename, EC);
    if (EC)
      reportError(OutputFilename, Twine("cannot open: ") + EC.message());
    
    const auto BoundsStr = [](uint64_t Address, uint16_t Size) {
      auto UpperBoundStr = Size > 1 ? formatv("-{0:x-}", Address + Size - 1)
                                    : SmallString<10>();
      return formatv("{0:x-}{1}", Address, UpperBoundStr).sstr<10>();
    };

    if (Platform == MLBPlatform::NES) {
      DenseMap<uint64_t, SmallVector<const ELF32LE::Phdr *>> PRGSegsByOffset;
      for (const auto &Seg :
          unwrapOrError(ELFFile.program_headers(), InputFilename))
        if (Seg.p_paddr >= 0x03000000 && Seg.p_paddr < 0x04000000)
          PRGSegsByOffset[Seg.p_offset].push_back(&Seg);

      char PRGRAMType = 'W';
      std::map<uint32_t, StringRef> PRGROMLMABanks;
      DenseMap<StringRef, uint32_t> PRGROMBankOffsets;
      SmallVector<StringRef> Matches;
      for (const ELFSymbolRef Sym : O->symbols()) {
        StringRef Name = unwrapOrError(Sym.getName(), O->getFileName());
        if (Name == "__prg_nvram_size") {
          if (unwrapOrError(Sym.getValue(), O->getFileName()))
            PRGRAMType = 'S';
        } else if (PRGROMRegex.match(Name, &Matches)) {
          uint32_t Value = unwrapOrError(Sym.getValue(), O->getFileName());
          if (Matches[2] == "lma") {
            PRGROMLMABanks[Value] = Matches[1];
          } else {
            assert(Matches[2] == "offset");
            PRGROMBankOffsets[Matches[1]] = Value;
          }
        }
      }

      for (const ELFSymbolRef Sym : O->symbols()) {
        SymbolRef::Type Type = unwrapOrError(Sym.getType(), InputFilename);
        uint64_t Address = unwrapOrError(Sym.getAddress(), InputFilename);
        StringRef Name = unwrapOrError(Sym.getName(), InputFilename);
        uint64_t Size = Sym.getSize();

        if (Type == SymbolRef::ST_File)
          continue;
        if (Type == SymbolRef::ST_Unknown)
          if (Name.startswith("__") && !Name.startswith("__rc"))
            continue;

        const auto TryPRGRom = [&]() {
          if ((Address & 0xffff) < 0x8000)
            return false;
          auto UB = PRGROMLMABanks.upper_bound(Address);
          if (UB == PRGROMLMABanks.begin())
            return false;
          --UB;
          uint32_t BankLMA = UB->first;
          StringRef Bank = UB->second;
          const auto OffsetIt = PRGROMBankOffsets.find(Bank);
          if (OffsetIt == PRGROMBankOffsets.end())
            return false;
          uint32_t Offset = OffsetIt->second;
          OS << formatv("P:{0}:{1}\n",
                        BoundsStr(Address - BankLMA + Offset, Size), Name);
          return true;
        };
        if (TryPRGRom())
          continue;
        if (Address < 0x2000) {
          OS << formatv("R:{0}:{1}\n", BoundsStr(Address, Size), Name);
          continue;
        }
        if ((Address & 0xffff) >= 0x6000 && (Address & 0xffff) < 0x8000) {
          uint8_t Bank = Address >> 16;
          // NOTE: This assumes 8K PRG-RAM banks. Once a mapper is added with
          // variable PRG-RAM banking, add a symbol to declare the bank size.
          OS << formatv(
              "{0}:{1}:{2}\n", PRGRAMType,
              BoundsStr((Address & 0xffff) - 0x6000 + Bank * 0x2000, Size), Name);
          continue;
        }

        OS << formatv("G:{0}:{1}\n", BoundsStr(Address, Size), Name);
      }
    } else if (Platform == MLBPlatform::PCE) {
      std::map<uint32_t, StringRef> ROMLMABanks;
      DenseMap<StringRef, uint32_t> ROMBankOffsets;
      SmallVector<StringRef> Matches;
      for (const ELFSymbolRef Sym : O->symbols()) {
        StringRef Name = unwrapOrError(Sym.getName(), O->getFileName());
        if (ROMRegex.match(Name, &Matches)) {
          uint32_t Value = unwrapOrError(Sym.getValue(), O->getFileName());
          if (Matches[2] == "lma") {
            ROMLMABanks[Value] = Matches[1];
          } else {
            assert(Matches[2] == "offset");
            ROMBankOffsets[Matches[1]] = Value;
          }
        }
      }

      for (const ELFSymbolRef Sym : O->symbols()) {
        SymbolRef::Type Type = unwrapOrError(Sym.getType(), InputFilename);
        uint64_t Address = unwrapOrError(Sym.getAddress(), InputFilename);
        StringRef Name = unwrapOrError(Sym.getName(), InputFilename);
        uint64_t Size = Sym.getSize();

        if (Type == SymbolRef::ST_File)
          continue;
        if (Type == SymbolRef::ST_Unknown)
          if (Name.startswith("__") && !Name.startswith("__rc"))
            continue;

        // 0x01xxxxxx - ROM bank addresses
        const auto TryROM = [&]() {
          if ((Address & 0xFF000000) != 0x01000000)
            return false;
          // Convert LMA to ROM position.
          auto UB = ROMLMABanks.upper_bound(Address);
          if (UB == ROMLMABanks.begin())
            return false;
          --UB;
          uint32_t BankLMA = UB->first;
          StringRef Bank = UB->second;
          const auto OffsetIt = ROMBankOffsets.find(Bank);
          if (OffsetIt == ROMBankOffsets.end())
            return false;
          uint32_t Offset = OffsetIt->second;
          
          OS << formatv("PcePrgRom:{0}:{1}\n", BoundsStr(Address - BankLMA + Offset, Size), Name);
          return true;
        };
        if (TryROM())
          continue;

        // 0x0000xxxx - PCE addresses
        if ((Address & 0xFFFF0000) == 0x00000000) {
          OS << formatv("PceMemory:{0}:{1}\n", BoundsStr(Address, Size), Name);
          continue;
        }

        // Unknown address
        reportWarning(InputFilename, formatv("Could not map symbol '{0}'", Name));
      }
    }
  }
}
