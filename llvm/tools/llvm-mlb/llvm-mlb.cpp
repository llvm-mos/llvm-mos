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

#include "llvm/ADT/SmallString.h"
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

enum class MLBPlatform { Auto, NES, PCE };

cl::opt<MLBPlatform>
    Platform("platform", cl::desc(""), cl::init(MLBPlatform::Auto),
             cl::values(clEnumValN(MLBPlatform::Auto, "Auto",
                                   "Detect automatically based on filename."),
                        clEnumValN(MLBPlatform::NES, "NES", "NES"),
                        clEnumValN(MLBPlatform::PCE, "PCE", "PCE")),
             cl::NotHidden, cl::cat(MLBCategory));

cl::list<std::string> ClInputFilenames(cl::Positional, cl::OneOrMore,
                                       cl::desc("<input ELF files>"),
                                       cl::cat(MLBCategory));

cl::opt<std::string> ClOutputFilename("o", cl::init(""),
                                      cl::desc("Override output filename"),
                                      cl::value_desc("filename"),
                                      cl::cat(MLBCategory));
} // namespace

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
static Regex VBankRegex("^__(.*_vbank.*)_(lma|offset)$");

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
      Platform = IsInputExtNES
                     ? MLBPlatform::NES
                     : (IsInputExtPCE ? MLBPlatform::PCE : MLBPlatform::Auto);

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
      auto UpperBoundStr =
          Size > 1 ? formatv("-{0:x-}", Address + Size - 1) : SmallString<10>();
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
          if (Name.starts_with("__") && !Name.starts_with("__rc"))
            continue;

        // Mesen 2 incorrectly displays executable symbols with a size attached.
        if (Type == SymbolRef::ST_Function)
          Size = 1;

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
              BoundsStr((Address & 0xffff) - 0x6000 + Bank * 0x2000, Size),
              Name);
          continue;
        }

        OS << formatv("G:{0}:{1}\n", BoundsStr(Address, Size), Name);
      }
    } else if (Platform == MLBPlatform::PCE) {
      std::map<uint32_t, StringRef> VBankLMABanks;
      DenseMap<StringRef, uint32_t> VBankBankOffsets;
      SmallVector<StringRef> Matches;
      for (const ELFSymbolRef Sym : O->symbols()) {
        StringRef Name = unwrapOrError(Sym.getName(), O->getFileName());
        if (VBankRegex.match(Name, &Matches)) {
          uint32_t Value = unwrapOrError(Sym.getValue(), O->getFileName());
          if (Matches[2] == "lma") {
            VBankLMABanks[Value] = Matches[1];
          } else {
            assert(Matches[2] == "offset");
            VBankBankOffsets[Matches[1]] = Value;
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
          if (Name.starts_with("__") && !Name.starts_with("__rc"))
            continue;

        const auto TryCard = [&]() {
          uint8_t Group = Address >> 24;
          uint8_t Bank = (Address >> 16) & 0xFF;
          int32_t DefaultOffset = -1; // Default offset.
          SmallString<32> Type;

          if (Group == 0x00) {
            // 0x00XXXXXX - base PC Engine bank addresses
            if (Bank >= 0x00 && Bank <= 0x7F) {
              Type = "PcePrgRom";
              DefaultOffset = Bank * 0x2000;
            } else if (Bank >= 0xF8 && Bank <= 0xFB) {
              Type = "PceWorkRam";
              DefaultOffset = (Bank - 0xF8) * 0x2000;
            } else {
              return false;
            }
          } else if (Group == 0x01) {
            // 0x01XXXXXX - card/CD-ROM RAM bank addresses
            if (Bank >= 0x80 && Bank <= 0x87) {
              Type = "PceCdromRam";
              DefaultOffset = (Bank - 0x80) * 0x2000;
            } else if (Bank >= 0x68 && Bank <= 0x7F) {
              Type = "PceCardRam";
              DefaultOffset = (Bank - 0x68) * 0x2000;
            } else if (Bank >= 0x40 && Bank <= 0x5F) {
              Type = "PceCardRam";
              DefaultOffset = (Bank - 0x40) * 0x2000;
            } else {
              return false;
            }
          } else if (Group <= 0x11) {
            // (0x02-0x11)XXXXXX, bank 40-7F - card ROM bank address (SF2
            // mapper)
            if (Bank < 0x40 || Bank >= 0x80)
              return false;
            Type = "PcePrgRom";
            DefaultOffset = (Group * 64 + (Bank - 0x40)) * 0x2000;
          } else {
            return false;
          }

          // Convert LMA to ROM position.
          uint32_t BankLMA;
          uint32_t Offset;
          auto UB = VBankLMABanks.upper_bound(Address);
          bool UBValid = false;
          if (UB != VBankLMABanks.begin()) {
            --UB;
            BankLMA = UB->first;
            UBValid = (BankLMA & 0xFFFF0000) == (Address & 0xFFFF0000);
          }
          if (UBValid) {
            StringRef Bank = UB->second;
            const auto OffsetIt = VBankBankOffsets.find(Bank);
            if (OffsetIt == VBankBankOffsets.end())
              return false;
            Offset = OffsetIt->second;
          } else {
            // VBanks not used; guess the LMA and offset.
            if (DefaultOffset < 0)
              return false;
            BankLMA = Address & 0xFFFFE000;
            Offset = DefaultOffset;
          }

          OS << formatv("{0}:{1}:{2}\n", Type,
                        BoundsStr(Address - BankLMA + Offset, Size), Name);
          return true;
        };
        if (TryCard())
          continue;

        // 0x00F7XXXX - CD backup RAM bank address
        if ((Address & 0xFFFF0000) == 0x00F70000) {
          OS << formatv("PceSaveRam:{0}:{1}\n",
                        BoundsStr(Address & 0x1FFF, Size), Name);
          continue;
        }
        // 0x00FFXXXX - I/O address
        if ((Address & 0xFFFF0000) == 0x00FF0000) {
          OS << formatv("PceMemory:{0}:{1}\n",
                        BoundsStr(Address & 0x1FFF, Size), Name);
          continue;
        }

        // Unknown address
        reportWarning(InputFilename,
                      formatv("Could not map symbol '{0}'", Name));
      }
    }
  }
}
