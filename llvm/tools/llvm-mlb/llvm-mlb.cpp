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
#include "llvm/Support/InitLLVM.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Regex.h"
#include "llvm/Support/WithColor.h"
#include <system_error>

using namespace llvm;
using namespace llvm::object;

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

int main(int argc, char **argv) {
  InitLLVM X(argc, argv);
  for (int I = 1; I < argc; I++) {
    StringRef InputFilename = argv[I];
    OwningBinary<Binary> OBinary =
        unwrapOrError(createBinary(InputFilename), InputFilename);
    Binary &Binary = *OBinary.getBinary();

    auto *O = dyn_cast<ELF32LEObjectFile>(&Binary);
    if (!O)
      reportError(InputFilename, "expected an ELF object file");
    const ELFFile<ELF32LE> &ELFFile = O->getELFFile();

    StringRef OutputBase = InputFilename;
    OutputBase.consume_back(".elf");
    OutputBase.consume_back(".nes");
    SmallString<32> OutputFilename = OutputBase;
    OutputFilename += ".mlb";
    std::error_code EC;
    raw_fd_ostream OS(OutputFilename, EC);
    if (EC)
      reportError(OutputFilename, Twine("cannot open: ") + EC.message());

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

      const auto BoundsStr = [](uint64_t Address, uint16_t Size) {
        auto UpperBoundStr = Size > 1 ? formatv("-{0:x-}", Address + Size - 1)
                                      : SmallString<10>();
        return formatv("{0:x-}{1}", Address, UpperBoundStr).sstr<10>();
      };

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
  }
}
