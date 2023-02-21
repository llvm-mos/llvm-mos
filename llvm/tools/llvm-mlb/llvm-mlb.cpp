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

template <class ELFT>
static uint64_t getSectionLMA(const ELFFile<ELFT> &Obj,
                              const object::ELFSectionRef &Sec) {
  auto PhdrRangeOrErr = Obj.program_headers();
  if (!PhdrRangeOrErr)
    report_fatal_error(Twine(toString(PhdrRangeOrErr.takeError())));

  // Search for a PT_LOAD segment containing the requested section. Use this
  // segment's p_addr to calculate the section's LMA.
  for (const typename ELFT::Phdr &Phdr : *PhdrRangeOrErr)
    if ((Phdr.p_type == ELF::PT_LOAD) && (Phdr.p_offset <= Sec.getOffset()) &&
        (Phdr.p_offset + Phdr.p_filesz > Sec.getOffset()))
      return Sec.getOffset() - Phdr.p_offset + Phdr.p_paddr;

  // Return section's VMA if it isn't in a PT_LOAD segment.
  return Sec.getAddress();
}

bool hasPRGNVRAM(const ELF32LEObjectFile &O) {
  for (const ELFSymbolRef Sym : O.symbols()) {
    if (unwrapOrError(Sym.getName(), O.getFileName()) != "__prg_nvram_size")
      continue;
    return unwrapOrError(Sym.getValue(), O.getFileName()) != 0;
  }
  return false;
}

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

    char PRGRAMType = hasPRGNVRAM(*O) ? 'S' : 'W';

    for (const ELFSymbolRef Sym : O->symbols()) {
      SymbolRef::Type Type = unwrapOrError(Sym.getType(), InputFilename);
      auto Sec = unwrapOrError(Sym.getSection(), InputFilename);
      uint64_t Address = unwrapOrError(Sym.getAddress(), InputFilename);
      StringRef Name = unwrapOrError(Sym.getName(), InputFilename);
      uint64_t LMA = Address - Sec->getAddress() + getSectionLMA(ELFFile, *Sec);
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

      // PRG-ROM
      if (LMA >= 0x01000000 && LMA < 0x02000000) {
        LMA -= 0x01000000;
        OS << formatv("P:{0}:{1}\n", BoundsStr(LMA, Size), Name);
      } else if (Address < 0x2000) {
        OS << formatv("R:{0}:{1}\n", BoundsStr(Address, Size), Name);
      } else if ((Address & 0xffff) >= 0x6000 && (Address & 0xffff) < 0x8000) {
        uint8_t Bank = Address >> 16;
        // NOTE: This assumes 8K PRG-RAM banks. Once a mapper is added with
        // variable PRG-RAM banking, add a symbol to declare the bank size.
        OS << formatv(
            "{0}:{1}:{2}\n", PRGRAMType,
            BoundsStr((Address & 0xffff) - 0x6000 + Bank * 0x2000, Size), Name);
      } else {
        OS << formatv("G:{0}:{1}\n", BoundsStr(Address, Size), Name);
      }
    }
  }
}
