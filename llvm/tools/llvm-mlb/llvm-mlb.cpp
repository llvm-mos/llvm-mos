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
                              const DenseMap<SectionRef, uint64_t> &PRGSecLMAs,
                              const object::ELFSectionRef &Sec) {
  auto It = PRGSecLMAs.find(Sec);
  if (It != PRGSecLMAs.end())
    return It->getSecond();
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

    SmallString<32> OutputFilename = InputFilename;
    sys::path::replace_extension(OutputFilename, "mlb");
    std::error_code EC;
    raw_fd_ostream OS(OutputFilename, EC);
    if (EC)
      reportError(OutputFilename, Twine("cannot open: ") + EC.message());

    DenseMap<uint64_t, SmallVector<const ELF32LE::Phdr *>> PRGSegsByOffset;
    for (const auto &Seg :
         unwrapOrError(ELFFile.program_headers(), InputFilename))
      if (Seg.p_paddr >= 0x03000000 && Seg.p_paddr < 0x04000000)
        PRGSegsByOffset[Seg.p_offset].push_back(&Seg);

    // Find PRG-RAM section LMAs.
    DenseMap<SectionRef, uint64_t> PRGSecLMAs;
    for (const auto &[Offset, PRGSegs] : PRGSegsByOffset) {
      uint64_t I = 0;
      uint64_t HighVMA = 0;
      for (const ELFSectionRef Sec : O->sections()) {
        if (Sec.getAddress() < 0x4020 || Sec.getType() != ELF::SHT_NOBITS ||
            Sec.getOffset() != Offset ||
            PRGSecLMAs.find(Sec) != PRGSecLMAs.end())
          continue;
        if (I >= PRGSegs.size())
          break;
        const ELF32LE::Phdr *CurSeg = PRGSegs[I];
        while (Sec.getAddress() < HighVMA ||
               Sec.getAddress() < CurSeg->p_vaddr ||
               Sec.getAddress() >= CurSeg->p_vaddr + CurSeg->p_memsz) {
          HighVMA = 0;
          ++I;
          if (I >= PRGSegs.size())
            break;
          CurSeg = PRGSegs[I];
        }
        PRGSecLMAs[Sec] = Sec.getAddress() - CurSeg->p_vaddr + CurSeg->p_paddr;
        HighVMA = std::max(HighVMA, Sec.getAddress() + Sec.getSize());
      }
    }

    char PRGRAMType = hasPRGNVRAM(*O) ? 'S' : 'W';

    for (const ELFSymbolRef Sym : O->symbols()) {
      SymbolRef::Type Type = unwrapOrError(Sym.getType(), InputFilename);
      auto Sec = unwrapOrError(Sym.getSection(), InputFilename);
      uint64_t Address = unwrapOrError(Sym.getAddress(), InputFilename);
      StringRef Name = unwrapOrError(Sym.getName(), InputFilename);
      uint64_t LMA = Address - Sec->getAddress() +
                     getSectionLMA(ELFFile, PRGSecLMAs, *Sec);
      uint64_t Size = Sym.getSize();

      if (Type == SymbolRef::ST_File)
        continue;
      if (Type == SymbolRef::ST_Unknown)
        if (Name.startswith("__") && !Name.startswith("__rc"))
          continue;

      // PRG-ROM
      if (LMA >= 0x01000000 && LMA < 0x02000000) {
        LMA -= 0x01000000;
        std::string SizeStr =
            Size > 1 ? formatv("-{0:x-}", LMA + Size - 1) : std::string();
        OS << formatv("P:{0:x-}{1}:{2}\n", LMA, SizeStr, Name);
      } else if (LMA >= 0x03000000 && LMA < 0x04000000) {
        LMA -= 0x03000000;
        std::string SizeStr =
            Size > 1 ? formatv("-{0:x-}", LMA + Size - 1) : std::string();
        OS << formatv("{0}:{1:x-}{2}:{3}\n", PRGRAMType, LMA, SizeStr, Name);
      } else if (Address <= 0x2000) {
        std::string SizeStr =
            Size > 1 ? formatv("-{0:x-}", Address + Size - 1) : std::string();
        OS << formatv("R:{0:x-}{1}:{2}\n", Address, SizeStr, Name);
      } else if (Address >= 0x4000 && Address < 0x4020) {
        std::string SizeStr =
            Size > 1 ? formatv("-{0:x-}", Address + Size - 1) : std::string();
        OS << formatv("G:{0:x-}{1}:{2}\n", Address, SizeStr, Name);
      } else if (Address >= 0x6000 && Address < 0x8000) {
        std::string SizeStr =
            Size > 1 ? formatv("-{0:x-}", Address + Size - 1) : std::string();
        OS << formatv("{0}:{1:x-}{2}:{3}\n", PRGRAMType, Address, SizeStr,
                      Name);
      } else {
        WithColor::warning(errs(), "llvm-mlb")
            << "'" << InputFilename << "': unhandled symbol: " << Name << "\n";
      }
    }
  }
}
