//===--- MOS.cpp - Implement MOS target feature support -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements MOS TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "MOS.h"
#include "clang/Basic/MacroBuilder.h"
#include "clang/Basic/TargetInfo.h"

using namespace clang::targets;

MOSTargetInfo::MOSTargetInfo(const llvm::Triple &Triple, const TargetOptions &)
    : TargetInfo(Triple) {
  static const char Layout[] =
      "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8";
  resetDataLayout(Layout);

  PointerWidth = 16;
  PointerAlign = 8;
  ShortAlign = 8;
  IntWidth = 16;
  IntAlign = 8;
  LongAlign = 8;
  LongLongAlign = 8;
  BFloat16Align = 8;
  FloatAlign = 8;
  DoubleAlign = 8;
  LongDoubleAlign = 8;
  Float128Align = 8;
  ShortAccumAlign = 8;
  AccumWidth = 16;
  AccumAlign = 8;
  LongAccumAlign = 8;
  FractWidth = FractAlign = 8;
  LongFractAlign = 8;
  AccumScale = 7;
  SuitableAlign = 8;
  DefaultAlignForAttributeAligned = 8;
  SizeType = UnsignedShort;
  PtrDiffType = SignedShort;
  IntPtrType = SignedShort;
  WCharType = UnsignedLong;
  WIntType = UnsignedLong;
  Int16Type = SignedInt;
  Char32Type = UnsignedLong;
  SigAtomicType = UnsignedChar;
}

bool MOSTargetInfo::validateAsmConstraint(
    const char *&Name, TargetInfo::ConstraintInfo &Info) const {
  switch (*Name) {
  default:
    return false;
  // The A, X, or Y registers.
  case 'a':
  case 'x':
  case 'y':
  // Any of A, X, or Y.
  case 'R':
  // The index (X or Y) registers.
  case 'd':
  // The C and V flags.
  case 'c':
  case 'v':
    Info.setAllowsRegister();
    return true;
  }
}

bool MOSTargetInfo::validateOutputSize(const llvm::StringMap<bool> &FeatureMap,
                                       StringRef Constraint,
                                       unsigned Size) const {
  // Strip off constraint modifiers.
  while (Constraint[0] == '=' || Constraint[0] == '+' || Constraint[0] == '&')
    Constraint = Constraint.substr(1);

  return validateOperandSize(FeatureMap, Constraint, Size);
}

bool MOSTargetInfo::validateInputSize(const llvm::StringMap<bool> &FeatureMap,
                                      StringRef Constraint,
                                      unsigned Size) const {
  return validateOperandSize(FeatureMap, Constraint, Size);
}

bool MOSTargetInfo::validateOperandSize(const llvm::StringMap<bool> &FeatureMap,
                                        StringRef Constraint,
                                        unsigned Size) const {
  switch (Constraint[0]) {
  default:
    return true;
  case 'a':
  case 'x':
  case 'y':
  case 'R':
  case 'd':
  case 'c':
  case 'v':
    return Size <= 8;
  }
}

llvm::StringRef
MOSTargetInfo::getConstraintRegister(StringRef Constraint,
                                     StringRef Expression) const {
  StringRef::iterator I, E;
  for (I = Constraint.begin(), E = Constraint.end(); I != E; ++I) {
    if (isalpha(*I))
      break;
  }
  if (I == E)
    return "";
  switch (*I) {
  case 'a':
    return "a";
  case 'x':
    return "x";
  case 'y':
    return "y";
  case 'c':
    return "c";
  case 'v':
    return "v";
  case 'd':
  case 'g':
  case 'r':
  case 'R':
    // This handles any global or local register variables that make this more
    // explicit.
    return Expression;
  }
  return "";
}

static const char *const GCCRegNames[] = {
    "a",     "x",     "y",     "c",     "v",     "p",     "rc0",   "rc1",
    "rc2",   "rc3",   "rc4",   "rc5",   "rc6",   "rc7",   "rc8",   "rc9",
    "rc10",  "rc11",  "rc12",  "rc13",  "rc14",  "rc15",  "rc16",  "rc17",
    "rc18",  "rc19",  "rc20",  "rc21",  "rc22",  "rc23",  "rc24",  "rc25",
    "rc26",  "rc27",  "rc28",  "rc29",  "rc30",  "rc31",  "rc32",  "rc33",
    "rc34",  "rc35",  "rc36",  "rc37",  "rc38",  "rc39",  "rc40",  "rc41",
    "rc42",  "rc43",  "rc44",  "rc45",  "rc46",  "rc47",  "rc48",  "rc49",
    "rc50",  "rc51",  "rc52",  "rc53",  "rc54",  "rc55",  "rc56",  "rc57",
    "rc58",  "rc59",  "rc60",  "rc61",  "rc62",  "rc63",  "rc64",  "rc65",
    "rc66",  "rc67",  "rc68",  "rc69",  "rc70",  "rc71",  "rc72",  "rc73",
    "rc74",  "rc75",  "rc76",  "rc77",  "rc78",  "rc79",  "rc80",  "rc81",
    "rc82",  "rc83",  "rc84",  "rc85",  "rc86",  "rc87",  "rc88",  "rc89",
    "rc90",  "rc91",  "rc92",  "rc93",  "rc94",  "rc95",  "rc96",  "rc97",
    "rc98",  "rc99",  "rc100", "rc101", "rc102", "rc103", "rc104", "rc105",
    "rc106", "rc107", "rc108", "rc109", "rc110", "rc111", "rc112", "rc113",
    "rc114", "rc115", "rc116", "rc117", "rc118", "rc119", "rc120", "rc121",
    "rc122", "rc123", "rc124", "rc125", "rc126", "rc127", "rc128", "rc129",
    "rc130", "rc131", "rc132", "rc133", "rc134", "rc135", "rc136", "rc137",
    "rc138", "rc139", "rc140", "rc141", "rc142", "rc143", "rc144", "rc145",
    "rc146", "rc147", "rc148", "rc149", "rc150", "rc151", "rc152", "rc153",
    "rc154", "rc155", "rc156", "rc157", "rc158", "rc159", "rc160", "rc161",
    "rc162", "rc163", "rc164", "rc165", "rc166", "rc167", "rc168", "rc169",
    "rc170", "rc171", "rc172", "rc173", "rc174", "rc175", "rc176", "rc177",
    "rc178", "rc179", "rc180", "rc181", "rc182", "rc183", "rc184", "rc185",
    "rc186", "rc187", "rc188", "rc189", "rc190", "rc191", "rc192", "rc193",
    "rc194", "rc195", "rc196", "rc197", "rc198", "rc199", "rc200", "rc201",
    "rc202", "rc203", "rc204", "rc205", "rc206", "rc207", "rc208", "rc209",
    "rc210", "rc211", "rc212", "rc213", "rc214", "rc215", "rc216", "rc217",
    "rc218", "rc219", "rc220", "rc221", "rc222", "rc223", "rc224", "rc225",
    "rc226", "rc227", "rc228", "rc229", "rc230", "rc231", "rc232", "rc233",
    "rc234", "rc235", "rc236", "rc237", "rc238", "rc239", "rc240", "rc241",
    "rc242", "rc243", "rc244", "rc245", "rc246", "rc247", "rc248", "rc249",
    "rc250", "rc251", "rc252", "rc253", "rc254", "rc255", "rs0",   "rs1",
    "rs2",   "rs3",   "rs4",   "rs5",   "rs6",   "rs7",   "rs8",   "rs9",
    "rs10",  "rs11",  "rs12",  "rs13",  "rs14",  "rs15",  "rs16",  "rs17",
    "rs18",  "rs19",  "rs20",  "rs21",  "rs22",  "rs23",  "rs24",  "rs25",
    "rs26",  "rs27",  "rs28",  "rs29",  "rs30",  "rs31",  "rs32",  "rs33",
    "rs34",  "rs35",  "rs36",  "rs37",  "rs38",  "rs39",  "rs40",  "rs41",
    "rs42",  "rs43",  "rs44",  "rs45",  "rs46",  "rs47",  "rs48",  "rs49",
    "rs50",  "rs51",  "rs52",  "rs53",  "rs54",  "rs55",  "rs56",  "rs57",
    "rs58",  "rs59",  "rs60",  "rs61",  "rs62",  "rs63",  "rs64",  "rs65",
    "rs66",  "rs67",  "rs68",  "rs69",  "rs70",  "rs71",  "rs72",  "rs73",
    "rs74",  "rs75",  "rs76",  "rs77",  "rs78",  "rs79",  "rs80",  "rs81",
    "rs82",  "rs83",  "rs84",  "rs85",  "rs86",  "rs87",  "rs88",  "rs89",
    "rs90",  "rs91",  "rs92",  "rs93",  "rs94",  "rs95",  "rs96",  "rs97",
    "rs98",  "rs99",  "rs100", "rs101", "rs102", "rs103", "rs104", "rs105",
    "rs106", "rs107", "rs108", "rs109", "rs110", "rs111", "rs112", "rs113",
    "rs114", "rs115", "rs116", "rs117", "rs118", "rs119", "rs120", "rs121",
    "rs122", "rs123", "rs124", "rs125", "rs126", "rs127",
};

llvm::ArrayRef<const char *> MOSTargetInfo::getGCCRegNames() const {
  return GCCRegNames;
}

uint64_t MOSTargetInfo::getPointerWidthV(LangAS AddrSpace) const {
  // Coerce all non-target address spaces to the default address space.
  if (!isTargetAddressSpace(AddrSpace))
    return getPointerWidthV(LangAS::FirstTargetAddressSpace);

  switch (toTargetAddressSpace(AddrSpace)) {
  case 1: // Zero page memory
    return 8;
  default:
    return 16;
  }
}

static constexpr llvm::StringLiteral ValidCPUNames[] = {
    {"mos6502"},    {"mos6502x"},   {"mos65c02"},   {"mosr65c02"},
    {"mosw65c02"},  {"mosw65816"},  {"mos65el02"},  {"mos65ce02"},
    {"moshuc6280"}, {"mossweet16"}, {"mos65dtv02"}, {"mos4510"},
    {"mos45gs02"},  {"mosspc700"}};

bool MOSTargetInfo::isValidCPUName(StringRef Name) const {
  return llvm::find(ValidCPUNames, Name) != std::end(ValidCPUNames);
}

void MOSTargetInfo::fillValidCPUList(SmallVectorImpl<StringRef> &Values) const {
  Values.append(std::begin(ValidCPUNames), std::end(ValidCPUNames));
}

bool MOSTargetInfo::setCPU(const std::string &Name) {
  if (isValidCPUName(Name)) {
    CPUName = Name;
    return true;
  }
  return false;
}

void MOSTargetInfo::getTargetDefines(const LangOptions &Opts,
                                     MacroBuilder &Builder) const {
  Builder.defineMacro("__mos__");
  Builder.defineMacro("__ELF__");
  Builder.defineMacro("__SOFTFP__");

  // Generate instruction feature set macros.
  const auto &CPUDefines =
      llvm::StringSwitch<std::vector<std::string>>(CPUName)
          .Case("mos6502", {"6502"})
          .Case("mos6502x", {"6502", "6502x"})
          .Case("mos65c02", {"6502", "65c02"})
          .Case("mosr65c02", {"6502", "65c02", "r65c02"})
          .Case("mosw65c02", {"6502", "65c02", "r65c02", "w65c02"})
          .Case("mosw65816", {"6502", "65c02", "w65816", "w65c02"})
          .Case("mos65el02", {"6502", "65c02", "65el02", "w65c02"})
          .Case("mos65ce02", {"6502", "65c02", "65ce02", "r65c02"})
          .Case("moshuc6280", {"6502", "65c02", "huc6280", "r65c02"})
          .Case("mos65dtv02", {"6502", "65dtv02"})
          .Case("mos4510", {"4510", "6502", "65c02", "65ce02", "r65c02"})
          .Case("mos45gs02",
                {"4510", "45gs02", "6502", "65c02", "65ce02", "r65c02"})
          .Case("mosspc700", {"spc700"})
          .Default({"6502"});
  for (const auto &CPUDefine : CPUDefines) {
    Builder.defineMacro("__mos" + CPUDefine + "__");
  }

  Builder.defineMacro("__zp", "__attribute__((__address_space__(1)))");
  Builder.defineMacro("__zeropage", "__attribute__((__address_space__(1)))");
}
