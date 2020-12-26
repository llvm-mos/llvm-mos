//===---------------------------------------------------------------------===//
// MOS Device Definitions
//===---------------------------------------------------------------------===//

// :TODO: Implement the skip errata, see `gcc/config/avr/avr-arch.h` for details
// :TODO: We define all devices with SRAM to have all variants of LD/ST/LDD/STD.
//        In reality, avr1 (no SRAM) has one variant each of `LD` and `ST`.
//        avr2 (with SRAM) adds the rest of the variants.


// A feature set aggregates features, grouping them. We don't want to create a
// new member in MOSSubtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "m_FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, list<SubtargetFeature> i>
  : FeatureSet<name, !strconcat("The device is a part of the ",
               name, " family"), i>;

// The device has SRAM, and supports the bare minimum of
// SRAM-relevant instructions.
//
// These are:
// LD - all 9 variants
// ST - all 9 variants
// LDD - two variants for Y and Z
// STD - two variants for Y and Z
// `LDS Rd, K`
// `STS k, Rr`
// `PUSH`/`POP`
def FeatureSRAM           : SubtargetFeature<"sram", "m_hasSRAM", "true",
                                  "The device has random access memory">;

// The device supports the `JMP k` and `CALL k` instructions.
def FeatureJMPCALL        : SubtargetFeature<"jmpcall", "m_hasJMPCALL", "true",
                                  "The device supports the `JMP` and "
                                  "`CALL` instructions">;


// The device supports the indirect branches `IJMP` and `ICALL`.
def FeatureIJMPCALL       : SubtargetFeature<"ijmpcall", "m_hasIJMPCALL",
                                  "true",
                                  "The device supports `IJMP`/`ICALL`"
                                  "instructions">;

// The device supports the extended indirect branches `EIJMP` and `EICALL`.
def FeatureEIJMPCALL      : SubtargetFeature<"eijmpcall", "m_hasEIJMPCALL",
                                  "true", "The device supports the "
                                  "`EIJMP`/`EICALL` instructions">;

// The device supports `ADDI Rd, K`, `SUBI Rd, K`.
def FeatureADDSUBIW       : SubtargetFeature<"addsubiw", "m_hasADDSUBIW",
                                  "true", "Enable 16-bit register-immediate "
                                  "addition and subtraction instructions">;

// The device has an 8-bit stack pointer (SP) register.
def FeatureSmallStack     : SubtargetFeature<"smallstack", "m_hasSmallStack",
                                  "true", "The device has an 8-bit "
                                  "stack pointer">;

// The device supports the 16-bit GPR pair MOVW instruction.
def FeatureMOVW           : SubtargetFeature<"movw", "m_hasMOVW", "true",
                                  "The device supports the 16-bit MOVW "
                                  "instruction">;

// The device supports the `LPM` instruction, with implied destination being r0.
def FeatureLPM            : SubtargetFeature<"lpm", "m_hasLPM", "true",
                                  "The device supports the `LPM` instruction">;

// The device supports the `LPM Rd, Z[+] instruction.
def FeatureLPMX           : SubtargetFeature<"lpmx", "m_hasLPMX", "true",
                                  "The device supports the `LPM Rd, Z[+]` "
                                  "instruction">;

// The device supports the `ELPM` instruction.
def FeatureELPM           : SubtargetFeature<"elpm", "m_hasELPM", "true",
                                  "The device supports the ELPM instruction">;

// The device supports the `ELPM Rd, Z[+]` instructions.
def FeatureELPMX          : SubtargetFeature<"elpmx", "m_hasELPMX", "true",
                                  "The device supports the `ELPM Rd, Z[+]` "
                                  "instructions">;

// The device supports the `SPM` instruction.
def FeatureSPM            : SubtargetFeature<"spm", "m_hasSPM", "true",
                                  "The device supports the `SPM` instruction">;

// The device supports the `SPM Z+` instruction.
def FeatureSPMX           : SubtargetFeature<"spmx", "m_hasSPMX", "true",
                                  "The device supports the `SPM Z+` "
                                  "instruction">;

// The device supports the `DES k` instruction.
def FeatureDES            : SubtargetFeature<"des", "m_hasDES", "true",
                                  "The device supports the `DES k` encryption "
                                  "instruction">;

// The device supports the Read-Write-Modify instructions
// XCH, LAS, LAC, and LAT.
def FeatureRMW            : SubtargetFeature<"rmw", "m_supportsRMW", "true",
                                  "The device supports the read-write-modify "
                                  "instructions: XCH, LAS, LAC, LAT">;

// The device supports the `[F]MUL[S][U]` family of instructions.
def FeatureMultiplication : SubtargetFeature<"mul", "m_supportsMultiplication",
                                  "true", "The device supports the "
                                  "multiplication instructions">;

// The device supports the `BREAK` instruction.
def FeatureBREAK          : SubtargetFeature<"break", "m_hasBREAK", "true",
                                  "The device supports the `BREAK` debugging "
                                  "instruction">;

// The device has instruction encodings specific to the Tiny core.
def FeatureTinyEncoding   : SubtargetFeature<"tinyencoding",
                                  "m_hasTinyEncoding", "true",
                                  "The device has Tiny core specific "
                                  "instruction encodings">;

class ELFArch<string name>  : SubtargetFeature<"", "ELFArch",
                                    !strconcat("ELF::",name), "">;

// ELF e_flags architecture values
def ELFArchMOS1    : ELFArch<"EF_MOS_ARCH_MOS1">;
def ELFArchMOS2    : ELFArch<"EF_MOS_ARCH_MOS2">;
def ELFArchMOS25   : ELFArch<"EF_MOS_ARCH_MOS25">;
def ELFArchMOS3    : ELFArch<"EF_MOS_ARCH_MOS3">;
def ELFArchMOS31   : ELFArch<"EF_MOS_ARCH_MOS31">;
def ELFArchMOS35   : ELFArch<"EF_MOS_ARCH_MOS35">;
def ELFArchMOS4    : ELFArch<"EF_MOS_ARCH_MOS4">;
def ELFArchMOS5    : ELFArch<"EF_MOS_ARCH_MOS5">;
def ELFArchMOS51   : ELFArch<"EF_MOS_ARCH_MOS51">;
def ELFArchMOS6    : ELFArch<"EF_MOS_ARCH_MOS6">;
def ELFArchTiny    : ELFArch<"EF_MOS_ARCH_MOSTINY">;
def ELFArchXMEGA1  : ELFArch<"EF_MOS_ARCH_XMEGA1">;
def ELFArchXMEGA2  : ELFArch<"EF_MOS_ARCH_XMEGA2">;
def ELFArchXMEGA3  : ELFArch<"EF_MOS_ARCH_XMEGA3">;
def ELFArchXMEGA4  : ELFArch<"EF_MOS_ARCH_XMEGA4">;
def ELFArchXMEGA5  : ELFArch<"EF_MOS_ARCH_XMEGA5">;
def ELFArchXMEGA6  : ELFArch<"EF_MOS_ARCH_XMEGA6">;
def ELFArchXMEGA7  : ELFArch<"EF_MOS_ARCH_XMEGA7">;

//===---------------------------------------------------------------------===//
// MOS Families
//===---------------------------------------------------------------------===//

// The device has at least the bare minimum that **every** single MOS
// device should have.
def FamilyMOS0           : Family<"avr0", []>;

def FamilyMOS1           : Family<"avr1", [FamilyMOS0, FeatureLPM]>;

def FamilyMOS2           : Family<"avr2",
                                 [FamilyMOS1, FeatureIJMPCALL, FeatureADDSUBIW,
                                  FeatureSRAM]>;

def FamilyMOS25          : Family<"avr25",
                                 [FamilyMOS2, FeatureMOVW, FeatureLPMX,
                                  FeatureSPM, FeatureBREAK]>;

def FamilyMOS3           : Family<"avr3",
                                 [FamilyMOS2, FeatureJMPCALL]>;

def FamilyMOS31          : Family<"avr31",
                                 [FamilyMOS3, FeatureELPM]>;

def FamilyMOS35          : Family<"avr35",
                                 [FamilyMOS3, FeatureMOVW, FeatureLPMX,
                                  FeatureSPM, FeatureBREAK]>;

def FamilyMOS4           : Family<"avr4",
                                 [FamilyMOS2, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK]>;

def FamilyMOS5           : Family<"avr5",
                                 [FamilyMOS3, FeatureMultiplication,
                                  FeatureMOVW, FeatureLPMX, FeatureSPM,
                                  FeatureBREAK]>;

def FamilyMOS51          : Family<"avr51",
                                 [FamilyMOS5, FeatureELPM, FeatureELPMX]>;

def FamilyMOS6           : Family<"avr6",
                                 [FamilyMOS51]>;

def FamilyTiny           : Family<"avrtiny",
                                 [FamilyMOS0, FeatureBREAK, FeatureSRAM,
                                  FeatureTinyEncoding]>;

def FamilyXMEGA          : Family<"xmega",
                                 [FamilyMOS51, FeatureEIJMPCALL, FeatureSPMX,
                                  FeatureDES]>;

def FamilyXMEGAU         : Family<"xmegau",
                                  [FamilyXMEGA, FeatureRMW]>;

def FeatureSetSpecial    : FeatureSet<"special",
                                      "Enable use of the entire instruction "
                                      "set - used for debugging",
                                      [FeatureSRAM, FeatureJMPCALL,
                                       FeatureIJMPCALL, FeatureEIJMPCALL,
                                       FeatureADDSUBIW, FeatureMOVW,
                                       FeatureLPM, FeatureLPMX, FeatureELPM,
                                       FeatureELPMX, FeatureSPM, FeatureSPMX,
                                       FeatureDES, FeatureRMW,
                                       FeatureMultiplication, FeatureBREAK]>;

//===---------------------------------------------------------------------===//
// MOS microcontrollers supported.
//===---------------------------------------------------------------------===//

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
  : Processor<Name, NoItineraries, !listconcat([Fam,Arch],ExtraFeatures)>;

// Generic MCUs
// Note that several versions of GCC has strange ELF architecture
// settings for backwards compatibility - see `gas/config/tc-avr.c`
// in MOS binutils. We do not replicate this.
def : Device<"avr1",      FamilyMOS1,    ELFArchMOS1>;
def : Device<"avr2",      FamilyMOS2,    ELFArchMOS2>;
def : Device<"avr25",     FamilyMOS25,   ELFArchMOS25>;
def : Device<"avr3",      FamilyMOS3,    ELFArchMOS3>;
def : Device<"avr31",     FamilyMOS31,   ELFArchMOS31>;
def : Device<"avr35",     FamilyMOS35,   ELFArchMOS35>;
def : Device<"avr4",      FamilyMOS4,    ELFArchMOS4>;
def : Device<"avr5",      FamilyMOS5,    ELFArchMOS5>;
def : Device<"avr51",     FamilyMOS51,   ELFArchMOS51>;
def : Device<"avr6",      FamilyMOS6,    ELFArchMOS6>;
def : Device<"avrxmega1", FamilyXMEGA,   ELFArchXMEGA1>;
def : Device<"avrxmega2", FamilyXMEGA,   ELFArchXMEGA2>;
def : Device<"avrxmega3", FamilyXMEGA,   ELFArchXMEGA3>;
def : Device<"avrxmega4", FamilyXMEGA,   ELFArchXMEGA4>;
def : Device<"avrxmega5", FamilyXMEGA,   ELFArchXMEGA5>;
def : Device<"avrxmega6", FamilyXMEGA,   ELFArchXMEGA6>;
def : Device<"avrxmega7", FamilyXMEGA,   ELFArchXMEGA7>;
def : Device<"avrtiny",   FamilyTiny,    ELFArchTiny>;

// Specific MCUs
def : Device<"at90s1200",          FamilyMOS0, ELFArchMOS1>;
def : Device<"attiny11",           FamilyMOS1, ELFArchMOS1>;
def : Device<"attiny12",           FamilyMOS1, ELFArchMOS1>;
def : Device<"attiny15",           FamilyMOS1, ELFArchMOS1>;
def : Device<"attiny28",           FamilyMOS1, ELFArchMOS1>;
def : Device<"at90s2313",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s2323",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s2333",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s2343",          FamilyMOS2, ELFArchMOS2>;
def : Device<"attiny22",           FamilyMOS2, ELFArchMOS2>;
def : Device<"attiny26",           FamilyMOS2, ELFArchMOS2, [FeatureLPMX]>;
def : Device<"at86rf401",          FamilyMOS2, ELFArchMOS25,
             [FeatureMOVW, FeatureLPMX]>;
def : Device<"at90s4414",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s4433",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s4434",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s8515",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90c8534",          FamilyMOS2, ELFArchMOS2>;
def : Device<"at90s8535",          FamilyMOS2, ELFArchMOS2>;
def : Device<"ata5272",            FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny13",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny13a",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny2313",         FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny2313a",        FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny24",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny24a",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny4313",         FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny44",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny44a",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny84",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny84a",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny25",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny45",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny85",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny261",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny261a",         FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny461",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny461a",         FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny861",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny861a",         FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny87",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny43u",          FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny48",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny88",           FamilyMOS25, ELFArchMOS25>;
def : Device<"attiny828",          FamilyMOS25, ELFArchMOS25>;
def : Device<"at43usb355",         FamilyMOS3,  ELFArchMOS3>;
def : Device<"at76c711",           FamilyMOS3,  ELFArchMOS3>;
def : Device<"atmega103",          FamilyMOS31, ELFArchMOS31>;
def : Device<"at43usb320",         FamilyMOS31, ELFArchMOS31>;
def : Device<"attiny167",          FamilyMOS35, ELFArchMOS35>;
def : Device<"at90usb82",          FamilyMOS35, ELFArchMOS35>;
def : Device<"at90usb162",         FamilyMOS35, ELFArchMOS35>;
def : Device<"ata5505",            FamilyMOS35, ELFArchMOS35>;
def : Device<"atmega8u2",          FamilyMOS35, ELFArchMOS35>;
def : Device<"atmega16u2",         FamilyMOS35, ELFArchMOS35>;
def : Device<"atmega32u2",         FamilyMOS35, ELFArchMOS35>;
def : Device<"attiny1634",         FamilyMOS35, ELFArchMOS35>;
def : Device<"atmega8",            FamilyMOS4,  ELFArchMOS4>; // FIXME: family may be wrong
def : Device<"ata6289",            FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega8a",           FamilyMOS4,  ELFArchMOS4>;
def : Device<"ata6285",            FamilyMOS4,  ELFArchMOS4>;
def : Device<"ata6286",            FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega48",           FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega48a",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega48pa",         FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega48p",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega88",           FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega88a",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega88p",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega88pa",         FamilyMOS4,  ELFArchMOS4>;
def : Device<"atmega8515",         FamilyMOS2,  ELFArchMOS4,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8535",         FamilyMOS2,  ELFArchMOS4,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8hva",         FamilyMOS4,  ELFArchMOS4>;
def : Device<"at90pwm1",           FamilyMOS4,  ELFArchMOS4>;
def : Device<"at90pwm2",           FamilyMOS4,  ELFArchMOS4>;
def : Device<"at90pwm2b",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"at90pwm3",           FamilyMOS4,  ELFArchMOS4>;
def : Device<"at90pwm3b",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"at90pwm81",          FamilyMOS4,  ELFArchMOS4>;
def : Device<"ata5790",            FamilyMOS5,  ELFArchMOS5>;
def : Device<"ata5795",            FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16",           FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16a",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega161",          FamilyMOS3,  ELFArchMOS5,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega162",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega163",          FamilyMOS3,  ELFArchMOS5,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega164a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega164p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega164pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega165",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega165a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega165p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega165pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega168",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega168a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega168p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega168pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega169",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega169a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega169p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega169pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32",           FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32a",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega323",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega324a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega324p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega324pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega325",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega325a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega325p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega325pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3250",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3250a",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3250p",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3250pa",       FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega328",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega328p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega329",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega329a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega329p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega329pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3290",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3290a",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3290p",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega3290pa",       FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega406",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega64",           FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega64a",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega640",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega644",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega644a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega644p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega644pa",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega645",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega645a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega645p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega649",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega649a",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega649p",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega6450",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega6450a",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega6450p",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega6490",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega6490a",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega6490p",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega64rfr2",       FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega644rfr2",      FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16hva",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16hva2",       FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16hvb",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16hvbrevb",    FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32hvb",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32hvbrevb",    FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega64hve",        FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90can32",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90can64",          FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90pwm161",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90pwm216",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90pwm316",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32c1",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega64c1",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16m1",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32m1",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega64m1",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega16u4",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32u4",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega32u6",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90usb646",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90usb647",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"at90scr100",         FamilyMOS5,  ELFArchMOS5>;
def : Device<"at94k",              FamilyMOS3,  ELFArchMOS5,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX]>;
def : Device<"m3000",              FamilyMOS5,  ELFArchMOS5>;
def : Device<"atmega128",          FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega128a",         FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega1280",         FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega1281",         FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega1284",         FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega1284p",        FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega128rfa1",      FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega128rfr2",      FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega1284rfr2",     FamilyMOS51, ELFArchMOS51>;
def : Device<"at90can128",         FamilyMOS51, ELFArchMOS51>;
def : Device<"at90usb1286",        FamilyMOS51, ELFArchMOS51>;
def : Device<"at90usb1287",        FamilyMOS51, ELFArchMOS51>;
def : Device<"atmega2560",         FamilyMOS6,  ELFArchMOS6>;
def : Device<"atmega2561",         FamilyMOS6,  ELFArchMOS6>;
def : Device<"atmega256rfr2",      FamilyMOS6,  ELFArchMOS6>;
def : Device<"atmega2564rfr2",     FamilyMOS6,  ELFArchMOS6>;
def : Device<"atxmega16a4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega16a4u",       FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16c4",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16d4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4u",       FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32c4",        FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32d4",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32e5",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega16e5",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega8e5",         FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32x1",        FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega64a3",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a3u",       FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64a4u",       FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b1",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b3",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64c3",        FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64d3",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64d4",        FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a1",        FamilyXMEGA, ELFArchXMEGA5>;
def : Device<"atxmega64a1u",       FamilyXMEGAU, ELFArchXMEGA5>;
def : Device<"atxmega128a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b1",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128d4",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3u",      FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256a3b",      FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3bu",     FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega384c3",       FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega384d3",       FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a1",       FamilyXMEGA, ELFArchXMEGA7>;
def : Device<"atxmega128a1u",      FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"atxmega128a4u",      FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"attiny4",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny5",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny9",            FamilyTiny, ELFArchTiny>;
def : Device<"attiny10",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny20",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny40",           FamilyTiny, ELFArchTiny>;
def : Device<"attiny102",          FamilyTiny, ELFArchTiny>;
def : Device<"attiny104",          FamilyTiny, ELFArchTiny>;

