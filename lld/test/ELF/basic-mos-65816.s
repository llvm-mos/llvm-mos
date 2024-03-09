# REQUIRES: mos

# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosw65816 %s -o %t.o.w65816
# RUN: ld.lld %t.o.w65816 -o %t.w65816
# RUN: llvm-readobj --file-headers --sections -l %t.w65816 | FileCheck %s -check-prefixes=CHECK,W65816

# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos65el02 %s -o %t.o.65el02
# RUN: ld.lld %t.o.65el02 -o %t.65el02
# RUN: llvm-readobj --file-headers --sections -l %t.65el02 | FileCheck %s -check-prefixes=CHECK,65EL02

# returns with 42 in accumulator
.globl _start
_start:
  lda #42
  rts

// CHECK:      Format: elf32-mos
// CHECK-NEXT: Arch: mos
// CHECK-NEXT: AddressSize: 32bit
// CHECK-NEXT: LoadName: <Not found>
// CHECK-NEXT: ElfHeader {
// CHECK-NEXT:   Ident {
// CHECK-NEXT:     Magic: (7F 45 4C 46)
// CHECK-NEXT:     Class: 32-bit (0x1)
// CHECK-NEXT:     DataEncoding: LittleEndian (0x1)
// CHECK-NEXT:     FileVersion: 1
// CHECK-NEXT:     OS/ABI: SystemV (0x0)
// CHECK-NEXT:     ABIVersion: 0
// CHECK-NEXT:     Unused: (00 00 00 00 00 00 00)
// CHECK-NEXT:   }
// CHECK-NEXT:   Type: Executable (0x2)
// CHECK-NEXT:   Machine: EM_MOS (0x1966)
// CHECK-NEXT:   Version: 1
// CHECK-NEXT:   Entry: 0x100B4
// CHECK-NEXT:   ProgramHeaderOffset: 0x34
// CHECK-NEXT:   SectionHeaderOffset: 0x128
// W65816-NEXT:  Flags [
// W65816-NEXT:    EF_MOS_ARCH_6502 (0x1)
// W65816-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// W65816-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// W65816-NEXT:    EF_MOS_ARCH_W65816 (0x100)
// W65816-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
// 65EL02-NEXT:  Flags [
// 65EL02-NEXT:    EF_MOS_ARCH_6502 (0x1)
// 65EL02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// 65EL02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// 65EL02-NEXT:    EF_MOS_ARCH_65EL02 (0x200)
// 65EL02-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
// CHECK-NEXT:   ]
// CHECK-NEXT:   HeaderSize: 52
// CHECK-NEXT:   ProgramHeaderEntrySize: 32
// CHECK-NEXT:   ProgramHeaderCount: 4
// CHECK-NEXT:   SectionHeaderEntrySize: 40
// CHECK-NEXT:   SectionHeaderCount: 6
// CHECK-NEXT:   StringTableSectionIndex: 4
// CHECK-NEXT: }
// CHECK-NEXT: Sections [
// CHECK-NEXT:   Section {
// CHECK-NEXT:     Index: 0
// CHECK-NEXT:     Name:  (0)
// CHECK-NEXT:     Type: SHT_NULL (0x0)
// CHECK-NEXT:     Flags [ (0x0)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Address: 0x0
// CHECK-NEXT:     Offset: 0x0
// CHECK-NEXT:     Size: 0
// CHECK-NEXT:     Link: 0
// CHECK-NEXT:     Info: 0
// CHECK-NEXT:     AddressAlignment: 0
// CHECK-NEXT:     EntrySize: 0
// CHECK-NEXT:   }
// CHECK-NEXT:   Section {
// CHECK-NEXT:     Index: 1
// CHECK-NEXT:     Name: .text (1)
// CHECK-NEXT:     Type: SHT_PROGBITS (0x1)
// CHECK-NEXT:     Flags [ (0x6)
// CHECK-NEXT:       SHF_ALLOC (0x2)
// CHECK-NEXT:       SHF_EXECINSTR (0x4)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Address: 0x100B4
// CHECK-NEXT:     Offset: 0xB4
// CHECK-NEXT:     Size: 3
// CHECK-NEXT:     Link: 0
// CHECK-NEXT:     Info: 0
// CHECK-NEXT:     AddressAlignment: 1
// CHECK-NEXT:     EntrySize: 0
// CHECK-NEXT:   }
// CHECK-NEXT:   Section {
// CHECK-NEXT:     Index: 2
// CHECK-NEXT:     Name: .comment (7)
// CHECK-NEXT:     Type: SHT_PROGBITS (0x1)
// CHECK-NEXT:     Flags [ (0x30)
// CHECK-NEXT:       SHF_MERGE (0x10)
// CHECK-NEXT:       SHF_STRINGS (0x20)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Address: 0x0
// CHECK-NEXT:     Offset: 0xB7
// CHECK-NEXT:     Size: 8
// CHECK-NEXT:     Link: 0
// CHECK-NEXT:     Info: 0
// CHECK-NEXT:     AddressAlignment: 1
// CHECK-NEXT:     EntrySize: 1
// CHECK-NEXT:   }
// CHECK-NEXT:   Section {
// CHECK-NEXT:     Index: 3
// CHECK-NEXT:     Name: .symtab (16)
// CHECK-NEXT:     Type: SHT_SYMTAB (0x2)
// CHECK-NEXT:     Flags [ (0x0)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Address: 0x0
// CHECK-NEXT:     Offset: 0xC0
// CHECK-NEXT:     Size: 48
// CHECK-NEXT:     Link: 5
// CHECK-NEXT:     Info: 2
// CHECK-NEXT:     AddressAlignment: 4
// CHECK-NEXT:     EntrySize: 16
// CHECK-NEXT:   }
// CHECK-NEXT:   Section {
// CHECK-NEXT:     Index: 4
// CHECK-NEXT:     Name: .shstrtab (24)
// CHECK-NEXT:     Type: SHT_STRTAB (0x3)
// CHECK-NEXT:     Flags [ (0x0)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Address: 0x0
// CHECK-NEXT:     Offset: 0xF0
// CHECK-NEXT:     Size: 42
// CHECK-NEXT:     Link: 0
// CHECK-NEXT:     Info: 0
// CHECK-NEXT:     AddressAlignment: 1
// CHECK-NEXT:     EntrySize: 0
// CHECK-NEXT:   }
// CHECK-NEXT:   Section {
// CHECK-NEXT:     Index: 5
// CHECK-NEXT:     Name: .strtab (34)
// CHECK-NEXT:     Type: SHT_STRTAB (0x3)
// CHECK-NEXT:     Flags [ (0x0)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Address: 0x0
// CHECK-NEXT:     Offset: 0x11A
// CHECK-NEXT:     Size: 14
// CHECK-NEXT:     Link: 0
// CHECK-NEXT:     Info: 0
// CHECK-NEXT:     AddressAlignment: 1
// CHECK-NEXT:     EntrySize: 0
// CHECK-NEXT:   }
// CHECK-NEXT: ]
// CHECK-NEXT: ProgramHeaders [
// CHECK-NEXT:   ProgramHeader {
// CHECK-NEXT:     Type: PT_PHDR (0x6)
// CHECK-NEXT:     Offset: 0x34
// CHECK-NEXT:     VirtualAddress: 0x10034
// CHECK-NEXT:     PhysicalAddress: 0x10034
// CHECK-NEXT:     FileSize: 128
// CHECK-NEXT:     MemSize: 128
// CHECK-NEXT:     Flags [ (0x4)
// CHECK-NEXT:       PF_R (0x4)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Alignment: 4
// CHECK-NEXT:   }
// CHECK-NEXT:   ProgramHeader {
// CHECK-NEXT:     Type: PT_LOAD (0x1)
// CHECK-NEXT:     Offset: 0x0
// CHECK-NEXT:     VirtualAddress: 0x10000
// CHECK-NEXT:     PhysicalAddress: 0x10000
// CHECK-NEXT:     FileSize: 180
// CHECK-NEXT:     MemSize: 180
// CHECK-NEXT:     Flags [ (0x4)
// CHECK-NEXT:       PF_R (0x4)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Alignment: 4
// CHECK-NEXT:   }
// CHECK-NEXT:   ProgramHeader {
// CHECK-NEXT:     Type: PT_LOAD (0x1)
// CHECK-NEXT:     Offset: 0xB4
// CHECK-NEXT:     VirtualAddress: 0x100B4
// CHECK-NEXT:     PhysicalAddress: 0x100B4
// CHECK-NEXT:     FileSize: 3
// CHECK-NEXT:     MemSize: 3
// CHECK-NEXT:     Flags [ (0x5)
// CHECK-NEXT:       PF_R (0x4)
// CHECK-NEXT:       PF_X (0x1)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Alignment: 1
// CHECK-NEXT:   }
// CHECK-NEXT:   ProgramHeader {
// CHECK-NEXT:     Type: PT_GNU_STACK (0x6474E551)
// CHECK-NEXT:     Offset: 0x0
// CHECK-NEXT:     VirtualAddress: 0x0
// CHECK-NEXT:     PhysicalAddress: 0x0
// CHECK-NEXT:     FileSize: 0
// CHECK-NEXT:     MemSize: 0
// CHECK-NEXT:     Flags [ (0x6)
// CHECK-NEXT:       PF_R (0x4)
// CHECK-NEXT:       PF_W (0x2)
// CHECK-NEXT:     ]
// CHECK-NEXT:     Alignment: 0
// CHECK-NEXT:   }
// CHECK-NEXT: ]
