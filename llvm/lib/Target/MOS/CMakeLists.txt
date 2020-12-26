add_llvm_component_group(MOS)

set(LLVM_TARGET_DEFINITIONS MOS.td)

tablegen(LLVM MOSGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM MOSGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM MOSGenCallingConv.inc -gen-callingconv)
tablegen(LLVM MOSGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM MOSGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM MOSGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM MOSGenGlobalISel.inc -gen-global-isel)
tablegen(LLVM MOSGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM MOSGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM MOSGenRegisterBank.inc -gen-register-bank)
tablegen(LLVM MOSGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM MOSGenSearchableTables.inc -gen-searchable-tables)
tablegen(LLVM MOSGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(MOSCommonTableGen)

add_llvm_target(MOSCodeGen
  MOSAsmPrinter.cpp
  MOSExpandPseudoInsts.cpp
  MOSFrameLowering.cpp
  MOSInstrInfo.cpp
  MOSISelDAGToDAG.cpp
  MOSISelLowering.cpp
  MOSMCInstLower.cpp
  MOSRelaxMemOperations.cpp
  MOSRegisterInfo.cpp
  MOSSubtarget.cpp
  MOSTargetMachine.cpp
  MOSTargetObjectFile.cpp

  DEPENDS
  intrinsics_gen

  LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  MC
  MOSDesc
  MOSInfo
  SelectionDAG
  Support
  Target

  ADD_TO_COMPONENT
  MOS
)

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
