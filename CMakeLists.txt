set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 2.8.4)
project(Rex)

find_package(Boost 1.58 COMPONENTS system filesystem serialization regex program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Sets up LLVM.
if(DEFINED ENV{CLANG_VER})
	set(CLANG_VER $ENV{CLANG_VER})
else()
	message( FATAL_ERROR "The CLANG_VER environment variable is not set. Please set it before compiling Rex." )
endif()
if(DEFINED ENV{LLVM_PATH})
	set(LLVM_PATH $ENV{LLVM_PATH})
else()
	message( FATAL_ERROR "The LLVM_PATH environment variable is not set. Please set it before compiling Rex." )
endif()

link_directories(${LLVM_PATH}/lib)
include_directories(${LLVM_PATH}/include)

add_definitions(
        -D__STDC_LIMIT_MACROS
        -D__STDC_CONSTANT_MACROS
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        Graph/TAGraph.cpp
        Graph/TAGraph.h
        Graph/RexNode.cpp
        Graph/RexNode.h
        Graph/RexEdge.cpp
        Graph/RexEdge.h
        Driver/RexMaster.cpp
        Driver/RexHandler.cpp
        Driver/RexHandler.h
        Walker/MinimalROSWalker.cpp
        Walker/MinimalROSWalker.h
        Walker/ParentWalker.cpp
        Walker/ParentWalker.h
        Walker/ROSWalker.cpp
        Walker/ROSWalker.h
        Walker/ROSConsumer.cpp
        Walker/ROSConsumer.h
		JSON/jsoncpp.cpp
        JSON/json-forwards.h
        JSON/json.h)

add_executable(Rex ${SOURCE_FILES})

target_link_libraries(Rex
        clangFrontend
        clangSerialization
        clangDriver
        clangParse
        clangSema
        clangAnalysis
        clangAST
        clangASTMatchers
        clangBasic
        clangEdit
        clangLex
        clangTooling
        )

target_link_libraries(Rex
        LLVMLTO
        LLVMPasses
        LLVMObjCARCOpts
        LLVMMIRParser
        LLVMSymbolize
        LLVMDebugInfoPDB
        LLVMDebugInfoDWARF
        LLVMCoverage
        LLVMTableGen
        LLVMOrcJIT
        LLVMXCoreDisassembler
        LLVMXCoreCodeGen
        LLVMXCoreDesc
        LLVMXCoreInfo
        LLVMXCoreAsmPrinter
        LLVMSystemZDisassembler
        LLVMSystemZCodeGen
        LLVMSystemZAsmParser
        LLVMSystemZDesc
        LLVMSystemZInfo
        LLVMSystemZAsmPrinter
        LLVMSparcDisassembler
        LLVMSparcCodeGen
        LLVMSparcAsmParser
        LLVMSparcDesc
        LLVMSparcInfo
        LLVMSparcAsmPrinter
        LLVMPowerPCDisassembler
        LLVMPowerPCCodeGen
        LLVMPowerPCAsmParser
        LLVMPowerPCDesc
        LLVMPowerPCInfo
        LLVMPowerPCAsmPrinter
        LLVMNVPTXCodeGen
        LLVMNVPTXDesc
        LLVMNVPTXInfo
        LLVMNVPTXAsmPrinter
        LLVMMSP430CodeGen
        LLVMMSP430Desc
        LLVMMSP430Info
        LLVMMSP430AsmPrinter
        LLVMMipsDisassembler
        LLVMMipsCodeGen
        LLVMMipsAsmParser
        LLVMMipsDesc
        LLVMMipsInfo
        LLVMMipsAsmPrinter
        LLVMLanaiDisassembler
        LLVMLanaiCodeGen
        LLVMLanaiAsmParser
        LLVMLanaiDesc
        LLVMLanaiAsmPrinter
        LLVMLanaiInfo
        LLVMHexagonDisassembler
        LLVMHexagonCodeGen
        LLVMHexagonAsmParser
        LLVMHexagonDesc
        LLVMHexagonInfo
        LLVMBPFDisassembler
        LLVMBPFCodeGen
        LLVMBPFDesc
        LLVMBPFInfo
        LLVMBPFAsmPrinter
        LLVMARMDisassembler
        LLVMARMCodeGen
        LLVMARMAsmParser
        LLVMARMDesc
        LLVMARMInfo
        LLVMARMAsmPrinter
        LLVMAMDGPUDisassembler
        LLVMAMDGPUCodeGen
        LLVMAMDGPUAsmParser
        LLVMAMDGPUDesc
        LLVMAMDGPUInfo
        LLVMAMDGPUAsmPrinter
        LLVMAMDGPUUtils
        LLVMAArch64Disassembler
        LLVMAArch64CodeGen
        LLVMAArch64AsmParser
        LLVMAArch64Desc
        LLVMAArch64Info
        LLVMAArch64AsmPrinter
        LLVMAArch64Utils
        LLVMObjectYAML
        LLVMLibDriver
        LLVMOption
        LLVMX86Disassembler
        LLVMX86AsmParser
        LLVMX86CodeGen
        LLVMGlobalISel
        LLVMSelectionDAG
        LLVMAsmPrinter
        LLVMDebugInfoCodeView
        LLVMDebugInfoMSF
        LLVMX86Desc
        LLVMMCDisassembler
        LLVMX86Info
        LLVMX86AsmPrinter
        LLVMX86Utils
        LLVMMCJIT
        LLVMLineEditor
        LLVMInterpreter
        LLVMExecutionEngine
        LLVMRuntimeDyld
        LLVMCodeGen
        LLVMTarget
        LLVMCoroutines
        LLVMipo
        LLVMInstrumentation
        LLVMVectorize
        LLVMScalarOpts
        LLVMLinker
        LLVMIRReader
        LLVMAsmParser
        LLVMInstCombine
        LLVMTransformUtils
        LLVMBitWriter
        LLVMAnalysis
        LLVMProfileData
        LLVMObject
        LLVMMCParser
        LLVMMC
        LLVMBitReader
        LLVMCore
        LLVMSupport
        LLVMDemangle
        )

include(FindCurses)
target_link_libraries(Rex
        pthread
        z
        dl
        crypto
        ${CURSES_LIBRARIES}
        )

target_link_libraries(Rex
        ${Boost_LIBRARIES}
        )

add_custom_command(TARGET Rex PRE_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
		${LLVM_PATH}/lib/clang/${CLANG_VER}/include ${CMAKE_SOURCE_DIR}/include)
add_custom_command(TARGET Rex POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/include $<TARGET_FILE_DIR:Rex>/include)
add_custom_command(TARGET Rex POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
	${CMAKE_SOURCE_DIR}/REX_IGNORE.db $<TARGET_FILE_DIR:Rex>/REX_IGNORE.db)

