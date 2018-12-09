set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.0.0)
project(Rex)

find_package(LLVM REQUIRED CONFIG)
message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")

find_package(Boost 1.58 COMPONENTS system filesystem serialization regex program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Sets up LLVM.
if(DEFINED ENV{CLANG_VER})
	set(CLANG_VER $ENV{CLANG_VER})

    # Get the appropriate version.
    if(CLANG_VERSION_STRING VERSION_LESS 7.0.0)
        add_definitions(-DCLANG_VER_LTE)
    else()
        add_compile_definitions(-DCLANG_VER_GT)
    endif()
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
        JSON/json.h
		XML/rapidxml.hpp
		XML/rapidxml_iterators.hpp
		XML/rapidxml_print.hpp
		XML/rapidxml_utils.hpp
		Configuration/ScenarioWalker.cpp
		Configuration/ScenarioWalker.h
        Graph/LowMemoryTAGraph.cpp
        Graph/LowMemoryTAGraph.h)

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

# Find the libraries that correspond to the LLVM components
# that we wish to use
llvm_map_components_to_libnames(llvm_libs all)

# Link against LLVM libraries
target_link_libraries(Rex ${llvm_libs})

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

