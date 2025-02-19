###########################################################################
# CMakeLists.txt generated for component Model_Lateral_Pacejka_Codegen
# Product type: executable
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(Model_Lateral_Pacejka_Codegen)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the
# environment if it is defined as an environment variable, but not as a
# CMake variable. This is to work around a bug in CMake 3.19 when the
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# Define common variables that are used within the whole project.
set(SYSLIB_PREFIX $<IF:$<AND:$<BOOL:${WIN32}>,$<NOT:$<BOOL:${MINGW}>>>,lib,>)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR .. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden
# by setting the variable from the command line, e.g.,
#
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2023B CACHE PATH "")

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# When a CMakeLists.txt defines multiple targets that have a common set of
# tool flags and preprocessor definitions, define variables containing
# data that is common to all targets defined in the file.
set(MODEL_LATERAL_PACEJKA_CODEGEN_COMMON_DEFINES -DMODEL=Model_Lateral_Pacejka_Codegen
    -DNUMST=1
    -DNCSTATES=0
    -DHAVESTDIO
    -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=0
    -DTERMFCN=1
    -DONESTEPFCN=1
    -DMAT_FILE=0
    -DMULTI_INSTANCE_CODE=1
    -DINTEGER_CODE=0
    -DMT=0)
set(MODEL_LATERAL_PACEJKA_CODEGEN_COMMON_LINKER_FLAGS -static
    -m64)

# Definition of target "Model_Lateral_Pacejka_Codegen_objects".
add_library(Model_Lateral_Pacejka_Codegen_objects OBJECT ${START_DIR}/Model_Lateral_Pacejka_Codegen_ert_rtw/Model_Lateral_Pacejka_Codegen.cpp
    ${START_DIR}/Model_Lateral_Pacejka_Codegen_ert_rtw/Model_Lateral_Pacejka_Codegen_data.cpp)

# Set properties for target "Model_Lateral_Pacejka_Codegen_objects".
set_target_properties(Model_Lateral_Pacejka_Codegen_objects PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON)

# Specify language features required for target
# "Model_Lateral_Pacejka_Codegen_objects".
target_compile_features(Model_Lateral_Pacejka_Codegen_objects PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target
# "Model_Lateral_Pacejka_Codegen_objects".
target_compile_definitions(Model_Lateral_Pacejka_Codegen_objects PRIVATE ${MODEL_LATERAL_PACEJKA_CODEGEN_COMMON_DEFINES})

# Specify include directories for target
# "Model_Lateral_Pacejka_Codegen_objects".
target_include_directories(Model_Lateral_Pacejka_Codegen_objects PUBLIC $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${START_DIR}/Model_Lateral_Pacejka_Codegen_ert_rtw>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/Model_Lateral_Pacejka_Codegen_ert_rtw>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>)

# Specify linker flags for target "Model_Lateral_Pacejka_Codegen_objects".
target_link_libraries(Model_Lateral_Pacejka_Codegen_objects PRIVATE ${MODEL_LATERAL_PACEJKA_CODEGEN_COMMON_LINKER_FLAGS})

# Specify library link dependencies for target
# "Model_Lateral_Pacejka_Codegen_objects".  CMake generator expressions
# are used to create a CMakeLists.txt file that supports multiple
# platforms with differently named system library dependencies.
target_link_libraries(Model_Lateral_Pacejka_Codegen_objects PUBLIC $<$<BOOL:${FOUND_LIBM}>:m>)


# Definition of target "Model_Lateral_Pacejka_Codegen".
add_executable(Model_Lateral_Pacejka_Codegen  ${START_DIR}/Model_Lateral_Pacejka_Codegen_ert_rtw/ert_main.cpp)

# Set properties for target "Model_Lateral_Pacejka_Codegen".
set_target_properties(Model_Lateral_Pacejka_Codegen PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON)

# Specify language features required for target
# "Model_Lateral_Pacejka_Codegen".
target_compile_features(Model_Lateral_Pacejka_Codegen PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target
# "Model_Lateral_Pacejka_Codegen".
target_compile_definitions(Model_Lateral_Pacejka_Codegen PRIVATE ${MODEL_LATERAL_PACEJKA_CODEGEN_COMMON_DEFINES})

# Specify linker flags for target "Model_Lateral_Pacejka_Codegen".
target_link_libraries(Model_Lateral_Pacejka_Codegen PRIVATE ${MODEL_LATERAL_PACEJKA_CODEGEN_COMMON_LINKER_FLAGS})

# Specify library link dependencies for target
# "Model_Lateral_Pacejka_Codegen".  CMake generator expressions are used
# to create a CMakeLists.txt file that supports multiple platforms with
# differently named system library dependencies.
target_link_libraries(Model_Lateral_Pacejka_Codegen PRIVATE Model_Lateral_Pacejka_Codegen_objects
    $<$<BOOL:${WIN32}>:kernel32>
    $<$<BOOL:${WIN32}>:ws2_32>
    $<$<BOOL:${WIN32}>:mswsock>
    $<$<BOOL:${WIN32}>:advapi32>)

# Extract DWARF debug symbols into a separate file when building
# executable programs for Apple platforms.
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    string(APPEND _dsymutilcmd "$<IF:$<CONFIG:DEBUG,RELWITHDEBINFO>,"
                "xcrun;dsymutil;$<TARGET_FILE:Model_Lateral_Pacejka_Codegen>;--flat,"
                "true"
        >)
    add_custom_command(TARGET Model_Lateral_Pacejka_Codegen
            POST_BUILD
            COMMAND "${_dsymutilcmd}"
            COMMAND_EXPAND_LISTS
            BYPRODUCTS Model_Lateral_Pacejka_Codegen.dwarf)
endif()


###########################################################################
## Target install rules
###########################################################################

# Install rule for objects of target
# "Model_Lateral_Pacejka_Codegen_objects" to allow support for SIL/PIL.
install(TARGETS Model_Lateral_Pacejka_Codegen_objects
    EXPORT Model_Lateral_Pacejka_CodegenTargets
    OBJECTS DESTINATION "Model_Lateral_Pacejka_Codegen_ert_rtw")

# Install rule for executable target "Model_Lateral_Pacejka_Codegen". The
# "OPTIONAL" keyword is required to support SIL/PIL when build of the
# final executable is suppressed.
install(TARGETS Model_Lateral_Pacejka_Codegen
    RUNTIME DESTINATION "."
    OPTIONAL)

# Install rule for Microsoft PDB debug symbols, if required.
if(CMAKE_C_LINKER_SUPPORTS_PDB)
    install(FILES $<TARGET_PDB_FILE:Model_Lateral_Pacejka_Codegen>
        DESTINATION .
        OPTIONAL
        CONFIGURATIONS Debug RelWithDebInfo)
endif()

# Install rule for Apple DWARF debug symbols, if required.
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    install(FILES $<TARGET_FILE_DIR:Model_Lateral_Pacejka_Codegen>/Model_Lateral_Pacejka_Codegen.dwarf
        DESTINATION .
        OPTIONAL
        CONFIGURATIONS Debug RelWithDebInfo)
endif()

# Write a rule that generates a wrapper around exported targets to
# preserve tokenization of "special" directories (e.g., MATLAB_ROOT).
# This avoids hard-coding absolute paths in the CMake file with the code
# used to import the targets, and avoids errors when include paths that do
# not exist in the current filesystem are defined, for example, after
# relocating code using PackNGo.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/Model_Lateral_Pacejka_Codegen.cmake" [=[include("${CMAKE_CURRENT_LIST_DIR}/Model_Lateral_Pacejka_CodegenTargets.cmake")]=] \n)
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/Model_Lateral_Pacejka_Codegen.cmake" [=[set(MODEL_LATERAL_PACEJKA_CODEGEN_OBJECTS_TOK_INC_DIRS_ALL]=] \n
    [=[    ${MATLAB_ROOT}/extern/include]=] \n
    [=[    ${MATLAB_ROOT}/simulink/include]=] \n
    [=[    ${MATLAB_ROOT}/rtw/c/src]=] \n
    [=[    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common]=] \n
    [=[    ${MATLAB_ROOT}/rtw/c/ert)]=] \n
    [=[foreach(TOKDIR_LOOP IN LISTS MODEL_LATERAL_PACEJKA_CODEGEN_OBJECTS_TOK_INC_DIRS_ALL)]=] \n
    [=[    if(IS_DIRECTORY ${TOKDIR_LOOP})]=] \n
    [=[        list(APPEND MODEL_LATERAL_PACEJKA_CODEGEN_OBJECTS_TOK_INC_DIRS ${TOKDIR_LOOP})]=] \n
    [=[    endif()]=] \n
    [=[endforeach()]=] \n
    [=[target_include_directories(Model_Lateral_Pacejka_Codegen::Model_Lateral_Pacejka_Codegen_objects INTERFACE ${MODEL_LATERAL_PACEJKA_CODEGEN_OBJECTS_TOK_INC_DIRS})]=] \n)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Model_Lateral_Pacejka_Codegen.cmake"
    DESTINATION "Model_Lateral_Pacejka_Codegen_ert_rtw/export")

# Generate and install a file that allows the targets generated from this
# CMakeLists.txt file to be imported into another project.
install(EXPORT Model_Lateral_Pacejka_CodegenTargets NAMESPACE Model_Lateral_Pacejka_Codegen:: DESTINATION Model_Lateral_Pacejka_Codegen_ert_rtw/export)

###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET Model_Lateral_Pacejka_Codegen POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created executable: $<TARGET_FILE:Model_Lateral_Pacejka_Codegen>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(Model_Lateral_Pacejka_Codegen)
endif()
