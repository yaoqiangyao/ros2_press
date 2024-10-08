# (c) 2020 Copyright, Real-Time Innovations, Inc. All rights reserved.
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form only
# for use with RTI products. The Software is provided "as is", with no warranty
# of any type, including any warranty for fitness for any purpose. RTI is under
# no obligation to maintain or support the Software. RTI shall not be liable for
# any incidental or consequential damages arising out of the use or inability to
# use the software.

###############################################
# Module to build the ConnextDDS communicator #
###############################################

# Function to generate the source files from the IDL files
# Parameters:
#   - IDL_FILE: path to the IDL file
#   - OUTPUT_DIR: where the code should be generated
#
# Output:
#   - idl_<IDL Name>__connextdds_sources target: CMake target to generate
#   the source code needed from the IDL file.
#   - GENERATED_CXX_FILES: list of the C++ source files that will be generated
function(connextdds_run_codegen)
    set(options)
    set(single_value_args IDL_FILE OUTPUT_DIR)
    set(multi_value_args)
    cmake_parse_arguments(_CONNEXTDDS
        "${options}"
        "${single_value_args}"
        "${multi_value_args}"
        ${ARGN}
    )

    # Get the files to be generated by Codegen
    get_filename_component(filename ${_CONNEXTDDS_IDL_FILE} NAME)
    string(REGEX REPLACE "\\.idl" "" idl_name ${filename})

    # Source files to be generated
    set(GENERATED_CXX_FILES
        "${_CONNEXTDDS_OUTPUT_DIR}/${idl_name}.cxx"
        "${_CONNEXTDDS_OUTPUT_DIR}/${idl_name}Plugin.cxx"
        "${_CONNEXTDDS_OUTPUT_DIR}/${idl_name}Support.cxx"
    )

    # Header files to be generated
    set(GENERATED_HEADER_FILES
        "${_CONNEXTDDS_OUTPUT_DIR}/${idl_name}.h"
        "${_CONNEXTDDS_OUTPUT_DIR}/${idl_name}Plugin.h"
        "${_CONNEXTDDS_OUTPUT_DIR}/${idl_name}Support.h"
    )

    set(GENERATED_FILES
        ${GENERATED_CXX_FILES}
        ${GENERATED_HEADER_FILES}
    )

    # Create a target to generate the source files
    # Instead of creating here the source files using the COMMAND parameter
    # from add_custom_target, we create a CMake custom command. The reason is
    # the COMMAND from add_custom_target will be executed always and the
    # custom command will be executed only if the IDL file was modified
    add_custom_target(idl_${idl_name}_connextdds_sources
        MAIN_DEPENDENCY
            ${GENERATED_FILES}
    )

    # Command to create the source files
    add_custom_command(
        OUTPUT
            ${GENERATED_FILES}
        COMMAND
            ${RTICODEGEN}
                -language C++
                -replace
                -d ${_CONNEXTDDS_OUTPUT_DIR}
                ${_CONNEXTDDS_IDL_FILE}
        COMMENT "Generating the type for ${idl_name}"
        MAIN_DEPENDENCY
            ${_CONNEXTDDS_IDL_FILE}
        VERBATIM
    )

    set(GENERATED_CXX_FILES ${GENERATED_CXX_FILES} PARENT_SCOPE)
endfunction()

# Where RTI Codegenerator is located
set(RTICODEGEN "${CONNEXTDDS_DIR}/bin/rtiddsgen")
if(WIN32)
    set(RTICODEGEN "${RTICODEGEN}.bat")
endif()


set(IDL_GEN_ROOT "${CMAKE_CURRENT_BINARY_DIR}/gen/connextdds")
file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

set(IDL_GEN_CXX_LIST)
foreach(idl ${IDL_FILES})
    get_filename_component(filename ${idl} NAME)

    file(READ ${idl} filedata)
    file(WRITE ${IDL_GEN_ROOT}/${filename} "${filedata}")

    connextdds_run_codegen(
        IDL_FILE ${IDL_GEN_ROOT}/${filename}
        OUTPUT_DIR ${IDL_GEN_ROOT}
    )

    list(APPEND IDL_GEN_CXX_LIST ${GENERATED_CXX_FILES})
endforeach()

# Create a library with all the source code from the types
add_library(rti_connextdds_idl
    ${IDL_GEN_CXX_LIST}
)

# Force the output directory for the library
set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/objs/${CONNEXTDDS_ARCH}")
set_target_properties(rti_connextdds_idl
    PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${output_dir}
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${output_dir}
        ARCHIVE_OUTPUT_DIRECTORY ${output_dir}
        LIBRARY_OUTPUT_DIRECTORY_RELEASE ${output_dir}
        LIBRARY_OUTPUT_DIRECTORY_DEBUG ${output_dir}
        LIBRARY_OUTPUT_DIRECTORY ${output_dir}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${output_dir}
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${output_dir}
        RUNTIME_OUTPUT_DIRECTORY ${output_dir}
)

set(lib_type "RELEASE")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(lib_type "DEBUG")
endif()

# Link the ConnextDDS libraries
target_link_libraries(rti_connextdds_idl
    PUBLIC
        ${CONNEXTDDS_C_API_LIBRARIES_${lib_type}_SHARED}
        ${CONNEXTDDS_CPP_API_LIBRARIES_${lib_type}_SHARED}
        ${CONNEXTDDS_EXTERNAL_LIBS}
)

# Include the ConnextDDS directories and the header files from the generated
# code
target_include_directories(rti_connextdds_idl
    PUBLIC
        ${CONNEXTDDS_INCLUDE_DIRS}
        ${IDL_GEN_ROOT}/..
)

# The variable that contains the definitions was changed in ConnextDDS 6.0.0
# Also, this change is needed to avoid a bug that affects the find package
# CMake script of Connext DDS 5.3.1
if(RTICONNEXTDDS_VERSION VERSION_LESS "6.0.0")
    # This will just affect to architectures using GCC in systems of 32 or 64
    # bits
    string(REPLACE "-m64 " ""
        FIXED_CONNEXTDDS_DEFINITIONS
        "${CONNEXTDDS_DEFINITIONS}"
    )
    string(REPLACE "-m32 " ""
        FIXED_CONNEXTDDS_DEFINITIONS
        "${FIXED_CONNEXTDDS_DEFINITIONS}"
    )
else()
    set(FIXED_CONNEXTDDS_DEFINITIONS ${CONNEXTDDS_COMPILE_DEFINITIONS})
endif()

# Compile definitions for the library
target_compile_definitions(rti_connextdds_idl
    PUBLIC
        ${FIXED_CONNEXTDDS_DEFINITIONS}
)
