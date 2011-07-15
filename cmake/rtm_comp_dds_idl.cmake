set(DDS_MAX_PC_SIZE 1048576 CACHE STRING
    "Maximum size of the DDS point cloud data")

###############################################################################
# Figure out the file names of the source outputs from nddsgen.
macro(DDS_IDL_SOURCE_OUTPUTS _result _idl _dir)
    set(${_result} ${_dir}/${_idl}.cxx ${_dir}/${_idl}Plugin.cxx
        ${_dir}/${_idl}Support.cxx)
endmacro(DDS_IDL_SOURCE_OUTPUTS)


###############################################################################
# Figure out the file names of the header outputs from nddsgen.
macro(DDS_IDL_HEADER_OUTPUT _result _idl _dir)
    set(${_result} ${_dir}/${_idl}.h ${_dir}/${_idl}Plugin.h
        ${_dir}/${_idl}Support.h)
endmacro(DDS_IDL_HEADER_OUTPUT)


###############################################################################
# Compile an IDL file, placing the output in the specified directory.
# The output header file names will be placed in ${_idl}_HEADERS and appended
# to IDL_ALL_HEADERS. The output source file names will be placed in
# ${_idl}_DDS_SOURCES and appended to DDS_IDL_ALL_SOURCES.
macro(DDS_COMPILE_INTF_IDL _idl_file _dir)
    set(_idl_compiler nddsgen)
    get_filename_component(_idl "${_idl_file}" NAME_WE)
    execute_process(COMMAND rtm-config --prefix OUTPUT_VARIABLE _rtm_prefix
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(RTM_IDL_DIR ${_rtm_prefix}/include/rtm/idl CACHE STRING
        "Directory containing the OpenRTM-aist IDL files.")
    set(_idl_srcs_var ${_idl}_DDS_SRCS)
    DDS_IDL_SOURCE_OUTPUTS(${_idl_srcs_var} ${_idl} ${_dir})
    set(_idl_hdrs_var ${_idl}_DDS_HDRS)
    DDS_IDL_HEADER_OUTPUT(${_idl_hdrs_var} ${_idl} ${_dir})
    file(MAKE_DIRECTORY ${_dir})
    add_custom_command(OUTPUT ${${_idl_srcs_var}} ${${_idl_hdrs_var}}
        COMMAND ${_idl_compiler} -I${RTM_IDL_DIR} -d ${_dir} -replace
        -D TARGET_IS_DDS ${_idl_file}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DEPENDS ${_idl_file}
        COMMENT "Compiling ${_idl_file} for DDS" VERBATIM)
    set(DDS_IDL_ALL_SOURCES ${DDS_IDL_ALL_SOURCES} ${${_idl_srcs_var}})
    set(DDS_IDL_ALL_HEADERS ${DDS_IDL_ALL_HEADERS} ${${_idl_hdrs_var}})
endmacro(DDS_COMPILE_INTF_IDL)


###############################################################################
# Compile a set of IDL files, placing the outputs in the location
# specified by _output_dir.
macro(DDS_COMPILE_IDL_FILES _output_dir)
    set(DDS_IDL_ALL_SOURCES)
    set(DDS_IDL_ALL_HEADERS)
    foreach(idl ${ARGN})
        DDS_COMPILE_INTF_IDL(${idl} ${_output_dir})
    endforeach(idl)
    set(DDS_IDL_HEADERS_DIR ${_output_dir})
endmacro(DDS_COMPILE_IDL_FILES)

