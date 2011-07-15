###############################################################################
# Figure out the file names of the source outputs from omniidl.
macro(CORBA_IDL_SOURCE_OUTPUTS _result _idl _dir)
    set(${_result} ${_dir}/${_idl}SK.cc ${_dir}/${_idl}DynSK.cc)
endmacro(CORBA_IDL_SOURCE_OUTPUTS)


###############################################################################
# Figure out the file names of the header outputs from omniidl.
macro(CORBA_IDL_HEADER_OUTPUT _result _idl _dir)
    set(${_result} ${_dir}/${_idl}.hh)
endmacro(CORBA_IDL_HEADER_OUTPUT)


###############################################################################
# Compile an IDL file, placing the output in the specified directory.
# The output header file names will be placed in ${_idl}_HEADERS and appended
# to IDL_ALL_HEADERS. The output source file names will be placed in
# ${_idl}_CORBA_SOURCES and appended to CORBA_IDL_ALL_SOURCES.
macro(CORBA_COMPILE_INTF_IDL _idl_file _dir)
    get_filename_component(_idl "${_idl_file}" NAME_WE)
    execute_process(COMMAND rtm-config --idlc OUTPUT_VARIABLE _idl_compiler
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _idl_compiler)
        message(FATAL_ERROR "Could not find IDL compiler.")
    endif(NOT _idl_compiler)
    execute_process(COMMAND rtm-config --idlflags OUTPUT_VARIABLE _idlc_flags
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    separate_arguments(_idlc_flags)
    execute_process(COMMAND rtm-config --prefix OUTPUT_VARIABLE _rtm_prefix
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(RTM_IDL_DIR ${_rtm_prefix}/include/rtm/idl CACHE STRING
        "Directory containing the OpenRTM-aist IDL files.")
    set(_idl_srcs_var ${_idl}_CORBA_SRCS)
    CORBA_IDL_SOURCE_OUTPUTS(${_idl_srcs_var} ${_idl} ${_dir})
    set(_idl_hdrs_var ${_idl}_CORBA_HDRS)
    CORBA_IDL_HEADER_OUTPUT(${_idl_hdrs_var} ${_idl} ${_dir})
    file(MAKE_DIRECTORY ${_dir})
    add_custom_command(OUTPUT ${${_idl_srcs_var}} ${${_idl_hdrs_var}}
        COMMAND ${_idl_compiler} ${_idlc_flags} -I${RTM_IDL_DIR} -C${_dir}
        ${_idl_file}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DEPENDS ${_idl_file}
        COMMENT "Compiling ${_idl_file} for CORBA" VERBATIM)
    set(CORBA_IDL_ALL_SOURCES ${CORBA_IDL_ALL_SOURCES} ${${_idl_srcs_var}})
    set(CORBA_IDL_ALL_HEADERS ${CORBA_IDL_ALL_HEADERS} ${${_idl_hdrs_var}})
endmacro(CORBA_COMPILE_INTF_IDL)


###############################################################################
# Compile a set of IDL files, placing the outputs in the location
# specified by _output_dir.
macro(CORBA_COMPILE_IDL_FILES _output_dir)
    set(CORBA_IDL_ALL_SOURCES)
    set(CORBA_IDL_ALL_HEADERS)
    foreach(idl ${ARGN})
        CORBA_COMPILE_INTF_IDL(${idl} ${_output_dir})
    endforeach(idl)
    set(CORBA_IDL_HEADERS_DIR ${_output_dir})
endmacro(CORBA_COMPILE_IDL_FILES)

