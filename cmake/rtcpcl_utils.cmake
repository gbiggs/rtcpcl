###############################################################################
# Convert a list to a space-separated string.
macro(LIST_TO_STRING _string _list)
    set(${_string})
    foreach(_item ${_list})
        set(${_string} "${${_string}} ${_item}")
    endforeach(_item)
endmacro(LIST_TO_STRING)


###############################################################################
# Filter a list by a pattern using the if() statement.
macro(FILTER_LIST _list _pattern _output)
    set(${_output})
    foreach(_item ${_list})
        if("${_item}" MATCHES ${_pattern})
            set(${_output} ${${_output}} ${_item})
        endif("${_item}" MATCHES ${_pattern})
    endforeach(_item)
endmacro(FILTER_LIST)


###############################################################################
# This macro processes a list of arguments into separate lists based on
# keywords found in the argument stream. For example:
# BUILDBLAG (misc_arg INCLUDEDIRS /usr/include LIBDIRS /usr/local/lib
#            LINKFLAGS -lthatawesomelib CFLAGS -DUSEAWESOMELIB SOURCES blag.c)
# Any other args found at the start of the stream will go into the variable
# specified in _other_args. Typically, you would take arguments to your macro
# as normal, then pass ${ARGN} to this macro to parse the dynamic-length
# arguments (so if ${_otherArgs} comes back non-empty, you've ignored something
# or the user has passed in some arguments without a keyword).
macro(PROCESS_ARGUMENTS _sources_args _include_dirs_args _lib_dirs_args _link_libs_args _link_flags_args _cflags_args _idl_args _other_args)
    set(${_sources_args})
    set(${_include_dirs_args})
    set(${_lib_dirs_args})
    set(${_link_libs_args})
    set(${_link_flags_args})
    set(${_cflags_args})
    set(${_idl_args})
    set(${_other_args})
    set(_current_dest ${_other_args})
    foreach(_arg ${ARGN})
        if(_arg STREQUAL "SOURCES")
            set(_current_dest ${_sources_args})
        elseif(_arg STREQUAL "INCLUDEDIRS")
            set(_current_dest ${_include_dirs_args})
        elseif(_arg STREQUAL "LIBDIRS")
            set(_current_dest ${_lib_dirs_args})
        elseif(_arg STREQUAL "LINKLIBS")
            set(_current_dest ${_link_libs_args})
        elseif(_arg STREQUAL "LINKFLAGS")
            set(_current_dest ${_link_flags_args})
        elseif(_arg STREQUAL "CFLAGS")
            set(_current_dest ${_cflags_args})
        elseif(_arg STREQUAL "IDL")
            set(_current_dest ${_idl_args})
        else(_arg STREQUAL "SOURCES")
            list(APPEND ${_current_dest} ${_arg})
        endif(_arg STREQUAL "SOURCES")
    endforeach(_arg)
endmacro(PROCESS_ARGUMENTS)


###############################################################################
# Creates a standalone executable for a component
# _templ The path to a suitable standalone.cpp template, such as that installed
#   with RTC:PCL.
# _name The type name of the component, as used by the manager.
# _comp_lib The library target containing the component implementation.
# _header The component's header file.
macro(MAKE_STANDALONE _templ _name _comp_lib _header)
    get_filename_component(_basename ${_templ} NAME_WE)
    set(_standalone_src ${CMAKE_CURRENT_BINARY_DIR}/${_basename}.cpp)
    set(_RTC_HEADER ${_header})
    set(_RTC_NAME ${_name})
    configure_file(${_templ} ${_standalone_src} @ONLY)
    set(_target_name ${_name}_standalone)
    add_executable(${_target_name} ${_standalone_src})
    target_link_libraries(${_target_name} ${_comp_lib})
    install(TARGETS ${_target_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT ${_name})
endmacro(MAKE_STANDALONE)


###############################################################################
macro(SET_DEST_DIRS)
    if(UNIX)
        if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
            set(LIB_INSTALL_DIR "lib64")
        else(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
            set(LIB_INSTALL_DIR "lib")
        endif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    else(UNIX)
        set(LIB_INSTALL_DIR "lib")
    endif(UNIX)
    set(INCLUDE_INSTALL_DIR "include/${PROJECT_NAME_LOWER}")
    set(SHARE_INSTALL_DIR "share/${PROJECT_NAME_LOWER}")
    set(BIN_INSTALL_DIR "bin")
endmacro(SET_DEST_DIRS)


###############################################################################
macro(DISSECT_VERSION)
    # Find version components
    string(REGEX REPLACE "^([0-9]+).*" "\\1"
        RTC_VERSION_MAJOR "${RTC_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
        RTC_VERSION_MINOR "${RTC_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1"
        RTC_VERSION_PATCH ${RTC_VERSION})
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
        RTC_VERSION_CAN ${RTC_VERSION})
endmacro(DISSECT_VERSION)

