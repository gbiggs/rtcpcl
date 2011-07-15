# Find OpenRTM-aist
#
# The following directories are searched:
# OPENRTM_ROOT (CMake variable)
# OPENRTM_ROOT (Environment variable)
#
# This sets the following variables:
# OPENRTM_FOUND - True if OpenRTM-aist was found.
# OPENRTM_INCLUDE_DIRS - Directories containing the OpenRTM-aist include files.
# OPENRTM_LIBRARIES - Libraries needed to use OpenRTM-aist.
# OPENRTM_DEFINITIONS - Compiler flags for OpenRTM-aist.
# OPENRTM_VERSION - The version of OpenRTM-aist found.
# OPENRTM_VERSION_MAJOR - The major version of OpenRTM-aist found.
# OPENRTM_VERSION_MINOR - The minor version of OpenRTM-aist found.
# OPENRTM_VERSION_PATCH - The revision version of OpenRTM-aist found.
# OPENRTM_VERSION_CAN - The candidate version of OpenRTM-aist found.

find_package(PkgConfig)
pkg_check_modules(PC_OPENRTM QUIET openrtm-aist)
pkg_check_modules(PC_COIL QUIET libcoil)

find_path(OPENRTM_INCLUDE_DIR rtm/RTC.h
    HINTS ${OPENRTM_ROOT}/include $ENV{OPENRTM_ROOT}/include
    ${PC_OPENRTM_INCLUDE_DIRS})
find_path(COIL_INCLUDE_DIR coil/config_coil.h
    HINTS ${OPENRTM_ROOT}/include $ENV{OPENRTM_ROOT}/include
    ${PC_COIL_INCLUDE_DIRS})
find_library(OPENRTM_LIBRARY RTC
    HINTS ${OPENRTM_ROOT}/lib $ENV{OPENRTM_ROOT}/lib
    ${PC_OPENRTM_LIBRARY_DIRS})
find_library(COIL_LIBRARY coil
    HINTS ${OPENRTM_ROOT}/lib $ENV{OPENRTM_ROOT}/lib
    ${PC_COIL_LIBRARY_DIRS})

set(OPENRTM_DEFINITIONS ${PC_OPENRTM_CFLAGS_OTHER} ${PC_COIL_CFLAGS_OTHER})
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIR} ${OPENRTM_INCLUDE_DIR}/rtm/idl
    ${COIL_INCLUDE_DIR})
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARY} ${COIL_LIBRARY} uuid dl pthread
    omniORB4 omnithread omniDynamic4)

#file(STRINGS ${OPENRTM_INCLUDE_DIR}/rtm/version.h OPENRTM_VERSION
#    NEWLINE_CONSUME)
set(OPENRTM_VERSION "1.1.0")
string(REGEX MATCH "version = \"([0-9]+)\\.([0-9]+)\\.([0-9]+)-?([a-zA-Z0-9]*)\""
    OPENRTM_VERSION "${OPENRTM_VERSION}")
set(OPENRTM_VERSION_MAJOR ${CMAKE_MATCH_1})
set(OPENRTM_VERSION_MINOR ${CMAKE_MATCH_2})
set(OPENRTM_VERSION_PATCH ${CMAKE_MATCH_3})
set(OPENRTM_VERSION_CAN ${CMAKE_MATCH_4})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenRTM
    REQUIRED_VARS OPENRTM_INCLUDE_DIR COIL_INCLUDE_DIR OPENRTM_LIBRARY
    COIL_LIBRARY VERSION_VAR ${OPENRTM_VERSION})

