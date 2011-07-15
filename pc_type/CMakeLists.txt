set(idl_files pointcloud.idl)
# Install the IDL files
install(FILES ${idl_files} DESTINATION ${INCLUDE_INSTALL_DIR}/idl
    COMPONENT typelib)

if(CORBA_SUPPORT)
    message(STATUS "Including CORBA support")
    include(${PROJECT_SOURCE_DIR}/cmake/rtm_comp_corba_idl.cmake)
    CORBA_COMPILE_IDL_FILES(${CMAKE_CURRENT_BINARY_DIR} ${idl_files})
    set(IDL_HEADERS_DIR ${CORBA_IDL_HEADERS_DIR} PARENT_SCOPE)
endif(CORBA_SUPPORT)

if(DDS_SUPPORT)
    message(STATUS "Including DDS support")
    include(${PROJECT_SOURCE_DIR}/cmake/rtm_comp_dds_idl.cmake)
    DDS_COMPILE_IDL_FILES(${CMAKE_CURRENT_BINARY_DIR} ${idl_files})
    set(IDL_HEADERS_DIR ${IDL_INCLUDE_DIR} ${DDS_IDL_HEADERS_DIR} PARENT_SCOPE)
endif(DDS_SUPPORT)

include_directories(${OPENRTM_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
    ${DDS_INCLUDE_DIRS} ${CORBA_IDL_HEADERS_DIR} ${DDS_IDL_HEADERS_DIR}
    ${PROJECT_BINARY_DIR}/include)

# Compile the point cloud type library
set(pc_lib rtcpcl_pointcloud_type)
add_library(${pc_lib} SHARED ${CORBA_IDL_ALL_SOURCES} ${DDS_IDL_ALL_SOURCES}
    pc_mgmt.cpp pc_traits.cpp)
# Install the library so other components can use this type
install(TARGETS ${pc_lib} LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    COMPONENT typelib)
install(FILES ${CORBA_IDL_ALL_HEADERS} ${DDS_IDL_ALL_HEADERS}
    DESTINATION ${INCLUDE_INSTALL_DIR}/idl COMPONENT typelib)

