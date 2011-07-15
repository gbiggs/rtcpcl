set(rtc_name rtcpclbase)

# Library target
set(lib_srcs rtcpclbase.cpp)
include_directories(${IDL_HEADERS_DIR} ${DEPS_INCLUDE_DIRS})
add_library(${rtc_name} SHARED ${lib_srcs} ${IDL_TYPE_SOURCES})
target_link_libraries(${rtc_name} rtcpcl_pointcloud_type ${DEPS_LIBRARIES})

install(TARGETS ${rtc_name} LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    COMPONENT ${rtc_name})

