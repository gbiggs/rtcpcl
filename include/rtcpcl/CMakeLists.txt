include(TestBigEndian)
TEST_BIG_ENDIAN(RTCPCL_IS_BIG_ENDIAN)

set(config_h ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(config.h.in ${config_h})
install(FILES ${config_h} pc_mgmt.h pc_traits.h rtcpclbase.h
    DESTINATION ${INCLUDE_INSTALL_DIR} COMPONENT typelib)

