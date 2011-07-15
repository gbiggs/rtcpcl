set(PKG_DESC "RT Components for PCL.")
set(PKG_EXTERNAL_DEPS "openrtm-aist")
set(pkg_conf_file ${CMAKE_CURRENT_BINARY_DIR}/rtcpcl.pc)
configure_file(rtcpcl.pc.in ${pkg_conf_file} @ONLY)
install(FILES ${pkg_conf_file}
    DESTINATION ${LIB_INSTALL_DIR}/pkgconfig/ COMPONENT pkgconfig)

