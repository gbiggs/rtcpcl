set (conf_in_files
    benchmark
    cuboid
    loader
    normals
    openni
    passthrough
    planesegmentation
    rainbowtube
    rtc
    saver
    viewer
    voxelfilter)

foreach(in_file ${conf_in_files})
    configure_file(${in_file}.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${in_file}.conf)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${in_file}.conf
        DESTINATION ${SHARE_INSTALL_DIR}/examples/conf
        COMPONENT examples)
endforeach(in_file)

install(FILES qos.xml DESTINATION ${SHARE_INSTALL_DIR}/examples/conf
    COMPONENT examples)

