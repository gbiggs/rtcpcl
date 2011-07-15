# English
if(EXISTS manual.pdf)
    install(FILES manual.pdf
        DESTINATION
        ${SHARE_INSTALL_DIR}/doc/${PROJECT_NAME_LOWER}-${RTC_VERSION_MAJOR}.${RTC_VERSION_MINOR}
        COMPONENT doc)
endif(EXISTS manual.pdf)
# Japanese
if(EXISTS manual_jp.pdf)
    install(FILES manual_jp.pdf
        DESTINATION
        ${SHARE_INSTALL_DIR}/doc/${PROJECT_NAME_LOWER}-${RTC_VERSION_MAJOR}.${RTC_VERSION_MINOR}
        COMPONENT doc)
endif(EXISTS manual_jp.pdf)

