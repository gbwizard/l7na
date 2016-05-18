add_executable(servoexample
    example.cpp
)

target_link_libraries(servoexample
    l7na
    ethercat
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    ${Boost_LOG_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_ATOMIC_LIBRARY}
    pthread
    rt
)

add_executable(servotest
    test.cpp
)

target_link_libraries(servotest
    l7na
    ethercat
    pthread
    rt
)
