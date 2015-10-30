add_executable(servoexample
    example.cpp
)

target_link_libraries(servoexample
    l7na
    ethercat
    ${Boost_LOG_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    atomic
    pthread
)

add_executable(servotest
    test.cpp
)

target_link_libraries(servotest
    l7na
    ethercat
)
