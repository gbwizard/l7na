add_executable(servotest
    test.cpp
)

target_link_libraries(servotest
    l7na
    ethercat
)
