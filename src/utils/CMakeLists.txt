add_executable(servotech
    test.cpp
)

target_link_libraries(servotech
    l7na
    ethercat
)
