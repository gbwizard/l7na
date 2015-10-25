include_directories(
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../ethercat/include
)

link_directories(
    ${CMAKE_SOURCE_DIR}/../ethercat/lib/.libs
)

add_subdirectory(l7na)
add_subdirectory(utils)
