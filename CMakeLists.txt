cmake_minimum_required(VERSION 2.8.2)

project(cpplab)

# Настройка boost
set(BOOST_ROOT /usr/local)
set(BOOST_VERSION 1.54.0)
set(Boost_DEBUG OFF)                                        # Дебаг поиска библиотек Буста
set(Boost_DETAILED_FAILURE_MSG ON)                          # Детализированый репорт об ошибках
add_definitions(-DBOOST_LOG_USE_NATIVE_SYSLOG)              # Используем родной сислог
set(Boost_USE_STATIC_LIBS OFF)                              # Используем динамические библиотеки
add_definitions(-DBOOST_ALL_DYN_LINK)                       # Динамическая линковка

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/rtl)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${CMAKE_LIBRARY_PATH})

# Макросы сборки
message(STATUS "Using PATH: $ENV{PATH}")

# Защита от сборки внутри исходников
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "In-source builds are not allowed. Stopping. Don't forget to remove CMakeFiles directory and CMakeCache.txt in project root.")
endif()

if (NOT DEFINED CMAKE_BUILD_TYPE)
    message(FATAL_ERROR "CMAKE_BUILD_TYPE not defined!")
else()
    # Приводим строку тип билда к нижнему регистру
    string(TOLOWER ${CMAKE_BUILD_TYPE} PROJECT_BUILD_TYPE)
    message(STATUS "Build type: ${PROJECT_BUILD_TYPE}")
endif()

# Это наша кастомная переменная, не CMake'овская
set(PROJECT_COMPILER_VERSION "Default-Undefined")

# Общие флаги для всех сборок.
if (CMAKE_COMPILER_IS_GNUCC)
    add_definitions(
        -DDLL_PUBLIC=__attribute__\(\(visibility\(\"default\"\)\)\)
        -D__STDC_LIMIT_MACROS=1
    )
endif()
add_definitions(
    -DBOOST_PARAMETER_MAX_ARITY=7
    -DGTEST_LANG_CXX11=1
)

macro(AddBuildFlags value)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${value}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${value}")
endmacro()

macro(AddGccBuildFlags value)
    if (CMAKE_COMPILER_IS_GNUCC)
        AddBuildFlags(${value})
    endif()
endmacro()

macro(AddLinkFlags value)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${value}")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${value}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${value}")
endmacro()

macro(AddGccLinkFlags value)
    if (CMAKE_COMPILER_IS_GNUCC)
        AddLinkFlags(${value})
    endif()
endmacro()

macro(AddGccBuildAndLinkFlags value)
    if (CMAKE_COMPILER_IS_GNUCC)
        AddBuildFlags(${value})
        AddLinkFlags(${value})
    endif()
endmacro()

AddGccBuildFlags("-fPIC -I. -ggdb3 -fstrict-aliasing -Wall -Werror")
AddGccBuildFlags("-Wl,-O1 -rdynamic -Wl,--gc-sections -L${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

# Детектируем версию компилятора, проверяем и выставляем флаги компиляции и линковки в соответствии с ней.
if (CMAKE_COMPILER_IS_GNUCC)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_CXX_VERSION)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_C_VERSION)

    if (NOT GCC_CXX_VERSION OR NOT GCC_C_VERSION)
        message(FATAL_ERROR "unknown gcc/g++ version")
    endif()

    if (NOT GCC_CXX_VERSION STREQUAL GCC_C_VERSION)
       message(FATAL_ERROR "gcc/g++ version missmatch!")
    endif()

    # Убиваем перевод строки.
    string(REGEX REPLACE "\n" "" PROJECT_COMPILER_VERSION ${GCC_CXX_VERSION})

    # Для GCC требуем версию компилятора не ниже указаной.
    set(PROJECT_MIN_GCC_VERSION 4.4)
    if (PROJECT_COMPILER_VERSION VERSION_LESS ${PROJECT_MIN_GCC_VERSION})
       message(FATAL_ERROR "GCC version ${PROJECT_COMPILER_VERSION} is below ${PROJECT_MIN_GCC_VERSION}.")
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# Выставляем флаги компиляции и линковки в зависимости от типа сборки.
if (PROJECT_BUILD_TYPE MATCHES "^profile($|-.*)")
    add_definitions(-DASSERT_CHECK=0 -DNDEBUG=1)
    AddGccBuildFlags("-O2 -pg")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
else()
    if (PROJECT_BUILD_TYPE MATCHES "^debug($|-.*)")
        # TARGET_SUFFIX_DEBUG - означает, что имена целей будут содержать debug-постфикс
        add_definitions(-DASSERT_CHECK=1 -DDEBUG=1 -DTARGET_POSTFIX_DEBUG=1)
        AddGccBuildFlags("-O0 -fno-inline")
    elseif (PROJECT_BUILD_TYPE MATCHES "^release($|-.*)" OR PROJECT_BUILD_TYPE MATCHES "^relwithdebinfo($|-.*)")
        add_definitions(-DNDEBUG=1)
        AddGccBuildFlags("-O2")
        if (PROJECT_BUILD_TYPE MATCHES "^.*-check($|-.*)")
            add_definitions(-DASSERT_CHECK=1)
            AddGccBuildFlags("-rdynamic")
        else()
            AddGccBuildFlags("-fvisibility=hidden")
        endif()
    else()
        message(FATAL_ERROR "Unsupported build type: ${PROJECT_BUILD_TYPE}")
    endif()

    if (PROJECT_BUILD_TYPE MATCHES "^.*-addresssan($|-.*)")
        message(STATUS "Enabling Address Sanitizer.")
        AddGccBuildAndLinkFlags("-fsanitize=address -lm -pthread -ldl")
        AddGccBuildFlags("-fno-omit-frame-pointer -Wno-error=maybe-uninitialized")
    elseif (PROJECT_BUILD_TYPE MATCHES "^.*-threadsan($|-.*)")
        message(STATUS "Enabling Thread Sanitizer.")
        AddGccBuildFlags("-fsanitize=thread")
        AddGccLinkFlags("-pie")
    endif()

    if (PROJECT_BUILD_TYPE MATCHES "^.*-coverage($|-.*)")
        add_definitions(-DCOVERAGE=1)
        AddGccBuildFlags("-Werror -fprofile-arcs -ftest-coverage")
        AddGccLinkFlags("-fprofile-arcs -lgcov")
    endif()
endif()

include(CTest)
enable_testing()

message(STATUS "Using compiler ${CMAKE_CXX_COMPILER}, version ${PROJECT_COMPILER_VERSION}.")
get_directory_property(definitions COMPILE_DEFINITIONS)
message(STATUS "Using definitions: ${definitions}")
message(STATUS "Using C flags: ${CMAKE_C_FLAGS}")
message(STATUS "Using C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Using exe linker flags: ${CMAKE_EXE_LINKER_FLAGS}")
message(STATUS "Using module linker flags: ${CMAKE_MODULE_LINKER_FLAGS}")
message(STATUS "Using shared linker flags: ${CMAKE_SHARED_LINKER_FLAGS}")
message(STATUS "Using c_link-executable: ${CMAKE_C_LINK_EXECUTABLE}")
message(STATUS "Using cxx_link_executable: ${CMAKE_CXX_LINK_EXECUTABLE}")
message(STATUS "Using linker: ${CMAKE_LINKER}")
message(STATUS "Using ar: ${CMAKE_AR}")
message(STATUS "Using nm: ${CMAKE_NM}")
message(STATUS "Using objcopy: ${CMAKE_OBJCOPY}")
message(STATUS "Using objdump: ${CMAKE_OBJDUMP}")
message(STATUS "Using ranlib: ${CMAKE_RANLIB}")
message(STATUS "Using strip: ${CMAKE_STRIP}")

find_package(Boost ${BOOST_VERSION} REQUIRED
    atomic
    chrono
    date_time
    filesystem
    log
    log_setup
    program_options
    system
    thread
)

set(ETHERCAT_DIR ${CMAKE_SOURCE_DIR}/../ethercat)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${ETHERCAT_DIR}/include
    ${ETHERCAT_DIR}/master
)

link_directories(
    ${ETHERCAT_DIR}/lib/.libs
)

add_subdirectory(externals)
add_subdirectory(src)
