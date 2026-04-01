
cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(tmcstepper
    DESCRIPTION "Viam C++ tmcstepper Module"
    LANGUAGES CXX
)

# Everything needs threads, and prefer -pthread if available
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(viam-cpp-sdk CONFIG REQUIRED COMPONENTS viamsdk)
find_package(libgpiod CONFIG REQUIRED)

add_executable(tmcstepper
    main.cpp
    src/tmc2209.cpp
)

target_include_directories(tmcstepper PUBLIC src)

target_link_libraries(tmcstepper
    viam-cpp-sdk::viamsdk
    libgpiod::gpiodcxx
    Threads::Threads
)

# libviam_rust_utils.a (transitive dep of viam-cpp-sdk) requires POSIX AIO from librt.
# CMAKE_CXX_STANDARD_LIBRARIES is appended at the very end of the link command,
# after all CMake-managed library ordering, which is the only reliable way to
# ensure -lrt appears after libviam_rust_utils.a.
if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_STANDARD_LIBRARIES "${CMAKE_CXX_STANDARD_LIBRARIES} -lrt")
endif()

install(
    FILES meta.json
    DESTINATION .
)

install(TARGETS tmcstepper)

set(CPACK_PACKAGE_NAME "tmcstepper")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)

