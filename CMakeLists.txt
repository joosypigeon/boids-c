cmake_minimum_required(VERSION 3.20)

project(boids C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(boids
    src/main.c
    src/boids.c
    src/spatial_hash.c
    src/normal_random.c
)

target_include_directories(boids PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(OpenMP)

if(OpenMP_C_FOUND)
    target_link_libraries(boids PRIVATE OpenMP::OpenMP_C)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(RAYLIB REQUIRED raylib)

target_include_directories(boids PRIVATE
    ${RAYLIB_INCLUDE_DIRS}
)

target_link_directories(boids PRIVATE
    ${RAYLIB_LIBRARY_DIRS}
)

target_compile_options(boids PRIVATE
    ${RAYLIB_CFLAGS_OTHER}
    -Wall
    -Wextra
)

target_link_libraries(boids PRIVATE
    ${RAYLIB_LIBRARIES}
    GL
    m
    pthread
    dl
    rt
    X11
)
