project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

if (WIN32)
    set(VCPKG_INSTALL $ {_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET})
    message("Using vc_pkg install ${VC_PCKG_INSTALL}")
    include_directories({VC_PKG_INSTALL}/include)
    link_directories({VC_PKG_INSTALL}/lib)
endif()

if (NOT WIN32)
    target_link_libraries(path_planning z ssl uv uWS)
else()
    target_link_libraries(path_planning ssleay32 libuv uWS ws2_32)
    target_link_libraries(path_planning optimized zlib)
    target_link_libraries(path_planning debug zlibd)
endif()