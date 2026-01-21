# CMakeLists.txt for JaguarEngine WebSocket Server

cmake_minimum_required(VERSION 3.16)
project(jaguar_server VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find JaguarEngine
if(NOT TARGET jaguar::engine)
    find_package(JaguarEngine REQUIRED)
endif()

# Find libwebsockets
find_package(PkgConfig REQUIRED)
pkg_check_modules(LWS REQUIRED libwebsockets)

# Server executable
add_executable(jaguar_server
    main.cpp
    websocket_server.cpp
    websocket_server.h
)

target_include_directories(jaguar_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LWS_INCLUDE_DIRS}
)

target_link_libraries(jaguar_server PRIVATE
    jaguar::engine
    ${LWS_LIBRARIES}
)

target_compile_definitions(jaguar_server PRIVATE
    LWS_NO_EXTENSIONS
)

# Install
install(TARGETS jaguar_server
    RUNTIME DESTINATION bin
)
