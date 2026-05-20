# tool: teleop — 调试工具（仅本机构建）
project(teleop)
file(GLOB_RECURSE SRCS src/*.cpp)
add_executable(teleop ${SRCS})
target_include_directories(teleop PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
)
target_link_libraries(teleop PRIVATE xutils pthread)

# x86 本机构建时 LCM runtime 为 stub，teleop 通过 TCP text 直连 navclean:9001
if(NOT CMAKE_CROSSCOMPILING)
    target_compile_definitions(teleop PRIVATE FJ212_TELEOP_USE_TCP=1)
    message(STATUS "[teleop] x86 host: using TCP text remote to navclean:9001")
endif()
