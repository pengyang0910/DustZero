# tests — 单元测试入口
add_subdirectory(Jephy_TEST)

option(BUILD_LEGACY_HARDWARE_TESTS "Build migrated hardware/platform history tests" OFF)
if(BUILD_LEGACY_HARDWARE_TESTS)
    add_subdirectory(WQ_TEST)
    add_subdirectory(XU_TEST)
    add_subdirectory(Zhb_TEST)
endif()

if(WIN32)
    add_subdirectory(WQ_WIN32_TEST)
endif()
