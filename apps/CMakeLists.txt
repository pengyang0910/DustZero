add_subdirectory(navclean)
add_subdirectory(slam)
add_subdirectory(localization)

option(BUILD_DAEMON "Build daemon process" ON)
option(BUILD_HMI "Build hmi_server process" ON)
# 交叉编译（目标平台）时默认构建 app_server；x86 本机默认关闭，避免缺少 SDK。
if(CMAKE_CROSSCOMPILING)
    option(BUILD_APP_SERVER "Build app_server process" ON)
else()
    option(BUILD_APP_SERVER "Build app_server process" OFF)
endif()
option(BUILD_MONITOR_SERVER "Build monitor_server process" ON)

if(BUILD_DAEMON)
    add_subdirectory(daemon)
endif()

if(BUILD_HMI)
    add_subdirectory(hmi)
endif()

if(BUILD_APP_SERVER)
    add_subdirectory(app_server)
endif()

if(BUILD_MONITOR_SERVER)
    add_subdirectory(monitor_server)
endif()
