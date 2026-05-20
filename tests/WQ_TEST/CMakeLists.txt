# Migrated from history/212alpha_test/WQ_TEST.
# These programs touch platform devices, LCM channels, navtask internals, or
# historical planner APIs, so they are built only when explicitly requested.

project(WQ_TEST)

function(add_legacy_wq_executable target)
    add_executable(${target} ${ARGN})
    target_compile_features(${target} PRIVATE cxx_std_17)
endfunction()

add_legacy_wq_executable(test_xd1yy test_xd1yy.cpp)

# The remaining historical sources are kept in this directory for migration
# completeness. They still need dependency-by-dependency adaptation before they
# can be enabled in the unified x86 build:
#   test_dstarlite.cpp, test_navmap.cpp, test_ncmms.cpp, test_GMP.cpp,
#   test_xd1y.cpp, test_wallfollow.cpp, test_trajFollow.cpp,
#   test_new_homing.cpp, test_stop.cpp, test_wheel.cpp, test_zplanning.cpp.
# Covered by default smoke tests in ../history_tools where possible.
