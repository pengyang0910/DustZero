# Migrated from history/212alpha_test/Zhb_TEST.
# This test depends on robot-side platform and OpenCV pieces. Keep it behind
# BUILD_LEGACY_HARDWARE_TESTS until those dependencies are adapted.

project(Zhb_TEST)

add_executable(testZhb testZhb.cpp)
target_compile_features(testZhb PRIVATE cxx_std_17)
target_link_libraries(testZhb PRIVATE xutils platform lcm opencv)
