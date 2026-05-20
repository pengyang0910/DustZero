# Migrated from history/212alpha_test/XU_TEST.
# Device and upgrade tests are retained here but disabled unless
# BUILD_LEGACY_HARDWARE_TESTS is set.

project(XU_TEST)

find_package(OpenSSL)

if(OpenSSL_FOUND)
    add_executable(testUpgrade test_upgrade.cpp xlog.cpp)
    target_compile_features(testUpgrade PRIVATE cxx_std_17)
    target_link_libraries(testUpgrade PRIVATE OpenSSL::Crypto)
endif()

# test_stop.cpp, test_RecordLaser.cpp, and test_imu.cpp require robot-side LCM
# runtime and devices, so they remain source-only in the unified x86 build.
