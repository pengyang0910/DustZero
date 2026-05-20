# Migrated from history/212alpha_test/Jephy_TEST.

project(Jephy_TEST)

add_executable(test_ts test_ts.cpp)
target_compile_features(test_ts PRIVATE cxx_std_17)
add_test(NAME test_ts COMMAND test_ts)

add_executable(test_tf test_tf.cpp)
target_link_libraries(test_tf PRIVATE navtask eigen3)
target_compile_features(test_tf PRIVATE cxx_std_17)
add_test(NAME test_tf COMMAND test_tf)

# Hardware/socket/HMI/RPC examples are intentionally kept as migrated source but
# not enabled by default: hmiClient.cpp, hmiServer.cpp, test_factory.cpp,
# test_mBrush.cpp, test_rpc_client.cpp, test_rpc_server.cpp, test_xbase.cpp,
# test_zcm_pub.cpp, test_zcm_sub.cpp, testini.cpp.
