# Migrated from history/212alpha_test/WQ_WIN32_TEST.

project(WQ_WIN32_TEST)

add_executable(test_curveTrajGenerate test_curveTrajGenerate.cpp)
target_compile_features(test_curveTrajGenerate PRIVATE cxx_std_17)
target_link_libraries(test_curveTrajGenerate PRIVATE xutils navtask platform opencv)

add_executable(test_traj_gui test_traj_gui.cpp)
target_compile_features(test_traj_gui PRIVATE cxx_std_17)
target_link_libraries(test_traj_gui PRIVATE opencv)
