cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_executable(srl_projection_test
	runTests.cpp
	TestPinholeCamera.cpp
	TestOusterLidar.cpp
)
target_include_directories(srl_projection_test
  PUBLIC
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(srl_projection_test
	SRL::Projection
	${GTEST_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

add_test(NAME srl_projection_test
	COMMAND srl_projection_test)

