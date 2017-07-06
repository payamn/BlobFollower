cmake_minimum_required(VERSION 2.6)

#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

include(FindPkgConfig)
pkg_search_module(STAGE REQUIRED stage)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${STAGE_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${STAGE_LIBRARY_DIRS})

set(PLUGIN_NAME mybot)
SET(GCC_COVERAGE_COMPILE_FLAGS "-ggdb")

# create a library module for each plugin and link libstage to each
ADD_LIBRARY(${PLUGIN_NAME} MODULE 
	src/my_bot.cpp 
	src/pid.cpp
	src/kalman.cpp
	src/utils.cpp
)

TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${STAGE_LIBRARIES} ${OpenCV_LIBRARIES})


# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES(${PLUGIN_NAME} PROPERTIES PREFIX "")


# install in <prefix>/lib
#install(TARGETS ${PLUGIN_NAME} DESTINATION lib)

