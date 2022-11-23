cmake_minimum_required(VERSION 3.5)

project(hive_dji_bridge)

add_compile_options(
  -std=c++17

  -Werror
  -Wall
  -Wextra
)

find_package(OpenCV)

find_package(CURL REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  dji_osdk_ros
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
  roscpp
)

# add_message_files(
# FILES
# )

# add_service_files(
# FILES
# )
generate_messages(
  DEPENDENCIES
  dji_osdk_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp dji_osdk_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB INSTALL_SCRIPTS
  scripts/*.sh
)
file(GLOB INSTALL_SERVICES
  service_files/*.service
)

file(GLOB SOURCES
  src/main.cpp
  src/application/*.cpp
  src/communication/*.cpp
  src/communication/http/*.cpp
  src/commands/*.cpp
  src/services/*.cpp
)
add_executable(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CURL_LIBRARIES}
  pthread
)

add_definitions("-D${PROJECT_NAME}_version=\"${${PROJECT_NAME}_VERSION}\"")

# Install #
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch PATTERN ".svn" EXCLUDE)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config PATTERN ".svn" EXCLUDE)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES
  ${INSTALL_SCRIPTS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(
  FILES
  ${INSTALL_SERVICES}
  DESTINATION "/etc/systemd/system"
)

# Unit Tests #
if(BUILD_UNIT_TESTS)
  add_definitions(-DBUILD_UNIT_TESTS)
  file(GLOB TESTS_LIBRARY_SOURCES
    tests/src/main.cpp

    # common
    tests/src/common/*.cpp

    # application
    src/application/*.cpp
    src/commands/*.cpp
    src/services/*.cpp
    tests/src/application/*.cpp
  )

  catkin_add_gtest(${PROJECT_NAME}_tests
    ${TESTS_LIBRARY_SOURCES}
  )

  if(TARGET ${PROJECT_NAME}_tests)
    target_link_libraries(${PROJECT_NAME}_tests
      ${catkin_LIBRARIES}
      pthread
    )
    add_dependencies(${PROJECT_NAME}_tests ${catkin_EXPORTED_TARGETS})
  endif()
endif(BUILD_UNIT_TESTS)