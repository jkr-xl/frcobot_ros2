cmake_minimum_required(VERSION 3.8)
project(fairino_hardware_v3_9_8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -g -ggdb3)
endif()

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fairino_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Select the vendored libfairino SDK blob by target architecture.
# The vendor ships x86-64 in this repo; the aarch64 build (for Jetson / other
# ARM64 hosts) comes from FAIR-INNOVATION/fairino-cpp-sdk
# (linux/libfairino/lib/arm3588.zip) and is staged under libfairino/lib/aarch64.
# Both carry SONAME "libfairino.so.2", so only that symlink matters at runtime.
# The aarch64 blob (SDK 2.3.9) is missing 4 force-control/external-axis symbols
# that command_server.cpp calls (FT_FindSurface, FT_LinInsertion,
# FT_SpiralSearch, GetInverseKinExaxis), so ros2_cmd_server and srv_test_client
# are not built there. The ros2_control plugin needs none of them.
# ---------------------------------------------------------------------------
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(FAIRINO_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/aarch64/libfairino.so.2.3.9")
  set(FAIRINO_SDK_NAME "libfairino.so.2.3.9")
  set(FAIRINO_BUILD_CMD_SERVER OFF)
  message(STATUS "fairino: aarch64 target -> ${FAIRINO_SDK_NAME} (plugin only, no ros2_cmd_server)")
else()
  set(FAIRINO_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.8")
  set(FAIRINO_SDK_NAME "libfairino.so.2.3.8")
  set(FAIRINO_BUILD_CMD_SERVER ON)
  message(STATUS "fairino: x86-64 target -> ${FAIRINO_SDK_NAME}")
endif()

if(NOT EXISTS "${FAIRINO_SDK_LIB}")
  message(FATAL_ERROR "fairino SDK blob not found: ${FAIRINO_SDK_LIB}")
endif()


#set variable
set(SRC_DIR EXP_SRC_DIR)
file(GLOB SRC_DIR ./src/command_server.cpp ./src/command_server_node.cpp ./src/CNDE_thread.cpp ./src/global_val_def.cpp)
file(GLOB EXP_SRC_DIR ./examples/src/*.cpp)

#compile
# file(CREATE_LINK 
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0"
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
# SYMBOLIC)

# file(CREATE_LINK 
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so"
# SYMBOLIC)

if(FAIRINO_BUILD_CMD_SERVER)
  add_executable(ros2_cmd_server ${SRC_DIR})
  add_executable(srv_test_client ${EXP_SRC_DIR})
endif()
add_library(
  fairino_hardware 
  SHARED 
  ./src/fairino_hardware_interface.cpp
)

#target_compile_definitions(ros2_cmd_server PRIVATE CHN_VERSION)
#target_compile_definitions(ros2_cmd_server PRIVATE ENG_VERSION)


#add headfile source
if(FAIRINO_BUILD_CMD_SERVER)
  target_include_directories(ros2_cmd_server PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/)

  target_include_directories(srv_test_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples/include)
endif()
target_include_directories(fairino_hardware PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
$<INSTALL_INTERFACE:include/fairino_hardware>
)
# target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
# target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
# target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0)
# target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0)

# 链接库时添加rpath参数
# rpath must cover BOTH the target's own dir ($ORIGIN) and the package lib dir
# one level up ($ORIGIN/..), where the SDK blob is installed. Without $ORIGIN/..
# the plugin resolves libfairino.so.2 only via LD_LIBRARY_PATH -- and on hosts
# where python3 carries file capabilities (setcap), glibc strips
# LD_LIBRARY_PATH from every `ros2 launch` child, so the plugin fails to load.
if(FAIRINO_BUILD_CMD_SERVER)
  target_link_libraries(ros2_cmd_server "-Wl,-rpath,$ORIGIN:$ORIGIN/.." ${FAIRINO_SDK_LIB})
endif()
target_link_libraries(fairino_hardware "-Wl,-rpath,$ORIGIN:$ORIGIN/.." ${FAIRINO_SDK_LIB})
set_target_properties(fairino_hardware PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH "$ORIGIN:$ORIGIN/.."
  INSTALL_RPATH_USE_LINK_PATH FALSE)

#add dependencies
if(FAIRINO_BUILD_CMD_SERVER)
  ament_target_dependencies(ros2_cmd_server rclcpp fairino_msgs)
  ament_target_dependencies(srv_test_client rclcpp fairino_msgs)
endif()
ament_target_dependencies(fairino_hardware 
  hardware_interface
  pluginlib
  rclcpp
  fairino_msgs
  rclcpp_lifecycle 
)


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)

#install all files
#command server
if(FAIRINO_BUILD_CMD_SERVER)
  install(TARGETS ros2_cmd_server
    DESTINATION lib/${PROJECT_NAME}
  )

  #test example
  install(TARGETS srv_test_client
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

#ros2_control hardware
install(TARGETS fairino_hardware
  EXPORT fairino_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

#copy fairino SDK lib file to specfic path
file(MAKE_DIRECTORY 
"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
)

file(COPY_FILE "${FAIRINO_SDK_LIB}"
"${CMAKE_INSTALL_PREFIX}/lib/${FAIRINO_SDK_NAME}"
)

file(CREATE_LINK
"${CMAKE_INSTALL_PREFIX}/lib/${FAIRINO_SDK_NAME}"
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2"
SYMBOLIC)

# The fairino_version dispatcher derives the blob filename from the robot
# firmware version (v3.9.8 -> libfairino.so.2.3.8) and re-creates the symlinks
# itself. Provide that name too so the dispatcher stays usable on aarch64.
if(NOT FAIRINO_SDK_NAME STREQUAL "libfairino.so.2.3.8")
  file(CREATE_LINK
  "${CMAKE_INSTALL_PREFIX}/lib/${FAIRINO_SDK_NAME}"
  "${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2.3.8"
  SYMBOLIC)
endif()

file(CREATE_LINK 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2" 
"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so" 
SYMBOLIC)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#export
ament_export_targets(fairino_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
