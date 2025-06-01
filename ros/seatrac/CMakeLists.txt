cmake_minimum_required(VERSION 2.8.3)
project(seatrac)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    ds_core_msgs
    rospy
)

get_cmake_property(_vars VARIABLES)
foreach(_var ${_vars})
  message(STATUS "${_var} = [${${_var}}]")
endforeach()

# Hacky: Install the seatrac package.
# Because it has the same package name as our ROS package, the two get merged.
# Neat? Awful? I can't decide.
find_package(PythonInterp REQUIRED)
add_custom_target(
  install_seatrac_api_package ALL
  COMMAND "${PYTHON_EXECUTABLE}" setup.py install
    --prefix="${CATKIN_DEVEL_PREFIX}"
    --install-lib="${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}"
    --single-version-externally-managed
    --record=/dev/null
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
)

add_message_files(
  FILES
  OutletStatus.msg
  PowerLevel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ds_core_msgs
)

catkin_package()
