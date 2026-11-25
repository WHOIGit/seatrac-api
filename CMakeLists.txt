cmake_minimum_required(VERSION 3.16)
project(seatrac)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ds_core_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(ros_dir "${CMAKE_CURRENT_SOURCE_DIR}/ros/seatrac")
rosidl_generate_interfaces(${PROJECT_NAME}
  "${ros_dir}:msg/OutletStatus.msg"
  "${ros_dir}:msg/PowerLevel.msg"
  DEPENDENCIES ds_core_msgs std_msgs
)

# rosidl supplies seatrac/__init__.py and seatrac.msg.
install(FILES seatrac/__main__.py seatrac/client.py seatrac/protocol.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
install(PROGRAMS seatrac/__main__.py DESTINATION bin RENAME seatrac)
install(PROGRAMS "${ros_dir}/src/seatrac_node.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME seatrac_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_seatrac_node "${ros_dir}/test/test_seatrac_node.py"
    APPEND_ENV "PYTHONPATH=${ros_dir}/src"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
