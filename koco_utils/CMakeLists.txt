cmake_minimum_required(VERSION 3.0.2)
project(koco_utils)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

# Linting
if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_PYTHON_OPTS --ignore=W,E226,E266,E501 --show-source)
  roslint_python()  # All *.py files are being linted
  roslint_add_test()
endif()

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/geometry_averages.test)
  add_rostest(test/import.test)
  add_rostest(test/koco_message_store_proxy.test)
  add_rostest(test/test_actions.test)
  add_rostest(test/test_goal_proximity.test)
  add_rostest(test/test_ros_path_resolver.test)
endif()
