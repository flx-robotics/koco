cmake_minimum_required(VERSION 3.0.2)
project(koco_services)
find_package(catkin REQUIRED)
catkin_package()

set(python_scripts
  nodes/pilz_set_robot_tool.py
  nodes/plan_and_store.py
)

install(PROGRAMS
  ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Linting
if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_PYTHON_OPTS --ignore=W,E226,E266,E501 --show-source)
  roslint_python(
    ${python_scripts}
  )
  roslint_add_test()
endif()