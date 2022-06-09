cmake_minimum_required(VERSION 3.0.2)
project(koco_nodes)

find_package(catkin REQUIRED)

catkin_package()

set(python_scripts
  nodes/database_publisher.py
  nodes/executed_trajectory_watchdog.py
  nodes/flexbe_current_state.py
  nodes/hardware_ready.py
  nodes/koco_status_driver.py
  nodes/multiple_counters_monitor.py
)

catkin_install_python(PROGRAMS
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