cmake_minimum_required(VERSION 3.0.2)
project(koco_core_flexbe_states)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

# Linting
if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_PYTHON_OPTS --ignore=W,E226,E501 --show-source)
  roslint_python(
    src/${PROJECT_NAME}/check_digital_input.py
    src/${PROJECT_NAME}/compare_kocopose_with_db.py
    src/${PROJECT_NAME}/insert_to_db.py
    src/${PROJECT_NAME}/move_joints_from_db.py
    src/${PROJECT_NAME}/move_sequence_action_state.py
    src/${PROJECT_NAME}/movej_action_state.py
    src/${PROJECT_NAME}/movel_action_state.py
    src/${PROJECT_NAME}/seq_move_joints_from_db.py
    src/${PROJECT_NAME}/seq_movej_action_state.py
    src/${PROJECT_NAME}/seq_movel_action_state.py
    src/${PROJECT_NAME}/service_set_outputs.py
    src/${PROJECT_NAME}/service_trigger_string.py
    src/${PROJECT_NAME}/wait_for_digital_input.py
    src/${PROJECT_NAME}/wait_for_machine_status.py
  )
  roslint_add_test()
endif()

# run tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/koco_core_states.test)
endif()