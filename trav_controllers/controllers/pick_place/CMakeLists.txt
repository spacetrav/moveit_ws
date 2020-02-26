install(PROGRAMS
  scripts/trav_pick_place.py
  scripts/flow_pick_place.py
  scripts/gripper.py
  scripts/move_group_python_interface_tutorial.py
  scripts/pick_and_place.py
  scripts/python_motion.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
