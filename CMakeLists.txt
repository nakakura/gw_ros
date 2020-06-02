cmake_minimum_required(VERSION 3.0.2)
project(skyway)

catkin_python_setup()

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED
  COMPONENTS rospy message_generation std_msgs)

#add_message_files(DIRECTORY)
add_service_files(
  FILES
  SkyWayEvents.srv
  SkyWayControls.srv
)

# Actually generate the language-specific message and usecase files
generate_messages(DEPENDENCIES std_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/infra/peer/runner.test)
  add_rostest(test/infra/data/runner.test)
  add_rostest(test/domain/common/model/runner.test)
  add_rostest(test/domain/peer/model/runner.test)
  add_rostest(test/domain/data/model/runner.test)
  add_rostest(test/usecase/peer/runner.test)
  add_rostest(test/usecase/data/runner.test)
endif()

