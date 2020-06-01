cmake_minimum_required(VERSION 3.0.2)
project(skyway)

catkin_python_setup()

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs)

#add_message_files(DIRECTORY)
#add_service_files(DIRECTORY)

# Actually generate the language-specific message and usecase files
#generate_messages(DEPENDENCIES std_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/mytest.test)
  add_rostest(test/infra/peer/test_infra_peer_api.test)
  add_rostest(test/infra/data/test_infra_data_api.test)
  add_rostest(test/domain/common/model/common_model.test)
  add_rostest(test/domain/peer/model/peer_model.test)
  add_rostest(test/domain/data/model/data_model.test)
  add_rostest(test/usecase/peer/test_create_request.test)
  add_rostest(test/usecase/peer/test_delete_request.test)
  add_rostest(test/usecase/peer/test_status_request.test)
  add_rostest(test/usecase/peer/test_subscribe_events.test)
endif()

