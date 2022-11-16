cmake_minimum_required(VERSION 3.5)
project(test_pakage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

####################
find_library(CYUSBSERIAL cyusbserial third_party/cyusbserial)

set(common_files
    api/dlpc_common.h
    api/dlpc_common_private.h
    api/dlpc_common.c
    samples/cypress_i2c.h
    samples/cypress_i2c.c)

# --------------------------------------
# DLPC347x Library Configuration
# --------------------------------------
set(DLPC347x_files 
    api/dlpc34xx.h
    api/dlpc34xx_dual.h
    api/dlpc347x_internal_patterns.h
    api/dlpc34xx.c
    api/dlpc34xx_dual.c
    api/dlpc347x_internal_patterns.c
    samples/dlpc347x_samples.c
    )

add_library(dlpc347x 
    ${common_files}
    ${DLPC347x_files}
)

target_include_directories(dlpc347x PRIVATE api)
target_include_directories(dlpc347x PRIVATE samples)
target_include_directories(dlpc347x PRIVATE third_party)
target_link_libraries(dlpc347x ${CYUSBSERIAL})

###############################
add_executable(test_node src/test_node.cpp)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#####################
target_include_directories(test_node PRIVATE api)
target_include_directories(test_node PRIVATE samples)
target_include_directories(test_node PRIVATE third_party)
target_link_libraries(test_node dlpc347x m ${CYUSBSERIAL})
#############################

install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
