cmake_minimum_required(VERSION 3.8)
project(cg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# flags padrao
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# executaveis
add_executable(resolver src/resolver.cpp)
ament_target_dependencies(resolver rclcpp cg_interfaces)

add_executable(resolverpt2 src/resolverpt2.cpp)
ament_target_dependencies(resolverpt2 rclcpp cg_interfaces)

add_executable(semalvopt2 src/semalvopt2.cpp)
ament_target_dependencies(semalvopt2 rclcpp cg_interfaces)

install(TARGETS
  resolver
  resolverpt2
  semalvopt2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()