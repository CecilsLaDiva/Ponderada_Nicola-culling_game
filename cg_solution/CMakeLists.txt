cmake_minimum_required(VERSION 3.8)
project(cg_solution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# Executável 1: BFS (Resolver)
add_executable(resolver src/resolver.cpp)
ament_target_dependencies(resolver rclcpp cg_interfaces)
install(TARGETS resolver
  DESTINATION lib/${PROJECT_NAME})

# Executável 2: DFS Guiado (Resolver PT2)
add_executable(resolverpt2 src/resolverpt2.cpp)
ament_target_dependencies(resolverpt2 rclcpp cg_interfaces)
install(TARGETS resolverpt2
  DESTINATION lib/${PROJECT_NAME})

# Executável 3: DFS Cego (Sem Alvo PT2)
add_executable(semalvopt2 src/semalvopt2.cpp)
ament_target_dependencies(semalvopt2 rclcpp cg_interfaces)
install(TARGETS semalvopt2
  DESTINATION lib/${PROJECT_NAME})

ament_package()