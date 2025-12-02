cmake_minimum_required(VERSION 3.8)
project(atividade)  # <--- MUDOU AQUI

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# --- ETAPA 01 ---
add_executable(etapa01 src/etapa01.cpp)
ament_target_dependencies(etapa01 rclcpp cg_interfaces)

# --- ETAPA 02 ---
add_executable(etapa02 src/etapa02.cpp)
ament_target_dependencies(etapa02 rclcpp cg_interfaces)

# Instala os executáveis na pasta do pacote novo
install(TARGETS
  etapa01
  etapa02
  DESTINATION lib/${PROJECT_NAME}) # <--- ${PROJECT_NAME} pega "atividade" automaticamente

ament_package()