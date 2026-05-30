cmake_minimum_required(VERSION 3.8)
project(auto_pkg)

# ---------------------------------------------------
# 1. 컴파일러 설정
# ---------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------
# 2. 필수 의존성 패키지 찾기
# ---------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) # [추가] MPC 노드의 Float32MultiArray 처리를 위해 필수
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 제어(MPC)용 수학/최적화 라이브러리
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# ---------------------------------------------------
# 3. 외부 라이브러리 (Webots, Eigen) 경로 설정
# ---------------------------------------------------
set(WEBOTS_HOME "/usr/local/webots")

include_directories(
  ${WEBOTS_HOME}/include/controller/cpp 
  ${WEBOTS_HOME}/include/controller/c
  ${Eigen3_INCLUDE_DIRS}
)
link_directories(${WEBOTS_HOME}/lib/controller)

# ---------------------------------------------------
# 4. 노드별 빌드 설정 (C++ Executables)
# ---------------------------------------------------

# [Node 1] 자율주행 차량 (Webots 통신 및 카메라)
add_executable(webots_node src/autonomous_vehicle.cpp)
ament_target_dependencies(webots_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  cv_bridge 
  OpenCV
)
target_link_libraries(webots_node CppController Controller)

# [Node 2] 이미지 전처리 (OpenCV)
add_executable(cv_node src/cv.cpp)
ament_target_dependencies(cv_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
)

# [Node 4] MPC 제어 (OsqpEigen 사용)
add_executable(mpc_node src/mpc_node.cpp)
ament_target_dependencies(mpc_node 
  rclcpp 
  geometry_msgs 
  std_msgs 
)
# mpc_test.cpp는 mpc_node.cpp 안에서 include 
target_link_libraries(mpc_node OsqpEigen::OsqpEigen)

# ---------------------------------------------------
# 5. 인스톨규칙 정의
# ---------------------------------------------------

# C++ 실행 파일
install(TARGETS
  webots_node
  cv_node
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

# (CNN)  
install(PROGRAMS
  scripts/cnn.py # 파이썬 파일이 저장된 실제 경로에 맞게 수정하세요. (예: 그냥 cnn.py라면 src/cnn.py)
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
