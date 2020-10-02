cmake_minimum_required(VERSION 3.10)

add_executable(sensors_calib_app
  ${CMAKE_CURRENT_LIST_DIR}/SensorsCalibApp.cpp
)

target_link_libraries(sensors_calib_app
  PUBLIC
    ${PROJECT_NAME}
)
