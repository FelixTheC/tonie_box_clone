cmake_minimum_required(VERSION 3.5)
project(cpp_pkg)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

include_directories(include)
include_directories(/usr/include)
include_directories(/usr/local/include)
include_directories(${WIRINGPI_INCLUDE_DIRS}) 

link_directories(include 
                /usr/include 
                /usr/local/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

add_library(MFRC522 src/MFRC522.cpp)
add_library(mfrc522_delegate src/mfrc522_delegate.cpp)

add_executable(mfrc522_reader src/mfrc522_reader.cpp)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(volume_down_button src/volume_down_button.cpp)
ament_target_dependencies(volume_down_button rclcpp example_interfaces)

add_executable(volume_up_button src/volume_up_button.cpp)
ament_target_dependencies(volume_up_button rclcpp example_interfaces)

add_executable(pause_button src/pause_button.cpp)
ament_target_dependencies(pause_button rclcpp example_interfaces)

target_link_libraries(mfrc522_reader 
                      MFRC522 
                      mfrc522_delegate 
                      -lbcm2835
                      ${WIRINGPI_LIBRARIES}
                    )

target_link_libraries(volume_down_button
                      -lbcm2835
                      ${WIRINGPI_LIBRARIES}
                      )
target_link_libraries(volume_up_button
                      -lbcm2835
                      ${WIRINGPI_LIBRARIES}
                      )
target_link_libraries(pause_button
                     -lbcm2835
                     ${WIRINGPI_LIBRARIES}
                     )

ament_target_dependencies(mfrc522_reader rclcpp example_interfaces)


install(TARGETS
        mfrc522_reader
        volume_down_button
        volume_up_button
        pause_button
        smartphone
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
