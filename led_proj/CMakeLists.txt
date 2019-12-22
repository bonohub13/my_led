cmake_minimum_required(VERSION 3.1)

project(LED_clock)

# find_package(names_of_added_packages REQUIRED)

# add_library(name_of_dir SHARED name_of_files)

add_executable(led_clock src/main.cpp)

# ----- only use this when add_library is used -----
# target_link_libraries(executable_file PUBLIC ${PROJECT_SOURCE_DIR}/include)
