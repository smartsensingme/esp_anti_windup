cmake_minimum_required(VERSION 3.16)
project(esp_anti_windup_tests C)
enable_testing()
set(ESP_PID_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../esp_pid" CACHE PATH "PID checkout")
set(ESP_PR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../esp_pr" CACHE PATH "PR checkout")
add_executable(test_group test_group.c ../esp_anti_windup.c
    ${ESP_PID_SOURCE_DIR}/esp_pid.c ${ESP_PR_SOURCE_DIR}/esp_pr.c)
target_include_directories(test_group PRIVATE ../include ${ESP_PID_SOURCE_DIR}/include
    ${ESP_PID_SOURCE_DIR}/tests/stubs ${ESP_PR_SOURCE_DIR}/include)
target_compile_features(test_group PRIVATE c_std_99)
target_compile_options(test_group PRIVATE -Wall -Wextra -Werror -pedantic -UNDEBUG)
target_link_libraries(test_group PRIVATE m)
add_test(NAME coordinator COMMAND test_group)
