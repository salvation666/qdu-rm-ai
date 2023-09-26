cmake_minimum_required(VERSION 3.12)
project(module_compensator)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})
target_link_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_BINARY_DIR}/om.out/om/src/
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    module_component
    task_aim_object
    OM
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<TARGET_PROPERTY:OneMessage,INTERFACE_INCLUDE_DIRECTORIES>
)
