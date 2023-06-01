project(ControlApp)

add_executable(${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
        PRIVATE
            ./src/main.cpp
)

target_include_directories(
    ${PROJECT_NAME}
        PRIVATE
            ./src/   
)

target_link_libraries(
    ${PROJECT_NAME}
        PUBLIC
            Packages
            b0
)

# set path to .so library
target_link_options(
    ${PROJECT_NAME}
        PRIVATE
            -L${CMAKE_CURRENT_LIST_DIR}/../_vrep_shared_lib
)

