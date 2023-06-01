project(Packages)

include(FetchContent)

add_library(
    ${PROJECT_NAME}      
       
)

target_sources(
    ${PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/coppelia_sim_git_repos/b0RemoteApiBindings/cpp/b0RemoteApi.cpp
)

set(MSGPACK_BUILD_EXAMPLES FALSE CACHE BOOL "x" FORCE) 


# msgpack dependency
FetchContent_Declare(
    msgpack-c
    GIT_REPOSITORY https://github.com/msgpack/msgpack-c.git
    GIT_TAG cpp-3.2.1 # version from Dec 10, 2019 - seems it was the newest when the programme was written
)
message(STATUS "    Getting msgpack-c")

#boost dependencies in a loop
set(boost_tag_ "boost-1.65.0") # version from July 25, 2019 - last before msgpack-c
set(boost_pkg_names_ function bind preprocessor 
    config throw_exception assert integer
    static_assert type_index core type_traits mpl move
)

list(TRANSFORM boost_pkg_names_ PREPEND "https://github.com/boostorg/" OUTPUT_VARIABLE boost_pkg_reps_)
list(TRANSFORM boost_pkg_reps_ APPEND ".git")

list(TRANSFORM boost_pkg_names_ PREPEND "boost_")

message(STATUS "${boost_pkg_reps_}")
message(STATUS "${boost_pkg_names_}")

# check if legth of the two lists *_names_ and *_reps_ is the same
# if not throw error
list(LENGTH boost_pkg_names_ boost_list_size1_)
list(LENGTH boost_pkg_reps_ boost_list_size2_)
# message(STATUS "${boost_list_size1_} ${boost_list_size2_}")
if(NOT ${boost_list_size1_} EQUAL ${boost_list_size2_})
    message(FATAL_ERROR "Error with boost dependencies fetching configuration")
endif()

# FetchContent_Declare for each boost dependency
set(iterator_ 0)
while(${iterator_} LESS ${boost_list_size1_})
    list(GET boost_pkg_names_ ${iterator_} pkg_name_)
    list(GET boost_pkg_reps_ ${iterator_} pkg_repo_)

    message(STATUS "    Getting ${pkg_name_} from ${pkg_repo_}")

    FetchContent_Declare(
        ${pkg_name_}
        GIT_REPOSITORY ${pkg_repo_}
        GIT_TAG ${boost_tag_}
    )

    # advance the loop
    math(EXPR iterator_ "${iterator_} + 1")
endwhile()
message(STATUS "    msgpack-c ${boost_pkg_names_}")

# make available all packages
FetchContent_MakeAvailable(msgpack-c ${boost_pkg_names_})

# create a list of directories to include
set(boost_pkg_names_transform_ ${boost_pkg_names_})
set(iterator_ 0)
while(${iterator_} LESS ${boost_list_size1_})
    #get package name from list - append, prepend and dereference proper values
    # package source directory is in : ${package_name_lowercase_SOURCE_DIR}
    list(GET boost_pkg_names_transform_ ${iterator_} temp_path_)
    set(temp_path_ "${${temp_path_}_SOURCE_DIR}/include")

    # message(STATUS "    " ${iterator_}\ ${temp_path_})

    # remove element at iterator_ and insert at that same place updated temp_path_
    list(REMOVE_AT boost_pkg_names_transform_ ${iterator_})
    list(INSERT boost_pkg_names_transform_ ${iterator_} ${temp_path_})
    # advance the loop
    math(EXPR iterator_ "${iterator_} + 1")
endwhile()
# message(STATUS "    ${boost_pkg_names_transform_}")

# export includes to the current dir project
target_include_directories(
    ${PROJECT_NAME} 
    INTERFACE 
        ${CMAKE_CURRENT_LIST_DIR}/coppelia_sim_git_repos/b0RemoteApiBindings/cpp
    PUBLIC
        ${msgpack-c_SOURCE_DIR}/include/ #add include files from proper directory inside deps
        ${boost_pkg_names_transform_}
        ${CMAKE_CURRENT_LIST_DIR}/coppelia_sim_git_repos/bluezero/include/b0/bindings
)

# target_link_libraries(
#     ${PROJECT_NAME}
#         INTERFACE
#             msgpack-c
#             ${boost_pkg_names_}
# )

foreach(element_ ${msgpack-c_SOURCE_DIR}/include/ ${boost_pkg_names_transform_})
    message(STATUS "    add include: ${element_}")
endforeach()
