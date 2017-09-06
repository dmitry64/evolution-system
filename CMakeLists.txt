cmake_minimum_required(VERSION 2.8)

project(evolution-system)

find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG)

set(SOURCES "main.cxx"
            "simulator.cxx"
            "simulationsystem.cxx"
            "actor.cxx"
             )

set(HEADERS "simulator.hpp"
            "simulationsystem.hpp"
            "actor.hpp"
             )

include_directories(${CHRONO_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ${CHRONO_LIBRARIES} pthread)
