cmake_minimum_required (VERSION 2.6.0)
project(mujinchallenge)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(baseplacementoptimizer SHARED src/placementoptimizer.cpp)
set_target_properties(baseplacementoptimizer PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(baseplacementoptimizer ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} boost_system boost_thread boost_program_options)


add_executable(mujinchallenge mujinchallenge.cpp)
set_target_properties(mujinchallenge PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(mujinchallenge ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} baseplacementoptimizer boost_system boost_thread boost_program_options)
install(TARGETS mujinchallenge DESTINATION .)
