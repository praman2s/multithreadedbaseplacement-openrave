cmake_minimum_required (VERSION 2.6.0)
project(mujinchallenge)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_BOOST_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(baseplacementoptimizer SHARED src/placementoptimizer.cpp)
set_target_properties(baseplacementoptimizer PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(baseplacementoptimizer ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} boost_system boost_thread boost_program_options)


add_executable(testfunction testfunction.cpp)
set_target_properties(testfunction PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(testfunction ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} baseplacementoptimizer boost_system boost_thread boost_program_options)
install(TARGETS testfunction DESTINATION .)

#add_executable(trial baseplacement_threaded_ln.cpp)
#set_target_properties(trial PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
#target_link_libraries(trial ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} boost_system boost_thread boost_program_options)
#install(TARGETS trial DESTINATION .)
