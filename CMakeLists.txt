cmake_minimum_required(VERSION 3.1)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-fPIC")
project(fcl2rct)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(fcl2rct_lib STATIC
  src/broadphase_bruteforce_vct.cpp
  src/FCLdistance2.cpp
  src/utils.cpp
	)

add_executable(test_fcl
	src/test.cpp
)

target_link_libraries(test_fcl PUBLIC
  fcl2rct_lib
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libfcl.so.0.5
  /usr/local/lib/ros_control_toolbox/Release/librct_lib.a
)


set(include_dest "include/${PROJECT_NAME}")
set(lib_dest_generic "lib/${PROJECT_NAME}")
set(lib_dest ${lib_dest_generic}/${CMAKE_BUILD_TYPE})

#rct and fcl include directories are missing
target_include_directories(fcl2rct_lib PUBLIC
                       $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include/fcl2rct> 
                       $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> 
                       $<BUILD_INTERFACE:${orocos_kdl_INCLUDE_DIRS}> 
                       $<BUILD_INTERFACE:${kdl_parser_INCLUDE_DIRS}> 
                       $<INSTALL_INTERFACE:${include_dest}> 
                       $<INSTALL_INTERFACE:${lib_dest}>
                       ) 

# Install                       
set(header_path "${${PROJECT_NAME}_SOURCE_DIR}/include/fcl2rct")
file(GLOB_RECURSE header "${header_path}/*.h")

install(FILES ${header} DESTINATION "${include_dest}")
install(FILES fcl2rct_lib-config.cmake DESTINATION ${lib_dest_generic})
install(TARGETS fcl2rct_lib EXPORT fcl2rct_lib-export DESTINATION "${lib_dest}")
install(EXPORT fcl2rct_lib-export DESTINATION "${lib_dest}")