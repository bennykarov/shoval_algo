cmake_minimum_required(VERSION 3.2)
project(Tracker100 VERSION 0.1.0)



link_directories(./usr/lib/x86_64-linux-gnu)



#target_link_libraries(matplotlib_cpp INTERFACE
#  Python3::Python
#  Python3::Module
#)


find_package(OpenCV 4.2 REQUIRED)
find_package (Threads REQUIRED)

set(BINARY_TYPE "debug") 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${BINARY_TYPE})


add_executable(Tracker100
	       tracker.cpp
	       mog.cpp
	       utils.cpp
	       main.cpp)


target_link_libraries(Tracker100 	${OpenCV_LIBS} 
			     	${Boost_LIBRARIES}
				${CMAKE_THREAD_LIBS_INIT})

