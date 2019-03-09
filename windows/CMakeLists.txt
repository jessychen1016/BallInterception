cmake_minimum_required (VERSION 3.0)

#项目名称
SET(PROJ_NAME "BlackOutTwo")
project (${PROJ_NAME})
set(ThirdParty "${CMAKE_CURRENT_LIST_DIR}/third-party")

find_package(Qt5 COMPONENTS QuickControls2)


# Qt5
set(QT_BIN "${ThirdParty}/Qt5/bin")
set(QT_INCLUDE "${ThirdParty}/Qt5/include"
			   "${ThirdParty}/Qt5/include/QtNetwork"
			   "${ThirdParty}/Qt5/include/QtCore"
	)
set(QT_LIB "${ThirdParty}/Qt5/lib")

SET(INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
				 ${CMAKE_CURRENT_LIST_DIR}/samples
                 ${CMAKE_CURRENT_LIST_DIR}/third-party/glfw-imgui/include
                 ${CMAKE_CURRENT_LIST_DIR}/third-party/
                 ${CMAKE_CURRENT_LIST_DIR}/include/actionmodule.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilterdir.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilter.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/singleton.hpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/matrix2d.h
                 ${QT_INCLUDE}
                 )

# 源文件
file(GLOB SOURCE ${CMAKE_CURRENT_LIST_DIR}/InterceptionSingleThread.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/actionmodule.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilterdir.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilter.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/matrix2d.cpp
                 )

# 链接文件
file(GLOB LINK_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/x64/realsense2.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/x64/glfw-imgui.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_world340.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_world340d.lib
					${QT_LIB}/x64/Qt5Network.lib
					${QT_LIB}/x64/Qt5Core.lib
					)

# 添加头文件
include_directories(${INCLUDE_DIRS})

# 添加源文件
add_executable(${PROJ_NAME} ${SOURCE})

# 编译输出路径
if ( MSVC )
    set_target_properties( ${PROJ_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin )
    set_target_properties( ${PROJ_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR}/bin )
    set_target_properties( ${PROJ_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR}/bin )
endif ( MSVC )

# 添加链接库
target_link_libraries(${PROJ_NAME} ${LINK_DIRS}
                                   glu32.lib
                                   opengl32.lib
                                   )