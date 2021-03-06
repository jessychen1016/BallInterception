﻿cmake_minimum_required (VERSION 3.0)

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
                 ${CMAKE_CURRENT_LIST_DIR}/third-party/pthread
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/include/opencv
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/core/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/flann/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/imgproc/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/ml/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/objdetect/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/photo/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/video/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/dnn/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/imgcodecs/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/shape/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/videoio/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/highgui/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/superres/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/ts/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/features2d/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/calib3d/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/stitching/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/videostab/include
				 ${CMAKE_CURRENT_LIST_DIR}/third-party/opencv-3.4/modules/tracking/include

                 ${CMAKE_CURRENT_LIST_DIR}/include/actionmodule.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilterdir.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilter.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/singleton.hpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/matrix2d.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/GetImage.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/RefereeBox.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/referee.pb.h
                 ${CMAKE_CURRENT_LIST_DIR}/include/game_event.pb.h
                 ${QT_INCLUDE}
                 )

# 源文件
file(GLOB SOURCE ${CMAKE_CURRENT_LIST_DIR}/InterceptionMultiThread_Tracker.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/actionmodule.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilterdir.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/kalmanfilter.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/matrix2d.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/gotoposition.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/GetImage.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/RefereeBox.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/include/referee.pb.cc
                 ${CMAKE_CURRENT_LIST_DIR}/include/game_event.pb.cc
                 )

# 链接文件
file(GLOB LINK_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/x64/realsense2.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/x64/glfw-imgui.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_core341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_highgui341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_imgproc341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_calib3d341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_dnn341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_features2d341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_flann341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_imgcodecs341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_ml341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_objdetect341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_photo341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_python2.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_shape341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_stitching341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_superres341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_video341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_videoio341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_videostab341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/opencv_tracking341.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/protobuf/libprotobuf.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/protobuf/libprotobufD.lib
					${QT_LIB}/x64/Qt5Network.lib
					${QT_LIB}/x64/Qt5Core.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/pthreadVC2.lib
					${CMAKE_CURRENT_LIST_DIR}/lib/libpthreadGC2.a
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