cmake_minimum_required(VERSION 3.10)

project(rknn_yolov8_seg_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# 从系统查找opencv

#set(OpenCV_DIR "/usr/share/opencv4")
#set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)
find_package(OpenCV  REQUIRED)
#find_path(MQTT_INCLUDE_DIR NAMES  mosquitto.h PATHS /usr/include)
 # 查找 MQTT 库的库文件 
#find_library(MQTT_LIBRARY NAMES mqtt PATHS /usr/lib) 
find_library(MOSQUITTO_LIB mosquitto)
find_library(ZMQ_LIBRARIES zmq)
if(NOT ZMQ_LIBRARIES)
    set(ZMQ_LIBRARIES "")
    message(STATUS "libzmq not found - ZMQ disabled (unused dead code)")
endif()
find_package(OpenSSL REQUIRED)
# 查找spdlog
find_package(spdlog REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

# 深度相机 RGBD SDK（海康 MV-DB12SME 双目结构光）
set(MV3D_INCLUDE /opt/Mv3dRgbdSDK/include)
set(MVS_INCLUDE /opt/MVS/include)
set(MV3D_LIB_DIR /opt/Mv3dRgbdSDK/lib/aarch64)
set(MVS_LIB_DIR /opt/MVS/lib/aarch64)

# rga
set(RGA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/librga)
set(LIBRGA ${RGA_PATH}/${CMAKE_SYSTEM_NAME}/aarch64/librga.a)
set(RGA_INCLUDES ${RGA_PATH}/include)
install(PROGRAMS ${RGA_PATH}/${CMAKE_SYSTEM_NAME}/aarch64/librga.so DESTINATION lib)

# only RGA on rv1106 and rk3588 support handle
if (TARGET_SOC STREQUAL "rk3588")
    add_definitions(-DLIBRGA_IM2D_HANDLE)
endif()

# RGA2 only support under 4G memory
if (ENABLE_DMA32 STREQUAL "TRUE")
    add_definitions(-DDMA_ALLOC_DMA32)
endif()

# rknn runtime
if (TARGET_SOC STREQUAL "rk3588" OR TARGET_SOC STREQUAL "rk356x")
    set(RKNN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rknpu2)
    set(LIBRKNNRT ${RKNN_PATH}/${CMAKE_SYSTEM_NAME}/aarch64/librknnrt.so)
    set(LIBRKNNRT_INCLUDES ${RKNN_PATH}/include)
endif()
install(PROGRAMS ${LIBRKNNRT} DESTINATION lib)
set(LIBRKNNRT ${LIBRKNNRT})

#dma
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/dma)

#drm
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/drm)

#zmq
include_directories(${ZMQ_INCLUDE_DIRS})

# 包含OpenSSL头文件
include_directories(${OPENSSL_INCLUDE_DIR})




# rknn_yolov8_seg_demo
add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    rknpu2/yolov8_seg.cc
    image_utils.c
)

target_link_libraries(${PROJECT_NAME}
    ${LIBRGA}
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    ${MOSQUITTO_LIB}
    ${ZMQ_LIBRARIES}  
    ${OPENSSL_LIBRARIES}
    # Boost::system
    pthread
    rt  # 添加rt库以支持sem_close等函数
    dl
    modbus
)



# 裸 ffmpeg 拉流库（read 超时中断）
# 自适应：板上若有 /usr/local/lib 的静态 .a 就打静态（不依赖运行时）；
# 没有则退回系统动态库。测试板与现场板 ffmpeg 布局不同，靠这里兼容。
if(EXISTS /usr/local/lib/libavformat.a)
    set(FFMPEG_LIBS /usr/local/lib/libavformat.a /usr/local/lib/libavcodec.a /usr/local/lib/libswscale.a /usr/local/lib/libswresample.a /usr/local/lib/libavutil.a -lm -latomic -lz -llzma -pthread)
    message(STATUS "ffmpeg: 使用静态库 /usr/local/lib/libav*.a")
else()
    find_library(AVFORMAT_LIB avformat REQUIRED)
    find_library(AVCODEC_LIB  avcodec  REQUIRED)
    find_library(AVUTIL_LIB   avutil   REQUIRED)
    find_library(SWSCALE_LIB  swscale  REQUIRED)
    find_library(SWRESAMPLE_LIB swresample)
    set(FFMPEG_LIBS ${AVFORMAT_LIB} ${AVCODEC_LIB} ${AVUTIL_LIB} ${SWSCALE_LIB})
    if(SWRESAMPLE_LIB)
        list(APPEND FFMPEG_LIBS ${SWRESAMPLE_LIB})
    endif()
    message(STATUS "ffmpeg: 使用系统动态库 ${AVFORMAT_LIB}")
endif()

# yolov8seg_videocapture_demo
# ===== 人工标注回流模块开关 =====
# ON（默认）：含 Web/遥控器人工标注回流，自动检测失败时触发标注
# OFF        ：精简生产版，不编译 manual_annotation.cc，失败直接置 0
option(ENABLE_MANUAL_ANNOTATION "启用人工标注回流模块" OFF)

set(DEMO_SOURCES
    yolov8seg_videocapture_demo.cc
    postprocess.cc
    rknpu2/yolov8_seg.cc
    image_utils.c
    hole_hybrid.cc
)
if(ENABLE_MANUAL_ANNOTATION)
    list(APPEND DEMO_SOURCES manual_annotation.cc)
    message(STATUS "人工标注回流模块: 启用")
else()
    message(STATUS "人工标注回流模块: 关闭（精简生产版）")
endif()

add_executable(yolov8seg_videocapture_demo ${DEMO_SOURCES})

target_link_libraries(yolov8seg_videocapture_demo
    ${LIBRGA}
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    ${MOSQUITTO_LIB}
    ${ZMQ_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    ${FFMPEG_LIBS}
    ${MV3D_LIB_DIR}/libMv3dRgbd.so ${MVS_LIB_DIR}/libMvCameraControl.so
    # Boost::system
    pthread
    rt  # 添加rt库以支持sem_close等函数
    dl
    modbus
)

# 添加定义（如果使用头文件模式）
target_compile_definitions(yolov8seg_videocapture_demo PRIVATE BOOST_INTERPROCESS_USE_FILE_MAPPING)
if(ENABLE_MANUAL_ANNOTATION)
    target_compile_definitions(yolov8seg_videocapture_demo PRIVATE ENABLE_MANUAL_ANNOTATION)
endif()


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RGA_INCLUDES}
    ${LIBRKNNRT_INCLUDES}
    ${MV3D_INCLUDE}
    ${MVS_INCLUDE}
)

install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(TARGETS yolov8seg_videocapture_demo DESTINATION .)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/model/bus.jpg DESTINATION model)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/model/coco_80_labels_list.txt DESTINATION model)
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)
