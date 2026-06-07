# 创建可执行文件
add_executable(calibrate camera_calibration.cpp)

# 链接 OpenCV 库
target_link_libraries(calibrate ${OpenCV_LIBS})

# 添加 OpenCV 头文件路径
target_include_directories(calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})

# 设置输出目录（可选）
set_target_properties(calibrate PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)