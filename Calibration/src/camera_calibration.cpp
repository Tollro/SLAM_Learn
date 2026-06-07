#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

int main()
{
    // ================== 1. 棋盘格参数 ==================
    cv::Size patternSize(10, 7);      // 内部角点数 (列, 行)
    float squareSize  = 0.021f;       // 格子边长，单位：米 (21mm)

    // ================== 2. 读取所有图片路径 ==================
    std::string imageFolder = "/home/vboxuser/WorkSpace/Temps/Dormitory_SLAM/Calib_Images/";
    std::vector<cv::String> imagePaths;
    
    // 支持多种格式
    cv::glob(imageFolder + "*.JPG", imagePaths, false);

    if (imagePaths.empty())
    {
        cv::glob(imageFolder + "*.jpg", imagePaths, true);
    }
    if (imagePaths.empty())
    {
        cv::glob(imageFolder + "*.png", imagePaths, true);
    }
    if (imagePaths.empty())
    {
        cv::glob(imageFolder + "*.PNG", imagePaths, true);
    }
    
    if (imagePaths.empty())
    {
        std::cerr << "未找到图片，请检查路径: " << imageFolder << std::endl;
        return -1;
    }
    
    std::cout << "找到 " << imagePaths.size() << " 张图片\n" << std::endl;

    // ================== 3. 生成世界坐标点 ==================
    std::vector<cv::Point3f> objectCorners;
    for (int i = 0; i < patternSize.height; ++i)
        for (int j = 0; j < patternSize.width; ++j)
            objectCorners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));

    // ================== 4. 逐张提取角点（按任意键继续） ==================
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    cv::Size imageSize;
    
    std::cout << "========================================" << std::endl;
    std::cout << "开始检测角点，按任意键处理下一张" << std::endl;
    std::cout << "按 ESC 键跳过当前图片" << std::endl;
    std::cout << "按 q 键结束采集并开始标定" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    for (size_t idx = 0; idx < imagePaths.size(); ++idx)
    {
        std::string path = imagePaths[idx];
        std::cout << "[" << (idx+1) << "/" << imagePaths.size() << "] 处理: " 
                  << path.substr(path.find_last_of("/")+1) << std::endl;
        
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (img.empty())
        {
            std::cout << "  ✗ 无法读取，跳过" << std::endl;
            continue;
        }

        // 检查图像尺寸一致性
        if (imageSize.width == 0) {
            imageSize = img.size();
        } else if (imageSize != img.size()) {
            std::cout << "  ✗ 图片尺寸不一致，跳过" << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, patternSize, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH + 
                                               cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            imagePoints.push_back(corners);
            objectPoints.push_back(objectCorners);
            
            std::cout << "  ✓ 角点检测成功 (已采集 " << imagePoints.size() << " 张)" << std::endl;
            
            // 绘制并显示角点
            cv::Mat displayImg;
            cv::cvtColor(img, displayImg, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(displayImg, patternSize, corners, found);
            
            // 添加文字信息
            std::string info = "Image " + std::to_string(idx+1) + "/" + 
                              std::to_string(imagePaths.size()) + 
                              " - Press any key to continue";
            cv::putText(displayImg, info, cv::Point(30, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::namedWindow("Corners Detection", cv::WINDOW_NORMAL);
            cv::resizeWindow("Corners Detection", 800, 600);
            cv::imshow("Corners Detection", displayImg);
            
            // 等待按键
            int key = cv::waitKey(0);  // 等待按键才继续
            if (key == 27) {  // ESC 键：移除最后一个结果
                imagePoints.pop_back();
                objectPoints.pop_back();
                std::cout << "  ← 已撤销（ESC）" << std::endl;
            } else if (key == 'q' || key == 'Q') {  // q 键：结束采集
                std::cout << "  结束采集" << std::endl;
                cv::destroyAllWindows();
                break;
            }
        }
        else
        {
            std::cout << "  ✗ 未检测到角点" << std::endl;
            
            // 显示失败的图片
            cv::Mat displayImg;
            cv::cvtColor(img, displayImg, cv::COLOR_GRAY2BGR);
            cv::putText(displayImg, "Detection FAILED - Press any key to continue", 
                       cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            cv::imshow("Corners Detection", displayImg);
            cv::waitKey(0);
        }
        
        std::cout << std::endl;
    }
    
    cv::destroyAllWindows();

    if (imagePoints.size() < 3)
    {
        std::cerr << "至少需要3张有效图像才能进行标定！当前仅有 " << imagePoints.size() << " 张" << std::endl;
        return -1;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "共采集 " << imagePoints.size() << " 张有效图像" << std::endl;
    std::cout << "开始标定..." << std::endl;
    std::cout << "========================================\n" << std::endl;

    // ================== 5. 相机标定 ==================
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs   = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    int flags = cv::CALIB_FIX_K3;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                     cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    // ================== 6. 输出结果 ==================
    std::cout << "\n======== 标定完成 ========" << std::endl;
    std::cout << "图像尺寸: " << imageSize.width << " x " << imageSize.height << std::endl;
    std::cout << "重投影误差 RMS: " << rms << " 像素" << std::endl;
    std::cout << "\n内参矩阵 K:\n" << cameraMatrix << std::endl;
    std::cout << "\n畸变系数 (k1,k2,p1,p2[,k3]):\n" << distCoeffs.t() << std::endl;
    
    // ================== 7. 评估每张图的重投影误差 ==================
    double totalErr = 0.0;
    int totalPoints = 0;
    std::vector<double> perViewErrors;
    
    std::cout << "\n各图像重投影误差:" << std::endl;
    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projectedPoints);
        double err = cv::norm(imagePoints[i], projectedPoints, cv::NORM_L2);
        int n = objectPoints[i].size();
        double meanErr = std::sqrt(err*err/n);
        perViewErrors.push_back(meanErr);
        totalErr += err*err;
        totalPoints += n;
        std::cout << "  图像 " << i << ": " << meanErr << " 像素" << std::endl;
    }
    
    std::cout << "\n整体平均重投影误差: " << std::sqrt(totalErr/totalPoints) << " 像素" << std::endl;
    
    // 找出误差最大的图像
    auto maxErr = std::max_element(perViewErrors.begin(), perViewErrors.end());
    auto minErr = std::min_element(perViewErrors.begin(), perViewErrors.end());
    int maxIdx = std::distance(perViewErrors.begin(), maxErr);
    int minIdx = std::distance(perViewErrors.begin(), minErr);
    std::cout << "最小误差: 图像 " << minIdx << " (" << *minErr << " 像素)" << std::endl;
    std::cout << "最大误差: 图像 " << maxIdx << " (" << *maxErr << " 像素)" << std::endl;

    // ================== 8. 保存标定结果 ==================
    std::string outputFile = "/home/vboxuser/WorkSpace/Temps/Dormitory_SLAM/camera_params.xml";
    cv::FileStorage fs(outputFile, cv::FileStorage::WRITE);
    
    fs << "calibration_time" << static_cast<double>(cv::getTickCount());
    fs << "image_size" << imageSize;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "reprojection_error" << rms;
    fs << "num_valid_images" << (int)imagePoints.size();
    
    fs.release();
    std::cout << "\n参数已保存至: " << outputFile << std::endl;

    // ================== 9. 生成去畸变示例 ==================
    std::cout << "\n正在生成去畸变示例..." << std::endl;
    cv::Mat testImg = cv::imread(imagePaths[0]);
    if (!testImg.empty()) {
        cv::Mat undistorted;
        cv::undistort(testImg, undistorted, cameraMatrix, distCoeffs);
        
        cv::Mat comparison;
        cv::hconcat(testImg, undistorted, comparison);
        
        // 缩小图片以节省内存
        if (comparison.cols > 1600) {
            cv::resize(comparison, comparison, cv::Size(), 0.5, 0.5);
        }
        
        std::string outputImg = "/home/vboxuser/WorkSpace/Temps/Dormitory_SLAM/undistort_comparison.jpg";
        cv::imwrite(outputImg, comparison);
        std::cout << "去畸变对比图已保存: " << outputImg << std::endl;
    }

    std::cout << "\n标定完成！" << std::endl;
    return 0;
}