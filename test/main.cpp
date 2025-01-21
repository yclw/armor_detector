#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <string.h>

#include "virtual_camera.hpp"

int main()
{
    // 创建共享资源
    std::unordered_map<std::string, std::shared_ptr<std::pair<cv::Mat, bool>>> producerFrames;
    std::mutex mtx;
    std::condition_variable cv;

    LightParams light_params;
    ArmorParams armor_params;
    armor_params.max_angle = 100.0;
    Detector detector(light_params, armor_params, 120, EnemyColor::BLUE);
    cv::Mat img = cv::imread("./1.png");
    if (img.empty())
    {
        std::cout << "Image not found" << std::endl;
    }

    std::cout << img.size() << std::endl;
    // cv::resize(img, img, cv::Size(640, 480));

    // 相机名称列表
    std::vector<std::string> cameraNames = {"Camera1", "Camera2", "Camera3"};
    std::vector<std::unique_ptr<VirtualCamera>> cameras;
    std::vector<std::thread> producerThreads;

    // 创建虚拟相机并启动线程
    for (const auto &name : cameraNames)
    {
        auto sharedFrame = std::make_shared<std::pair<cv::Mat, bool>>(img, false);
        producerFrames[name] = sharedFrame;
        cameras.push_back(std::make_unique<VirtualCamera>(name, sharedFrame, mtx, cv, img, 5));
        producerThreads.emplace_back(std::ref(*cameras.back()));
    }
    // cameras[0]->setSleep(20);

    // 创建识别消费者
    DetectorConsumer detectorConsumer(producerFrames, mtx, cv, detector);

    // 启动识别消费者线程
    std::thread consumerThread(std::ref(detectorConsumer));

    // 创建识别消费者
    Detector detector2(light_params, armor_params, 120, EnemyColor::BLUE);
    // 创建识别消费者
    DetectorConsumer detectorConsumer2(producerFrames, mtx, cv, detector2);
    // 启动识别消费者线程
    std::thread consumerThread2(std::ref(detectorConsumer2));

    // 主线程统计帧率
    int previousFrameCount = 0;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒统计一次
        int currentFrameCount = detectorConsumer.getFrameCount() + detectorConsumer2.getFrameCount();
        int processedFrames = currentFrameCount - previousFrameCount;
        previousFrameCount = currentFrameCount;

        for (size_t i = 0; i < cameras.size(); ++i)
        {
            // 程序运行开始到现在的总丢帧数
            std::cout << cameraNames[i] << " lost frames: " << cameras[i]->getLostFrames() << std::endl;
        }

        // 输出帧率
        std::cout << "FPS: " << processedFrames << " frames/second" << std::endl;
    }

    // 等待线程结束
    for (auto &producer : producerThreads)
    {
        producer.join();
    }
    consumerThread.join();

    return 0;
}
