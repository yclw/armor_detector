#include "armor_detector.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <virtual_camera.hpp>

// 消费者类
class Consumer {
public:
    Consumer(std::unordered_map<std::string, std::queue<Data>>& producerQueues, std::mutex& mtx, std::condition_variable& cv, Deterctor& deterctor)
        : producerQueues(producerQueues), mtx(mtx), cv(cv), deterctor(deterctor) {}

    void operator()() {
        while (true) {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this] {
                for (const auto& pair : producerQueues) {
                    if (!pair.second.empty()) {
                        return true;
                    }
                }
                return false;
            });

            std::unordered_map<std::string, Data> latestData;
            for (auto& pair : producerQueues) {
                if (!pair.second.empty()) {
                    latestData[pair.first] = pair.second.front();
                    pair.second.pop();
                }
            }

            // 消费最新数据
            if (!latestData.empty()) {
                std::cout << "Consumer consumed: ";
                for (const auto& data : latestData) {
                    std::vector<Result> results;
                    deterctor.deterct(data.second, results);
                    std::cout << data.first << ": ";
                    for (const auto& result : results) {
                        std::cout << "(" << result.number << ") ";
                    }
                    std::cout << std::endl;
                }
                std::cout << std::endl;
            }

            // std::this_thread::sleep_for(std::chrono::seconds(1)); // 模拟消费延时
        }
    }

private:
    // 识别器
    Deterctor deterctor;

    std::unordered_map<std::string, std::queue<Data>>& producerQueues;
    std::mutex& mtx;
    std::condition_variable& cv;
};

int main()
{
    LightParams light_params;
    ArmorParams armor_params;
    armor_params.max_angle = 100.0;
    Deterctor deterctor(light_params,armor_params,120,EnemyColor::BLUE);
    cv::Mat img = cv::imread("./1.png");
    if (img.empty())
    {
        std::cout << "Image not found" << std::endl;
    }
    

    std::mutex mtx;
    std::unordered_map<std::string, std::queue<Data>> producerQueues;
    std::condition_variable cv;

    // 创建并启动生产者线程
    std::vector<std::thread> producers;
    producers.emplace_back(VirtualCamera("Producer1", producerQueues["Producer1"], mtx, cv, img));
    producers.emplace_back(VirtualCamera("Producer2", producerQueues["Producer2"], mtx, cv, img));
    producers.emplace_back(VirtualCamera("Producer3", producerQueues["Producer3"], mtx, cv, img));

    // 创建消费者
    std::thread consumerThread(Consumer(producerQueues, mtx, cv, deterctor));

    // 等待线程结束
    for (auto& producer : producers) {
        producer.join();
    }
    consumerThread.join();

    return 0;
}

