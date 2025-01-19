#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>
#include <chrono>
#include <unordered_map>
#include <opencv2/core.hpp>




using Data = cv::Mat;
class VirtualCamera {
public:
    VirtualCamera(const std::string& name, std::queue<Data>& dataQueue, std::mutex& mtx, std::condition_variable& cv , Data& img)
        : name(name), dataQueue(dataQueue), mtx(mtx), cv(cv),img(img) {}

    void operator()() {
        
        while (true) {
            Data newData = img.clone();
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (!dataQueue.empty()) {
                    dataQueue.pop(); // 弹出旧数据，确保队列中只有最新数据
                }
                dataQueue.push(newData); // 插入新数据
            }
            cv.notify_all(); // 通知消费者数据已更新
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 模拟数据生成延时
        }
    }

private:
    Data img;
    std::string name;
    std::queue<Data>& dataQueue;
    std::mutex& mtx;
    std::condition_variable& cv;
};