#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <memory>
#include <atomic>
#include <chrono>
#include "armor_detector.hpp"

class VirtualCamera {
public:
    VirtualCamera(const std::string& name, std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex& mtx, std::condition_variable& cv, cv::Mat frame, int sleep = 5)
        : name(name), sharedFrame(sharedFrame), mtx(mtx), cv(cv), frame(frame.clone()), sleep(sleep), lostFrames(0) {}

    void operator()() {
        while (true) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (sharedFrame->second) lostFrames++;
                sharedFrame->first = generateNewFrame();
                sharedFrame->second = true;
            }
            cv.notify_all();

            // 模拟帧生成速度
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep)); // 默认200帧/秒
        }
    }

    int getLostFrames() const {
        return lostFrames.load();
    }

    // 设置帧生成速度
    void setSleep(int sleep) {
        this->sleep = sleep;
    }

private:
    cv::Mat generateNewFrame() {
        return frame.clone();
    }

    cv::Mat frame;


    std::string name;
    std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame;
    std::mutex& mtx;
    std::condition_variable& cv;
    int sleep;

    std::atomic<int> lostFrames; // 丢弃帧计数器
};

class DetectorConsumer {
public:
    DetectorConsumer(std::unordered_map<std::string, std::shared_ptr<std::pair<cv::Mat, bool>>>& producerFrames, std::mutex& mtx, std::condition_variable& cv, Detector& detector)
        : producerFrames(producerFrames), mtx(mtx), cv(cv), detector(detector), counst(0) {}

    void operator()() {

        while (true) {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this] {
                for (const auto& pair : producerFrames) {
                    if (pair.second  && pair.second->second) return true;
                }
                return false;
            });

            for (auto& pair : producerFrames) {
                if (pair.second && pair.second->second) {
                    pair.second->second = false; // 帧处理标志
                    cv::Mat frame = pair.second->first.clone();
                    lock.unlock();

                    std::vector<Result> results;
                    detector.detect(frame, results);

                    counst++; // 统计帧数

                    lock.lock();
                }
            }
        }
    }

    // 获取处理帧数
    int getFrameCount() const {
        return counst.load();
    }

private:

    std::atomic<int> counst; // 处理帧计数器

    Detector detector;
    std::unordered_map<std::string, std::shared_ptr<std::pair<cv::Mat, bool>>>& producerFrames;
    std::mutex& mtx;
    std::condition_variable& cv;
};
