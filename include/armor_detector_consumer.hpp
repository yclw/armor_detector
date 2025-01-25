#ifndef ARMOR_DETECTOR_CONSUMER_HPP
#define ARMOR_DETECTOR_CONSUMER_HPP

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

class DetectorConsumer
{
public:
    DetectorConsumer(std::unordered_map<std::string, std::shared_ptr<std::pair<cv::Mat, bool>>> &producerFrames, std::mutex &mtx, std::condition_variable &cv, Detector &detector)
        : producerFrames(producerFrames), mtx(mtx), cv(cv), detector(detector), counst(0) {}

    void operator()()
    {

        while (true)
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this]
                    {
                for (const auto& pair : producerFrames) {
                    if (pair.second  && pair.second->second) return true;
                }
                return false; });

            for (auto &pair : producerFrames)
            {
                if (pair.second && pair.second->second)
                {
                    
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
    int getFrameCount() const
    {
        return counst.load();
    }

private:
    std::atomic<int> counst; // 处理帧计数器

    Detector detector;
    std::unordered_map<std::string, std::shared_ptr<std::pair<cv::Mat, bool>>> &producerFrames;
    std::mutex &mtx;
    std::condition_variable &cv;
};

#endif // ARMOR_DETECTOR_CONSUMER_HPP