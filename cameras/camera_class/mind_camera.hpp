#ifndef MIND_CAMERA_HPP
#define MIND_CAMERA_HPP

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

#include <CameraApi.h>
class MindCamera
{
public:
    MindCamera(std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex &mtx, std::condition_variable &cv, tSdkCameraDevInfo *device)
        : sharedFrame(sharedFrame), mtx(mtx), cv(cv), lostFrames(0)
    {
        CameraInit(device, -1, -1, &camera_handle_);
        // CameraGetCapability(camera_handle_, &t_capability_);
        CameraSetAeState(camera_handle_, false);

        // CameraSetExposureTime(camera_handle_, exposure_time);
        // CameraSetAnalogGain(camera_handle_, analog_gain);
        // CameraSetGain(camera_handle_, r_gain_, g_gain_, b_gain_);
        // CameraSetSaturation(camera_handle_, saturation);
        // CameraSetGamma(camera_handle_, gamma);
    }

    MindCamera(std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex &mtx, std::condition_variable &cv)
        : sharedFrame(sharedFrame), mtx(mtx), cv(cv), lostFrames(0)
    {
        std::vector<tSdkCameraDevInfo> devices = getMindCameraList();
        if (devices.empty())
        {
            std::cout << "No camera found." << std::endl;
            return;
        }
        CameraInit(&devices[0], -1, -1, &camera_handle_);
        // CameraGetCapability(camera_handle_, &t_capability_);
        CameraSetAeState(camera_handle_, false);

        // CameraSetExposureTime(camera_handle_, exposure_time);
        // CameraSetAnalogGain(camera_handle_, analog_gain);
        // CameraSetGain(camera_handle_, r_gain_, g_gain_, b_gain_);
        // CameraSetSaturation(camera_handle_, saturation);
        // CameraSetGamma(camera_handle_, gamma);
    }

    ~MindCamera()
    {
        std::cout << "MindCamera destroyed." << std::endl;
    }

    static std::vector<tSdkCameraDevInfo> getMindCameraList()
    {
        CameraSdkInit(1);
        
        // 枚举设备，并建立设备列表
        int i_camera_counts = 4;
        int ret = -1;
        tSdkCameraDevInfo device_list[4];
        ret = CameraEnumerateDevice(device_list, &i_camera_counts);
        std::cout << "Enumerate device status = " << ret << std::endl;
        std::cout << "Found camera count = " << i_camera_counts << std::endl;

        std::vector<tSdkCameraDevInfo> devices;
        for (int i = 0; i < i_camera_counts; i++)
        {
            devices.push_back(device_list[i]);
        }
        return devices;
    }

    void operator()()
    {
        int nRet = CAMERA_STATUS_SUCCESS;
        CameraPlay(camera_handle_);
        CameraSetIspOutFormat(camera_handle_, CAMERA_MEDIA_TYPE_RGB8);
        tSdkFrameHead out_frame_head;
        uint8_t * out_frame_buffer;

        char input;
        while (true)
        {
            nRet = CameraGetImageBuffer(camera_handle_, &out_frame_head, &out_frame_buffer, 1000);
            if (nRet == CAMERA_STATUS_SUCCESS)
            {
                image_data_.resize(out_frame_head.iHeight * out_frame_head.iWidth * 3);
                CameraImageProcess(camera_handle_, out_frame_buffer, image_data_.data(), &out_frame_head);
                frame = cv::Mat(out_frame_head.iHeight, out_frame_head.iWidth, CV_8UC3, image_data_.data());

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    if (sharedFrame->second)
                        lostFrames++;
                    sharedFrame->first = generateNewFrame();
                    sharedFrame->second = true;
                }
                cv.notify_all();

                CameraReleaseImageBuffer(camera_handle_, out_frame_buffer);
                fail_conut_ = 0;
            }
            else
            {
                std::cout << "Get buffer failed, retrying..." << std::endl;
                fail_conut_++;
            }

            if (fail_conut_ > 5)
            {
                std::cout << "Camera failed..." << std::endl;
            }
        }
    }

    int getLostFrames() const
    {
        return lostFrames.load();
    }

private:
    cv::Mat generateNewFrame()
    {
        return frame.clone();
    }

    // 相机句柄，用于相机的控制和通信
    CameraHandle camera_handle_;
    // 默认曝光时间为5000微秒
    double exposure_time = 5000;
    // 默认增益为16
    double gain = 16;
    // 存储图像数据的缓冲区
    std::vector<uint8_t> image_data_;
    // 标记图像采集是否完成
    bool over = false;
    // 记录相机连接失败的次数
    int fail_conut_ = 0;

    cv::Mat frame;

    std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame;
    std::mutex &mtx;
    std::condition_variable &cv;

    std::atomic<int> lostFrames; // 丢弃帧计数器
};


#endif // MIND_CAMERA_HPP