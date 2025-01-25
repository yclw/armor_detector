#ifndef DAH_CAMERA_HPP
#define DAH_CAMERA_HPP

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

#include <GxIAPI.h>

class DahCamera
{
public:
    DahCamera(std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex &mtx, std::condition_variable &cv, GX_DEVICE_BASE_INFO *device)
        : sharedFrame(sharedFrame), mtx(mtx), cv(cv), lostFrames(0), stop(false)
    {
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.openMode = GX_OPEN_SN;
        stOpenParam.pszContent = device->szSN;
        GXOpenDevice(&stOpenParam, &camera_handle_);
        GXStreamOn(camera_handle_);
        GXSetAcqusitionBufferNumber(camera_handle_, nBufferNum);
        GXSetEnum(camera_handle_,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        // GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME,exposure_time);
    }

    DahCamera(std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex &mtx, std::condition_variable &cv)
        : sharedFrame(sharedFrame), mtx(mtx), cv(cv), lostFrames(0)
    {
        std::vector<GX_DEVICE_BASE_INFO> devices = getDahCameraList();
        if (devices.size()==0)
        {
            std::cout << "No camera found." << std::endl;
            return;
        }
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.openMode = GX_OPEN_SN;
        stOpenParam.pszContent = devices[0].szSN;
        GXOpenDevice(&stOpenParam, &camera_handle_);
        


        GXSetAcqusitionBufferNumber(camera_handle_, nBufferNum);
        GXSetEnum(camera_handle_,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        // GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME,exposure_time);
    }

    ~DahCamera()
    {
        stop = false;
        GXStreamOff(camera_handle_);
        GXCloseDevice(camera_handle_);
        GXCloseLib();
        std::cout << "HikCamera destroyed." << std::endl;
    }

    static std::vector<GX_DEVICE_BASE_INFO> getDahCameraList()
    {
        uint32_t nDeviceNum = 0;
        GXInitLib();
        GXUpdateDeviceList(&nDeviceNum, 1000);
        std::cout << "Device number: " << nDeviceNum << std::endl;

        std::vector<GX_DEVICE_BASE_INFO> device_list(nDeviceNum);

        if (nDeviceNum == 0) return device_list;

        size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        // 获取所有设备的基础信息
        GXGetAllDeviceBaseInfo(device_list.data(), &nSize);

        return device_list;
    }


    void operator()()
    {
        int nRet = GX_STATUS_SUCCESS;
        GXStreamOn(camera_handle_);
        char input;
        while (stop.load())
        {
            nRet = GXDQBuf(camera_handle_, &pFrameBuffer, 1000);
            if (GX_STATUS_SUCCESS == nRet)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS){
                    frame = cv::Mat(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3, pFrameBuffer->pImgBuf);
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        if (sharedFrame->second)
                            lostFrames++;
                        sharedFrame->first = generateNewFrame();
                        sharedFrame->second = true;
                    }
                    cv.notify_all();
                }
                nRet = GXQBuf(camera_handle_, pFrameBuffer);
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
    GX_DEV_HANDLE camera_handle_ = nullptr;
    // 相机启动参数
    GX_OPEN_PARAM stOpenParam;
    // 设置采集队列的缓冲区数量
    uint64_t nBufferNum = 1;
    //定义 GXDQBuf 的传入参数
    PGX_FRAME_BUFFER pFrameBuffer;
    // 默认曝光时间为5000微秒
    float exposure_time = 5000;
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
    std::atomic<bool> stop;

    std::atomic<int> lostFrames; // 丢弃帧计数器
};

#endif // DAH_CAMERA_HPP