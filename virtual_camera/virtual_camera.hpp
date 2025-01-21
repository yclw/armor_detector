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

class VirtualCamera
{
public:
    VirtualCamera(const std::string &name, std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex &mtx, std::condition_variable &cv, cv::Mat frame, int sleep = 5)
        : name(name), sharedFrame(sharedFrame), mtx(mtx), cv(cv), frame(frame.clone()), sleep(sleep), lostFrames(0) {}

    void operator()()
    {
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (sharedFrame->second)
                    lostFrames++;
                sharedFrame->first = generateNewFrame();
                sharedFrame->second = true;
            }
            cv.notify_all();

            // 模拟帧生成速度
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep)); // 默认200帧/秒
        }
    }

    int getLostFrames() const
    {
        return lostFrames.load();
    }

    // 设置帧生成速度
    void setSleep(int sleep)
    {
        this->sleep = sleep;
    }

private:
    cv::Mat generateNewFrame()
    {
        return frame.clone();
    }

    cv::Mat frame;

    std::string name;
    std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame;
    std::mutex &mtx;
    std::condition_variable &cv;
    int sleep;

    std::atomic<int> lostFrames; // 丢弃帧计数器
};

#include "MvCameraControl.h"
class HikCamera
{
public:
    HikCamera(std::shared_ptr<std::pair<cv::Mat, bool>> sharedFrame, std::mutex &mtx, std::condition_variable &cv, MV_CC_DEVICE_INFO *device)
        : sharedFrame(sharedFrame), mtx(mtx), cv(cv), lostFrames(0)
    {
        MV_CC_CreateHandle(&camera_handle_, device);
        MV_CC_OpenDevice(camera_handle_);
        MV_CC_GetImageInfo(camera_handle_, &img_info_);
        image_data_.reserve(img_info_.nWidthValue * img_info_.nHeightValue * 3);
        convert_param_.nWidth = img_info_.nWidthValue;
        convert_param_.nHeight = img_info_.nHeightValue;
        convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
        MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
        MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    }

    ~HikCamera()
    {
        std::cout << "HikCamera destroyed." << std::endl;
    }

    static std::vector<MV_CC_DEVICE_INFO *> getHikCameraList()
    {
        MV_CC_DEVICE_INFO_LIST device_list;
        // enum device
        int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        std::cout << "Enum state: " << std::hex << nRet << std::endl;
        std::cout << "Device number: " << device_list.nDeviceNum << std::endl;

        while (device_list.nDeviceNum == 0)
        {
            std::cout << "Not found camera, retrying..." << std::endl;
            std::cout << "Enum state: " << std::hex << nRet << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        }

        std::vector<MV_CC_DEVICE_INFO *> devices;
        for (auto &device_list_item : device_list.pDeviceInfo)
        {
            devices.push_back(device_list_item);
        }
        return devices;
    }

    void operator()()
    {
        int nRet = MV_OK;
        MV_CC_StartGrabbing(camera_handle_);
        MV_FRAME_OUT out_frame = {0};
        memset(&out_frame, 0, sizeof(MV_FRAME_OUT));
        char input;
        while (true)
        {
            if (MV_OK == MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000))
            {
                image_data_.resize(out_frame.stFrameInfo.nWidth * out_frame.stFrameInfo.nHeight * 3);
                convert_param_.pDstBuffer = image_data_.data();
                convert_param_.nDstBufferSize = image_data_.size();
                convert_param_.pSrcData = out_frame.pBufAddr;
                convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
                convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
                MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
                frame = cv::Mat(out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth, CV_8UC3, image_data_.data());

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    if (sharedFrame->second)
                        lostFrames++;
                    sharedFrame->first = generateNewFrame();
                    sharedFrame->second = true;
                }
                cv.notify_all();

                MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                fail_conut_ = 0;
            }
            else
            {
                std::cout << "Get buffer failed, retrying..." << std::endl;
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_StartGrabbing(camera_handle_);
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
    void *camera_handle_;
    // 图像基本信息结构体，包含图像的宽度、高度和像素格式等信息
    MV_IMAGE_BASIC_INFO img_info_;
    // 默认曝光时间为5000微秒
    double exposure_time = 5000;
    // 默认增益为16
    double gain = 16;
    // 像素转换参数结构体，用于图像像素格式的转换
    MV_CC_PIXEL_CONVERT_PARAM convert_param_;
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
