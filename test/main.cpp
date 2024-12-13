#include "armor_detector.hpp"
#include <iostream>
#include "CameraApi.h"
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

unsigned char           * g_pRgbBuffer;

int main()
{
    ////////////////////////////////
        int                     iCameraCounts = 4;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList[4];
    int                     hCamera;
    tSdkCameraCapbility     tCapability;
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    tSdkImageResolution     sImageSize;
    int                     i=0;
	int						num=0;


    //sdk初始化  0 English 1中文
    CameraSdkInit(1);
    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);




	printf("iCameraCounts =%d  \n",iCameraCounts);

    if(iCameraCounts==0){
        return -1;
    }



    for(i=0;i<iCameraCounts;i++)
    {
        printf("num =%d %s  %s \n",i,tCameraEnumList[i].acProductName,tCameraEnumList[i].acFriendlyName);
    }


    printf("input   0-%d:",iCameraCounts-1);
    scanf("%d", &num);
    printf("you input num %d \n",num);

    if(num>=iCameraCounts || num < 0)
    {
        printf("Enter a number is invalid  %d \n",num);
        return -1;
    }


    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList[num],-1,-1,&hCamera);

	printf("CameraInit iStatus =%d \n",iStatus);
    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }


    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);
	printf("CameraGetCapability \n");
    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);




    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
	printf("CameraPlay \n");


#if  0
    memset(&sImageSize,0,sizeof(tSdkImageResolution));
    sImageSize.iIndex=0xff;
    sImageSize.iHOffsetFOV=0;
    sImageSize.iVOffsetFOV=0;
    sImageSize.iWidthFOV=800;
    sImageSize.iHeightFOV=600;
    sImageSize.iWidth=800;
    sImageSize.iHeight=600;


    CameraSetImageResolution(hCamera,&sImageSize);
#else
    CameraSetImageResolution(hCamera,&tCapability.pImageSizeDesc[0]);
#endif

/*
    设置图像处理的输出格式，彩色黑白都支持RGB24位
*/
    if(tCapability.sIspCapacity.bMonoSensor){
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_RGB8);
    }
    printf("CameraSetIspOutFormat \n");
    sleep(2);
    ////////////////////////////////


    // cv::VideoCapture cap("resources/v.mp4");
    // if (!cap.isOpened())
    // {
    //     std::cout << "open video failed" << std::endl;
    //     return -1;
    // }
    // cv::Mat frame; // = cv::imread("resources/blue.png");

    LightParams light_params;
    ArmorParams armor_params;
    Deterctor deterctor(light_params,armor_params,128,EnemyColor::BLUE);
    

    int n = 0;
    int64 start = cv::getTickCount();
    

    // while (true)
    // {
    //     // cap >> frame;
    //     std::vector<Result> result;
    //     deterctor.deterct(frame,result);
    //     // for (auto &r:result){std::cout << r.number << " " << r.x << " " << r.y << std::endl;}
    //     n++;
    //     int64 end = cv::getTickCount();
    //     double timeInSeconds = (end - start) / cv::getTickFrequency();
    //     if (timeInSeconds>=1)
    //     {
    //         std::cout << "FPS: " << n << std::endl;
    //         n=0;
    //         start = cv::getTickCount();
    //     }
    // }
    while (true) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 2000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
            cv::Mat image(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3, g_pRgbBuffer);
            
            std::vector<Result> result;
            deterctor.deterct(image,result);
            for (auto &r:result){
                // std::cout << r.number << " " << r.x << " " << r.y << std::endl;
                cv::circle(image, cv::Point(r.x, r.y), 5, cv::Scalar(0, 0, 255), -1);
            }
            cv::imshow("Camera Image", image);
            cv::waitKey(1);

            CameraReleaseImageBuffer(hCamera, pbyBuffer);
            if (cv::waitKey(1) == 'q') {
                break;
            }
        } else {
            printf("timeout \n");
        }

        n++;
        int64 end = cv::getTickCount();
        double timeInSeconds = (end - start) / cv::getTickFrequency();
        if (timeInSeconds>=1)
        {
            std::cout << "FPS: " << n << std::endl;
            n=0;
            start = cv::getTickCount();
        }
    }

    return 0;
}