#include "armor_detector.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


int main()
{
    cv::Mat frame = cv::imread("1.png");
    // cv::resize(frame,frame,cv::Size(640,480));

    std::cout << frame.size() << std::endl;

    LightParams light_params;
    ArmorParams armor_params;
    armor_params.max_angle = 100.0;
    Deterctor deterctor(light_params,armor_params,60,EnemyColor::BLUE);
    

    int n = 0;
    int64 start = cv::getTickCount();
    

    while (true)
    {
        // cap >> frame;
        std::vector<Result> result;
        deterctor.deterct(frame,result);
        for (auto &r:result){std::cout << r.number << " " << r.x << " " << r.y << std::endl;}
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