#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>

int main()
{
    cv::Mat img = cv::imread("/home/ericlab/tsuchida/real_image/image_1_1_16_45_3.jpg");
    int sum = 0;
    int count = 0;
    clock_t start = clock();
    for (int i = 0; i < img.size().height; i+=10) {
        for (int j = 0; j < img.size().width; j+=10) {
            sum =  (int)img.at<cv::Vec3b>(i, j)[0] + sum;
            count++;
        }
    }
    std::cout << "count: " << count << std::endl;
    std::cout << (double)(sum / count) << std::endl;
    clock_t end = clock();
    std::cout << "time: " << (double)(end - start) / CLOCKS_PER_SEC << "sec. " << std::endl;
    // std::cout << (int)img.at<cv::Vec3b>(10, 10)[0] << std::endl;

}