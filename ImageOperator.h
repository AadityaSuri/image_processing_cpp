#pragma once

#include <opencv2/opencv.hpp>


class ImageOperator {
public:
    ImageOperator() = default;
    ~ImageOperator() = default;

    static void to_gray_m1(const cv::Mat& input, cv::Mat& output);
    static void to_gray_m2(const cv::Mat& input, cv::Mat& output);
    static void to_gray_m3(const cv::Mat& input, cv::Mat& output);
   /* static void to_gray(const unsigned char* input,
        const int width,
        const int height,
        const int channel,
        unsigned char* output);*/

    static void to_gray(const std::unique_ptr<unsigned char[]>& bgr_input,
        const int width,
        const int height,
        const int channel,
        std::unique_ptr<unsigned char[]>& gray_output);


    static void to_yuv(const std::unique_ptr<unsigned char[]>& bgr_input,
        const int width,
        const int height,
        const int channel,
        std::unique_ptr<unsigned char[]>& yuv_output);
};