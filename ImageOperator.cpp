#include <opencv2/opencv.hpp>
#include "ImageOperator.h"
#include <memory>
#include <algorithm>

void ImageOperator::to_gray_m1(const cv::Mat& input, cv::Mat& output) {

    unsigned char* data_out = (unsigned char*)(output.data);

    int ind = 0;
    auto end = input.end<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> it = input.begin<cv::Vec3b>();
    for (; it != end; ++it) {
        const unsigned char& r = (*it)[2];
        const unsigned char& g = (*it)[1];
        const unsigned char& b = (*it)[0];
        data_out[ind] = (std::min({ r, g, b}) + std::max({ r, g, b}) ) / 2;
        ind++;
    }

}


void ImageOperator::to_gray_m2(const cv::Mat& input, cv::Mat& output) {
    unsigned char* data_in = (unsigned char*)(input.data);
    unsigned char* data_out = (unsigned char*)(output.data);

    int index = 0;
    int byte_size = input.channels() * input.rows * input.cols;
    while (index != byte_size) {
        data_out[index / input.channels()] = unsigned(0.11 * data_in[index] + 0.59 * data_in[index + 1] + 0.3 * data_in[index + 2]);

        index += 3;
    }
}



void ImageOperator::to_gray_m3(const cv::Mat& input, cv::Mat& output) {

    unsigned char* data_in = (unsigned char*)(input.data);
    unsigned char* data_out = (unsigned char*)(output.data);

    int index = 0;
    for (int row = 0; row < input.rows; ++row) {
        for (int col = 0; col < input.cols * input.channels(); col += input.channels()) {
            data_out[index] = 0.11 * data_in[row * input.step + col] +
                0.59 * data_in[row * input.step + col + 1] +
                0.3 * data_in[row * input.step + col + 2];
            index++;
        }
    }
}

//
//void ImageOperator::to_gray(const unsigned char* bgr_input,
//    const int width,
//    const int height,
//    const int channel,
//    unsigned char* gray_output) {
//    int index = 0;
//    int step = channel * width;
//    for (int row = 0; row < height; ++row) {
//        for (int col = 0; col < width * channel; col += channel) {
//            gray_output[index] = 0.11 * bgr_input[row * step + col] +
//                0.59 * bgr_input[row * step + col + 1] +
//                0.3 * bgr_input[row * step + col + 2];
//            index++;
//        }
//    }
//}



void ImageOperator::to_gray(const std::unique_ptr<unsigned char[]>& bgr_input,
    const int width,
    const int height,
    const int channel,
    std::unique_ptr<unsigned char[]>& gray_output) {

    const int pixel_count = width * height;
    gray_output = std::make_unique<unsigned char[]>(pixel_count);

    int index = 0;
    int step = channel * width;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width * channel; col += channel) {
            gray_output[index] = 0.11 * bgr_input[row * step + col] +
                0.59 * bgr_input[row * step + col + 1] +
                0.3 * bgr_input[row * step + col + 2];
            index++;
        }
    }
}

//
void ImageOperator::to_yuv(const std::unique_ptr<unsigned char[]>& bgr_input,
    const int width,
    const int height,
    const int channel,
    std::unique_ptr<unsigned char[]>& yuv_output) {

    const int pixel_count = width * height;
    //gray_output = std::make_unique<unsigned char[]>(pixel_count);
    yuv_output = std::make_unique<unsigned char[]>(pixel_count);
 
    int index = 0;
    int step = channel * width;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width * channel; col += channel) {
            unsigned char r = bgr_input[row * step + col + 2];
            unsigned char g = bgr_input[row * step + col + 1];
            unsigned char b = bgr_input[row * step + col];

            yuv_output[index] = (r >> 2) + (g >> 1) + (b >> 1); // Y channel
            yuv_output[index + pixel_count] =  (b - yuv_output[index]) >> 1; // U channel
            yuv_output[index + pixel_count * 2] = (r - yuv_output[index]) >> 1; // V channel
            index++;
            
        }
    }
}

