#include <opencv2/opencv.hpp>
#include "ImageOperator.h"


void ImageOperator::to_gray_m1(const cv::Mat& input, cv::Mat& output) {

    unsigned char* data_out = (unsigned char*)(output.data);
    int ind = 0;
    auto end = input.end<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> it = input.begin<cv::Vec3b>();
    for (; it != end; ++it) {
        const unsigned char& r = (*it)[2];
        const unsigned char& g = (*it)[1];
        const unsigned char& b = (*it)[0];
        data_out[ind] = 0.3 * r + 0.59 * g + 0.11 * b;
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