
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageOperator.h"
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>



cv::Mat TO_GRAY(const cv::Mat& input) {
	Eigen::Tensor<unsigned char, 3> inputTensor(input.cols, input.rows, 3);
	cv::cv2eigen(input, inputTensor);

	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> output(input.cols, input.rows);
	for (int i = 0; i < input.cols; ++i) {
		for (int j = 0; j < input.rows; ++j) {
			unsigned char r = inputTensor(i, j, 2);
			unsigned char g = inputTensor(i, j, 1);
			unsigned char b = inputTensor(i, j, 0);

			output(i, j) = 0.11 * r + 0.59 * g + 0.3 * b;
		}
	}

	cv::Mat img;
	cv::eigen2cv(output, img);
	return img;

}

cv::Mat RED(const cv::Mat& input) {

	Eigen::Tensor<unsigned char, 3> inputTensor(input.cols, input.rows, 3);
	cv::cv2eigen(input, inputTensor);

	Eigen::Tensor<unsigned char, 3> output(input.cols, input.rows, 3);
	for (int i = 0; i < input.cols; ++i) {
		for (int j = 0; j < input.rows; ++j) {
			unsigned char r = inputTensor(i, j, 2);
			unsigned char g = inputTensor(i, j, 1);
			unsigned char b = inputTensor(i, j, 0);
			output(i, j, 2) =  r;
			output(i, j, 1) = 0;
			output(i, j, 0) = 0;

		}
	}
	cv::Mat img;
	cv::eigen2cv(output, img);
	return img;
}

cv::Mat BLUE(cv::Mat& input) {
	Eigen::Tensor<unsigned char, 3> inputTensor(input.cols, input.rows, 3);
	cv::cv2eigen(input, inputTensor);

	Eigen::Tensor<unsigned char, 3> output(input.cols, input.rows, 3);
	for (int i = 0; i < input.cols; ++i) {
		for (int j = 0; j < input.rows; ++j) {
			unsigned char r = inputTensor(i, j, 2);
			unsigned char g = inputTensor(i, j, 1);
			unsigned char b = inputTensor(i, j, 0);
			output(i, j, 2) = 0;
			output(i, j, 1) = 0;
			output(i, j, 0) = b;
		}
	}
	cv::Mat img;
	cv::eigen2cv(output, img);
	return img;
}

cv::Mat GREEN(cv::Mat& input) {
	Eigen::Tensor<unsigned char, 3> inputTensor(input.cols, input.rows, 3);
	cv::cv2eigen(input, inputTensor);

	Eigen::Tensor<unsigned char, 3> output(input.cols, input.rows, 3);
	for (int i = 0; i < input.cols; ++i) {
		for (int j = 0; j < input.rows; ++j) {
			unsigned char r = inputTensor(i, j, 2);
			unsigned char g = inputTensor(i, j, 1);
			unsigned char b = inputTensor(i, j, 0);
			output(i, j, 2) = 0;
			output(i, j, 1) = g;
			output(i, j, 0) = 0;
		}
	}
	cv::Mat img;
	cv::eigen2cv(output, img);
	return img;
}

int main() {
	cv::VideoCapture cam(0);

	if (!cam.isOpened()) {
		throw std::runtime_error("Error");
	}

	cv::namedWindow("Window");

	while (true) {
		cv::Mat frame;
		cam >> frame;
		cv::resize(frame, frame, cv::Size(250, 250));
		cv::imshow("bgr_frame", frame);

		cv::imshow("eigen_img", TO_GRAY(frame));
		cv::imshow("eigen_red", RED(frame));
		cv::imshow("eigen_blue", BLUE(frame));
		cv::imshow("eigen_green", GREEN(frame));

		if (cv::waitKey(30) >= 0) break;
	}

}




//
//int main() {
//    cv::VideoCapture cam(0);
//
//    if (!cam.isOpened()) {
//        throw std::runtime_error("Error");
//    }
//
//    cv::namedWindow("Window");
//    cv::Mat output(350, 350, CV_8UC1);
//    cv::Mat rgb_output(350, 350, CV_8UC3);
//
//    std::unordered_map<std::string, std::vector<double>> timings_map;
//
//    while (true) {
//        cv::Mat frame;
//        cam >> frame;
//        cv::resize(frame, frame, cv::Size(350, 350));
//
//        cv::imshow("bgr_frame", frame);
//
//        auto start_cv = std::chrono::high_resolution_clock::now();
//        cv::cvtColor(frame, output, cv::COLOR_BGR2GRAY);
//        cv::imshow("opencv_func", output);
//        auto end_cv = std::chrono::high_resolution_clock::now();
//        auto duration_cv = std::chrono::duration_cast<std::chrono::microseconds>(end_cv - start_cv);
//        timings_map["opencv_func"].push_back(duration_cv.count());
//
//        auto start_m1 = std::chrono::high_resolution_clock::now();
//        ImageOperator::to_gray_m1(frame, output);
//        cv::imshow("to_gray_m1", output);
//        auto end_m1 = std::chrono::high_resolution_clock::now();
//        auto duration_m1 = std::chrono::duration_cast<std::chrono::microseconds>(end_m1 - start_m1);
//        timings_map["to_gray_m1"].push_back(duration_m1.count());
//
//
//        auto start_m2 = std::chrono::high_resolution_clock::now();
//        ImageOperator::to_gray_m2(frame, output);
//        cv::imshow("to_gray_m2", output);
//        auto end_m2 = std::chrono::high_resolution_clock::now();
//        auto duration_m2 = std::chrono::duration_cast<std::chrono::microseconds>(end_m2 - start_m2);
//        timings_map["to_gray_m2"].push_back(duration_m2.count());
//
//        auto start_m3 = std::chrono::high_resolution_clock::now();
//        ImageOperator::to_gray_m3(frame, output);
//        cv::imshow("to_gray_m3", output);
//        auto end_m3 = std::chrono::high_resolution_clock::now();
//        auto duration_m3 = std::chrono::duration_cast<std::chrono::microseconds>(end_m3 - start_m3);
//        timings_map["to_gray_m3"].push_back(duration_m3.count());
//
//
//        //No OpenCV
//        const unsigned char* bgr_input = (unsigned char*)frame.data;
//        unsigned char* gray_output = new unsigned char[frame.rows * frame.cols];
//
//        auto start_no_opencv = std::chrono::high_resolution_clock::now();
//        ImageOperator::to_gray(bgr_input, frame.cols, frame.rows, frame.channels(), gray_output);
//        cv::Mat output_gray(frame.rows, frame.cols, CV_8UC1, gray_output);
//        cv::imshow("to_gray_no_opencv", output_gray);
//        auto end_no_opencv = std::chrono::high_resolution_clock::now();
//        auto duration_no_opencv = std::chrono::duration_cast<std::chrono::microseconds>(end_no_opencv - start_no_opencv);
//        timings_map["to_gray_no_opencv"].push_back(duration_no_opencv.count());
//
//        if (cv::waitKey(30) >= 0) break;
//    }
//    
//    std::unordered_map<std::string, double> avg_timings_map;
//    for (auto it : timings_map) {
//		double sum = 0;
//        for (auto& val : it.second) {
//			sum += val;
//		}
//		avg_timings_map[it.first] = sum / it.second.size();
//	}
//
//    std::cout << "AVG TIMING MAP\n";
//    for (auto it : avg_timings_map) {
//		std::cout << it.first << " : " << it.second << std::endl;
//	}
//
//
//    return 0;
//}