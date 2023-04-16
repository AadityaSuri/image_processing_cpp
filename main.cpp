
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageOperator.h"
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


//
//Eigen::MatrixXf cvMatTo(const cv::Mat &input) {
//    Eigen::MatrixXf output(input.rows, input.cols);
//    for (int i = 0; i < input.rows; ++i) {
//        for (int j = 0; j < input.cols; ++j) {
//			output(i, j) = input.at<float>(i, j);
//		}
//	}
//	return output;
//}

int main() {
	cv::VideoCapture cam(0);

	if (!cam.isOpened()) {
		throw std::runtime_error("Error");
	}

	cv::namedWindow("Window");
	//cv::Mat output(350, 350, CV_8UC1);
	//cv::Mat rgb_output(350, 350, CV_8UC3);

	//cv::Mat frame;
	//cam >> frame;
	//cv::resize(frame, frame, cv::Size(350, 350));
	//cv::imshow("bgr_frame", frame);
	//Eigen::Tensor<float, 3> eigenmat(350, 350, 3);
	//cv::cv2eigen(frame, eigenmat);

	//std::cout << eigenmat << std::endl;

	while (true) {
		cv::Mat frame;
		cam >> frame;
		cv::resize(frame, frame, cv::Size(10, 10));
		cv::imshow("bgr_frame", frame);

		Eigen::Tensor<float, 3> eigenmat(10, 10, 3);
		cv::cv2eigen(frame, eigenmat);

		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 10; j++) {
				std::cout << eigenmat(i, j, 0) << ' ';
			}
			std::cout << std::endl;
		}

		if (cv::waitKey(30) >= 0) break;
		
		std::cout << '\n';
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