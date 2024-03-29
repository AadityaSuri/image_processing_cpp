
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

	Eigen::Tensor<unsigned char, 3> inputTensor(input.rows, input.cols, 3);
	cv::cv2eigen(input, inputTensor);

	Eigen::Tensor<unsigned char, 3> output(input.rows, input.cols, 3);
	for (int i = 0; i < input.rows; ++i) {
		for (int j = 0; j < input.cols; ++j) {
			unsigned char r = inputTensor(i, j, 2);
			unsigned char g = inputTensor(i, j, 1);
			unsigned char b = inputTensor(i, j, 0);
			output(i, j, 2) =  r;
			output(i, j, 1) = 0.5 * g;
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

//cv::Mat YUV(const cv::Mat& input) {
//
//	Eigen::Tensor<unsigned char, 3> inputTensor(input.rows, input.cols, 3);
//	cv::cv2eigen(input, inputTensor);
//
//	//Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> Y(input.rows, input.cols);
//	//Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> U(input.rows, input.cols);
//	//Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> V(input.rows, input.cols);
//
//	Eigen::Tensor<unsigned char, 3> YUVTensor(input.rows, input.cols, 3);
//
//	for (int i = 0; i < input.rows; i++) {
//		for (int j = 0; j < input.cols; j++) {
//			unsigned char r = inputTensor(i, j, 2);
//			unsigned char g = inputTensor(i, j, 1);
//			unsigned char b = inputTensor(i, j, 0);
//
//			YUVTensor(i, j, 0) = (r >> 2) + (g >> 1) + (b >> 2);
//			YUVTensor(i, j, 1) = (r >> 2) - (b >> 2);
//			YUVTensor(i, j, 2) = (b >> 2) - (r >> 2);
//
//		}
//	}
//
//	cv::Mat yuv;
//	cv::eigen2cv(YUVTensor, yuv);
//
//	return yuv;
//
//}


int main() {
	cv::VideoCapture cam(0);

	if (!cam.isOpened()) {
		throw std::runtime_error("Error");
	}

	cv::namedWindow("Window");

	while (true) {
		cv::Mat frame;
		cam >> frame;
		cv::resize(frame, frame, cv::Size(400, 300));
		cv::imshow("bgr_frame", frame);

		cv::Mat hsv_output;
		cv::cvtColor(frame, hsv_output, cv::COLOR_BGR2HSV);
		cv::imshow("hsv_image", hsv_output);

		cv::Mat yuv_output;
		cv::cvtColor(frame, yuv_output, cv::COLOR_BGR2YUV);
		cv::imshow("yuv_output", yuv_output);

		cv::Mat YCrCb_output;
		cv::cvtColor(frame, YCrCb_output, cv::COLOR_BGR2YCrCb);
		cv::imshow("YCrCb_output", YCrCb_output);


		//cv::imshow("yuv_format", YUV(frame));


		//cv::imshow("eigen_img", TO_GRAY(frame));
		//cv::imshow("eigen_red", RED(frame));
		//cv::imshow("eigen_blue", BLUE(frame));
		//cv::imshow("eigen_green", GREEN(frame));
		//cv::imshow("Y", YUV(frame)[0]);
		//cv::imshow("U", YUV(frame)[1]);
		//cv::imshow("V", YUV(frame)[2]);

		//const unsigned char* bgr_input = (unsigned char*)frame.data;
		//unsigned char* gray_output = new unsigned char[frame.rows * frame.cols];
		//
		//ImageOperator::to_gray(bgr_input, frame.cols, frame.rows, frame.channels(), gray_output);
		//cv::Mat output_gray(frame.rows, frame.cols, CV_8UC1, gray_output);
  //      cv::imshow("to_gray_no_opencv", output_gray);

		//std::unique_ptr<unsigned char[]> bgr_input = std::make_unique<unsigned char[]>(frame.total() * frame.elemSize());
		//std::copy(frame.data, frame.data + frame.total() * frame.elemSize(), bgr_input.get());

		//std::unique_ptr<unsigned char[]> gray_output;
		//ImageOperator::to_gray(bgr_input, frame.cols, frame.rows, frame.channels(), gray_output);

		//cv::Mat output_gray(frame.rows, frame.cols, CV_8UC1, gray_output.get());
		//cv::imshow("to_gray_no_opencv", output_gray);


		//std::unique_ptr<unsigned char[]> yuv_output;
		//ImageOperator::to_yuv(bgr_input, frame.cols, frame.rows, frame.channels(), yuv_output);

		//cv::Mat output_yuv(frame.rows, frame.cols, CV_8UC1, yuv_output.get());
		//cv::imshow("to_yuv_no_opencv", output_yuv);



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