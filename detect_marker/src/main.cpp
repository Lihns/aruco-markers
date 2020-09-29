/*
 * Copyright (c) 2019 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>


namespace {
	const char* about = "Detect ArUco marker images";
	const char* keys =
		"{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
		"DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
		"DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
		"DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
		"DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{h        |false | Print help }"
		"{v        |<none>| Custom video source, otherwise '0' }"
		;
}

int main(int argc, char** argv)
{
	int dictionaryId = 8;
	float marker_length_m = 0.2f;
	int wait_time = 10;
	cv::VideoCapture in_video(0);

	//if (!in_video.isOpened()) {
	//	std::cerr << "failed to open video input: " << videoInput << std::endl;
	//	return 1;
	//}

	cv::Ptr<cv::aruco::Dictionary> dictionary =
		cv::aruco::getPredefinedDictionary(\
			cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
	cv::Mat camera_matrix, dist_coeffs;
	fs["camera_matrix"] >> camera_matrix;
	fs["distortion_coefficients"] >> dist_coeffs;
	std::cout << "camera_matrix\n" << camera_matrix << std::endl;
	std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

	while (in_video.grab()) {
		cv::Mat image, image_copy;
		in_video.retrieve(image);
		image.copyTo(image_copy);
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids);
#ifdef _DEBUG
		ids.push_back(0);
		std::vector<cv::Point2f> points = { cv::Point2f(1.5,1),cv::Point2f(3,1),cv::Point2f(4.5,3),cv::Point2f(0,3) };
		corners.push_back(points);
#endif // _DEBUG



		// If at least one marker detected
		if (ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m, camera_matrix, dist_coeffs, rvecs, tvecs);

			auto firstMarkerRvec = rvecs[0];
			auto firstMarkerTvec = tvecs[0];

			//this matrix transforms object to camera
			cv::Mat rotation;
			cv::Rodrigues(firstMarkerRvec, rotation);
			cv::Mat objectToCam = cv::Mat::eye(4, 4, rotation.type());
			objectToCam(cv::Range(0, 3), cv::Range(0, 3)) = rotation * 1;
			objectToCam(cv::Range(0, 3), cv::Range(3, 4)) = cv::Mat(firstMarkerTvec) * 1;
			std::cout << objectToCam << std::endl;
			//cam to obj
			cv::Mat external;
			cv::invert(objectToCam, external);
			std::cout << external << std::endl;

			//assign a world position for the marker
			double x, y, z;
			std::cin >> x >> y >> z;
			cv::Mat pos = (cv::Mat_<double>(3, 1) << x, y, z);
			//from object to world
			//cv::Mat objectToWorld = cv::Mat::eye(4, 4, pos.type());
			//objectToWorld(cv::Range(0, 3), cv::Range(3, 4)) = 1 * pos;
			//std::cout << objectToWorld << std::endl;

			//from cam to world
			cv::Mat camToWorld = objectToWorld * external;
			std::cout << camToWorld << std::endl;

		}

		imshow("Detected markers", image_copy);
		char key = (char)cv::waitKey(wait_time);
		if (key == 27)
			break;
	}

	in_video.release();

	return 0;
}
