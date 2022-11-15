// DepthImage.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <windows.h>
#include <thread>
#include <vector>
#include <algorithm>

using namespace cv;

int numDisparities = 1;
int blockSize = 15;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;


cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, blockSize);
cv::Mat MainImage;
cv::Mat imgL;
cv::Mat imgR;
cv::Mat imgL_gray;
cv::Mat imgR_gray;
cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;


void StereoCompute(cv::Ptr<cv::StereoBM> ptr, cv::Mat LeftN, cv::Mat RightN, cv::Mat res)
{
	ptr->compute(LeftN, RightN, res);
}
static void on_trackBar1(int, void*)
{
	stereo->setNumDisparities(numDisparities * 16);
	numDisparities = numDisparities * 16;
}

static void on_trackBar2(int, void*)
{
	stereo->setBlockSize(blockSize * 2 + 5);
	blockSize = blockSize * 2 + 5;
}

static void on_trackBar3(int, void*)
{
	stereo->setPreFilterType(preFilterType);
}

static void on_trackBar4(int, void*)
{
	stereo->setPreFilterSize(preFilterSize * 2 + 5);
	preFilterSize = preFilterSize * 2 + 5;
}
static void on_trackbar5(int, void*)
{
	stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6(int, void*)
{
	stereo->setTextureThreshold(textureThreshold);
}
static void on_trackbar7(int, void*)
{
	stereo->setUniquenessRatio(uniquenessRatio);
}
static void on_trackbar8(int, void*)
{
	stereo->setSpeckleRange(speckleRange);
}
static void on_trackbar9(int, void*)
{
	stereo->setSpeckleWindowSize(speckleWindowSize * 2);
	speckleWindowSize = speckleWindowSize * 2;
}
static void on_trackbar10(int, void*)
{

	stereo->setDisp12MaxDiff(disp12MaxDiff);

}
static void on_trackbar11(int, void*)
{
	stereo->setMinDisparity(minDisparity);
}


typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} RGB_t;

typedef struct {
	uint8_t h;
	uint8_t s;
	uint8_t v;
} HSV_t;

inline RGB_t rgb(uint8_t r, uint8_t g, uint8_t b) {
	return RGB_t{ r, g, b };
}

inline HSV_t hsv(uint8_t h, uint8_t s, uint8_t v) {
	return HSV_t{ h, s, v };
}

RGB_t hsv_to_rgb(float h, float s, float v)
{
	if (s == 0.0)
	{
		return RGB_t{ (uint8_t)(v * 255), (uint8_t)(v * 255), (uint8_t)(v * 255) };
	}
	int i = int(h * 6.0);
	float f = (h * 6.0) - i;
	float p = v * (1.0 - s);
	float q = v * (1.0 - s * f);
	float t = v * (1.0 - s * (1.0 - f));
	i = i % 6;
	if (i == 0)
	{
		return RGB_t{ (uint8_t)(v * 255), (uint8_t)(t * 255), (uint8_t)(p * 255) };
	}
	if (i == 1)
	{
		return RGB_t{ (uint8_t)(q * 255), (uint8_t)(v * 255), (uint8_t)(p * 255) };
	}
	if (i == 2)
	{
		return RGB_t{ (uint8_t)(p * 255), (uint8_t)(v * 255), (uint8_t)(t * 255) };
	}
	if (i == 3)
	{
		return RGB_t{ (uint8_t)(p * 255), (uint8_t)(q * 255), (uint8_t)(v * 255) };
	}
	if (i == 4)
	{
		return RGB_t{ (uint8_t)(t * 255), (uint8_t)(v * 255), (uint8_t)(p * 255) };
	}
	if (i == 5)
	{
		return RGB_t{ (uint8_t)(v * 255), (uint8_t)(p * 255), (uint8_t)(q * 255) };
	}
}

RGB_t pseudocolor(float val, float minval, float maxval)
{
	float h = (float(val - minval) / (maxval - minval)) * 120;
	return hsv_to_rgb(h / 360, 1., 1.);
}

int main()
{

	int CamId{ 0 };
	cv::VideoCapture cam(CamId);

	cv::namedWindow("disparity", cv::WINDOW_NORMAL);
	cv::resizeWindow("disparity", 900, 900);

	cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackBar1);

	cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackBar2);

	cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackBar3);

	cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackBar4);

	cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);

	cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);

	cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);

	cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);

	cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);

	cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);

	cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

	cv::Mat disp, disparity, disp2;

	float sr = 0;
	std::vector<std::chrono::nanoseconds> times;
	std::vector<float> dist;
	while (true)
	{
		cam >> MainImage;
		int width = MainImage.cols;
		int height = MainImage.rows;

		imgL = MainImage(cv::Rect(0, 0, width / 2, height));
		imgR = MainImage(cv::Rect(width / 2, 0, width / 2, height));

		auto start_time = std::chrono::steady_clock::now();
		cv::cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
		cv::cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);

		stereo->compute(imgL_gray, imgR_gray, disp);
		disp.convertTo(disparity, CV_32F, 1.0);
		disparity = (disparity / 16.0f - 1) / (16);

		Mat img_rgb_disparity, img_hsv_disparity;
		Mat coordinates(Size(400, 400), CV_8UC3);
		Mat pseudocvt(Size(disparity.cols, disparity.rows), CV_8UC3);

		int Xc = disparity.cols / 2;
		int Yc = disparity.rows / 2;
		Mat Xcenter = disparity(Rect(Xc - 30, Yc - 30, 60, 60));
		std::vector<float> d;
		for (int i = 0; i < Xcenter.cols; i++)
		{
			
				float intensity = Xcenter.at<float>(20, i);
				//intensity += 0.125;
				d.push_back(intensity);
			
		}
		auto it = std::max_element(d.begin(),d.end());
		d.clear();
		float r = 1 + 1-*(it._Ptr)*1;
		dist.push_back(r);
		float sumR=0;
		if (dist.size() == 100)
		{			
			sr = 0;
			for (int i = 0; i < dist.size(); i++)
			{
				sumR += dist[i];
			}
			sr = sumR / 100;
			dist.clear();
		}

		for (int i = 0; i < disparity.cols; i++)
		{
			for (int j = 0; j < disparity.rows; j++)
			{
				float intensity = disparity.at<float>(j, i);
				intensity += 1;
				unsigned char intens = (unsigned char)(intensity * 100);
				RGB_t rgb = pseudocolor(intensity, 0, 1);
				pseudocvt.at<Vec3b>(j, i)[0] = rgb.r;
				pseudocvt.at<Vec3b>(j, i)[1] = rgb.g;
				pseudocvt.at<Vec3b>(j, i)[2] = rgb.b;

				//coordinates.at<Vec3b>(j, i)[0] = 255;
				//coordinates.at<Vec3b>(j, i)[0] = 255;
				//coordinates.at<Vec3b>(j, i)[0] = 255;
			}
		}
		std::string result = "distance: " + std::to_string(sr);

		int left = 0, right = 0;
		for (int i = 0; i < Xcenter.cols; i++)
		{
			if (Xcenter.at<float>(20, i) > *(it._Ptr)-0.05)
			{
				left = i;
				break;
			}

		}
		for (int i = Xcenter.cols-1; i >= 0; i--)
		{
			if (Xcenter.at<float>(20, i) > * (it._Ptr)-0.05)
			{
				right = i;
				break;
			}

		}
		int xCoord = 0;
		int xRealCoord = 0;
		std::string coordX;
		if (left != 0 && right != 0)
		{
			//xCoord = (left + right) / 2;

			xRealCoord = Xc - 30 + xCoord;
			coordX = "X: " + std::to_string(right);
		}
		else
		{
			coordX = "X: undefined";
		}
		putText(coordinates, coordX, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255));
		putText(coordinates, result, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255));

		auto end_time = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
		times.push_back(elapsed);

		cv::imshow("cam", MainImage);
		cv::imshow("camL", imgL);
		cv::imshow("camR", imgR);
		cv::imshow("disp", disparity);
		cv::imshow("pseudo", pseudocvt);
		imshow("coord", coordinates);
		imshow("center", Xcenter);


		if (cv::waitKey(1) == 27) break;
	}
	long time = 0;
	for (int i = 0; i < 100; i++)
	{
		time += times[i].count();
	}
	long res = time / 100;
	std::cout << "Time:" << res << std::endl;
	cam.release();
	return 0;
}

