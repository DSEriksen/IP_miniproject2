#include <iostream>
#include <opencv2\opencv.hpp>
#include <cmath>
#include <vector>

using namespace cv;
using namespace std;


int nWidth;
int nHeight;
double angle;
double PI = 3.14159;

int interpolatePos(Mat image, double _x, double _y) {

	int x = (int)round(_x);
	int y = (int)round(_y);

	if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
		return (int)image.at<uchar>(x, y);
	}
	else {
		return -1;
	}
}

Vec3d getRotatedVector(Mat _rotationMat, int x, int y) {
	//Setup vector for both the input vector and the return-vector
	Vec3d inVector = Vec3d(x, y, 1);
	Vec3d outVector;

	for(int i = 0; i < _rotationMat.rows; i++) {
		double sum = 0;
		for (int j = 0; j < _rotationMat.cols; j++) {
			sum += _rotationMat.at<double>(i, j) * inVector[j];
		}
		outVector(i) = sum;
	}

	return outVector;
}

Mat getTranslationMat(double _rX, double _rY) {
	//---Setup return-translation matrix---
	Mat translationMat = Mat(3, 3, CV_64FC1);

	//---1. row---
	translationMat.at<double>(0, 0) = 1;
	translationMat.at<double>(0, 1) = 0;
	translationMat.at<double>(0, 2) = _rX;
	//---2. row---
	translationMat.at<double>(1, 0) = 0;
	translationMat.at<double>(1, 1) = 1;
	translationMat.at<double>(1, 2) = _rY;
	//---3. row---
	translationMat.at<double>(2, 0) = 0;
	translationMat.at<double>(2, 1) = 0;
	translationMat.at<double>(2, 2) = 1;

	return translationMat;
}

Mat getRotationMat(double _angle) {
	//---Setup return-rotation matrix---
	Mat rotationMat = Mat(3, 3, CV_64FC1);
	double cosA = cos(angle);
	double sinA = sin(angle);

	//---Input values at each position, based on the input angle
	//---1. row---
	rotationMat.at<double>(0, 0) = cosA;
	rotationMat.at<double>(0, 1) = (sinA)*(-1);
	rotationMat.at<double>(0, 2) = 0;
	//---2. row---
	rotationMat.at<double>(1, 0) = sinA;
	rotationMat.at<double>(1, 1) = cosA;
	rotationMat.at<double>(1, 2) = 0;
	//---3. row---
	rotationMat.at<double>(2, 0) = 0;
	rotationMat.at<double>(2, 1) = 0;
	rotationMat.at<double>(2, 2) = 1;

	return rotationMat;
}

Mat getTransformMat(double rX, double rY, double a, double b, double _angle) {

	//---Setup a return-transform matrix---
	Mat transformMat = Mat(3, 3, CV_64FC1);
	//---tempoary matrix to hold rotationMat*translationMat
	Mat tempMat = Mat(transformMat.rows, transformMat.cols, transformMat.type());

	//---Setup and call respective matrices for later multiplication---
	Mat translationMat = getTranslationMat(rX, rY);
	Mat translationMat2 = getTranslationMat(-(rX + a), -(rY + b));
	Mat rotationMat = getRotationMat(_angle);
	
	//multiply translationMat and rotationMat
	for (int i = 0; i < translationMat.rows; i++) {
		for (int j = 0; j < rotationMat.cols; j++) {
			double sum = 0;
			for (int k = 0; k < transformMat.cols; k++) {
				sum += rotationMat.at<double>(i, k) * translationMat.at<double>(k, j);
			}
			tempMat.at<double>(i, j) = sum;
		}
	}

	//multiply tempMat(translationMat*rotationMat) and translationMat2
	for (int i = 0; i < translationMat2.rows; i++) {
		for (int j = 0; j < tempMat.cols; j++) {
			double sum = 0;
			for (int k = 0; k < transformMat.cols; k++) {
				sum += tempMat.at<double>(i, k) * translationMat2.at<double>(k, j);
			}
			transformMat.at<double>(i, j) = sum;
		}
	}


	return transformMat;
}

int main(int, char** argv) {

	//---1---Load image and display original (grayscale)---
	Mat image = imread("dog.jpg");
	cvtColor(image, image, CV_BGR2GRAY);

	cv::namedWindow("ORIGINAL");
	cv::imshow("ORIGINAL", image);
	//-------------------------------------------------


	//---2---Get angle from input---
	cout << "Input angle: ";
	cin >> angle;
	angle = angle * PI / 180;
	//--------------------------


	//---3---Setup output matrix to be displayed---
	//nWidth and nHeight makes space for rotation, so it doesnt cut off
	nWidth = (int)round((image.cols)*cos(angle) + (image.rows)*sin(angle));
	nHeight = (int)round((image.rows)*cos(angle) + (image.cols)*sin(angle));
	Mat outImage = Mat(nWidth, nHeight, image.type());
	//-----------------------------------------
	
	cout << "Strange cols: " <<(outImage.cols - image.cols) / 2 << endl;
	cout << "Strange rows: " << (outImage.rows - image.rows) / 2 << endl;


	//---4---Get a transformed rotation matrix. This both processes the angle (???and the offset for point of rotation???)-------
	Mat rotation = getTransformMat(
		1000, // de her gør ikke en skid
		1000, // de her gør ikke en skid
		0,
		outImage.cols/2,
		//(outImage.cols - image.cols) / 2.,
		//(outImage.rows - image.rows) / 2.,
		angle
	);
	//Prints out the transformed rotation matrix for good measure
	cout << rotation;
	//--------------------------------------------------------------------------------------------------------------------


	//---5---Access the elements(pixels) within outImage, and rotate position using the rotation matrix and a vector----------
	for (int y = 0; y < outImage.rows; y++) {
		for (int x = 0; x < outImage.cols; x++) {
			Vec3d vector = getRotatedVector(rotation, x, y);

			//Mapping --- interpolatePos gets the position, and returns a pixel-value (0-255)
			int pixel = interpolatePos(image, vector[0], vector[1]);
			
			if (pixel >= -1) {
				outImage.at<uchar>(x, y) = pixel;
			}
			else {
				outImage.at<uchar>(x, y) = 0;
			}

		}
	}

	cv::namedWindow("output");
	cv::imshow("output", outImage);


	cv::waitKey(0);
	return 0;
}