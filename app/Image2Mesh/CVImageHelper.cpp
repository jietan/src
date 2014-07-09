#include "CVImageHelper.h"

#define XY_TO_INT(x, y) (((y)<<12)|(x))

void FromCVToImage(const cv::Mat& data, Image<Eigen::Vector3f>& img)
{
	img.Create(data.rows, data.cols);
	for (int i = 0; i < data.rows; ++i)
	{
		for (int j = 0; j < data.cols; ++j)
		{
			cv::Vec3b col = data.at<cv::Vec3b>(i, j);
			img[i][j][0] = col[0];
			img[i][j][1] = col[1];
			img[i][j][2] = col[2];
		}
	}
}
void FromImageToCV(const Image<Eigen::Vector3f>& img, cv::Mat& data)
{
	int numRows = img.Height();
	int numCols = img.Width();
	data = cv::Mat(numRows, numCols, CV_8UC3);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			data.at<cv::Vec3b>(i, j) = cv::Vec3b(img[i][j][0], img[i][j][1], img[i][j][2]);
		}
	}
}


void FromImageToCV(const Image<float>& img, cv::Mat& data)
{
	int numRows = img.Height();
	int numCols = img.Width();
	data = cv::Mat(numRows, numCols, CV_8U);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			data.at<uchar>(i, j) = static_cast<uchar>(img[i][j]);
		}
	}
}

void FromImageToCV(const Image<Eigen::Vector2i>& img, cv::Mat& data)
{
	int numRows = img.Height();
	int numCols = img.Width();
	data = cv::Mat(numRows, numCols, CV_16UC3);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			data.at<cv::Vec3w>(i, j) = cv::Vec3w(img[i][j][0], img[i][j][1], 0);
		}
	}
}

void FromPosImageToColorCV(const Image<Eigen::Vector2i>& img, cv::Mat& data)
{
	int numRows = img.Height();
	int numCols = img.Width();
	data = cv::Mat(numRows, numCols, CV_8UC3);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			//int col = XY_TO_INT(img[i][j][1], img[i][j][0]);

			//int dr = (col & 255);
			//int dg = (col >> 8) & 255;
			//int db = (col >> 16);
			int x = static_cast<int>(img[i][j][1] / 3.0);
			int y = static_cast<int>(img[i][j][0] / 2.0);
			data.at<cv::Vec3b>(i, j) = cv::Vec3b(x, 0, y);
		}
	}
}