#ifndef _CV_IMAGE_HELPER_H
#define _CV_IMAGE_HELPER_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Image.h"

void FromCVToImage(const cv::Mat& data, Image<Eigen::Vector3f>& img);
void FromImageToCV(const Image<Eigen::Vector3f>& img, cv::Mat& data);
void FromImageToCV(const Image<Eigen::Vector2i>& img, cv::Mat& data);
void FromPosImageToColorCV(const Image<Eigen::Vector2i>& img, cv::Mat& data);

#endif