#ifndef THRESHOLD_HPP_
#define THRESHOLD_HPP_

#include <vector>
#include <tuple>

#include <opencv2/core/core.hpp>

using namespace cv;

// get image

std::vector<std::tuple<int,int,int> > get_image(int &width, int &height);


std::vector<std::tuple<int,int,int> > get_image_cvMat(Mat image, int &width, int &height);

std::vector<int> convert_to_greyscale(std::vector<std::tuple<int,int,int> > image, int width, int height);

// thresholding functions

std::vector<int> create_histogram(std::vector<int> greyscale, int width, int height);

std::vector<double> memoize_P(std::vector<int> histogram, int width, int height);

double variance(std::vector<double> P);

int automatic_threshold(std::vector<int> greyscale, int width, int height);

Mat threshold_image(std::vector<int> greyscale, int width, int height);


// connectivity functions

Mat connect_image(Mat binarized);


// moment functions


// misc functions

Mat mask_by_color(Mat componentized, uint16_t value);


#endif // THRESHOLD_HPP_
