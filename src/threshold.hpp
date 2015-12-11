#ifndef THRESHOLD_HPP_
#define THRESHOLD_HPP_

#include <vector>
#include <tuple>

#include <opencv2/core/core.hpp>

using namespace cv;

#define COMPONENT_SEPARATION_CONST 7001

#define SQ(x) (x)*(x)

// get image

Mat get_image(char *image_file, int &width, int &height);

Mat convert_to_greyscale(Mat image);

// thresholding functions

std::vector<int> create_histogram(Mat greyscale);

std::vector<double> memoize_P(std::vector<int> histogram, int width, int height);

double variance(std::vector<double> P);

int automatic_threshold(Mat greyscale);

Mat threshold_image(Mat greyscale);


// connectivity functions

Mat connected_components(Mat binarized, std::vector<uint16_t> &components);


// moment functions


// misc functions

Mat mask_by_component(Mat componentized, uint16_t value);

Scalar component_avg_color(Mat original, Mat component);

#endif // THRESHOLD_HPP_
