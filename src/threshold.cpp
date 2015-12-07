#include <iostream>
#include <vector>
#include <tuple>
#include <unordered_map>

#include <inttypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "threshold.hpp"

// N is just the maximum value a pixel can take
#define N 255

#define COMPONENT_SEPARATION_CONST 7001

#define index_image(image, x, y) (image)[(y)*(width)+(x)]

using namespace cv;

std::vector<std::tuple<int,int,int> > get_image(char *image_file, int &width, int &height) {
	std::vector<std::tuple<int,int,int> >image;
	Mat cv_img;
	cv_img = imread(image_file, 1);
	Mat cv_blur = cv_img.clone();
	GaussianBlur(cv_img, cv_blur, Size(3,3), 0, 0);

	width = cv_img.cols;
	height = cv_img.rows;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			Vec3b pix = cv_blur.at<Vec3b>(y,x);
			// pix is BRG and we want RGB so 1 2 0
			image.push_back(std::make_tuple(pix[1], pix[2], pix[0]));
		}
	}

	return image;
}



std::vector<std::tuple<int,int,int> > get_image_cvMat(Mat input_image, int &width, int &height) {
	
	std::vector<std::tuple<int,int,int> >image;
	Mat cv_img = input_image;
	//cv_img = imread(image_file, 1);

	Mat cv_blur = cv_img.clone();
	GaussianBlur(cv_img, cv_blur, Size(3,3), 0, 0);

	width = cv_img.cols;
	height = cv_img.rows;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			Vec3b pix = cv_blur.at<Vec3b>(y,x);
			// pix is BRG and we want RGB so 1 2 0
			image.push_back(std::make_tuple(pix[1], pix[2], pix[0]));
		}
	}

	return image;
}


std::vector<int> convert_to_greyscale(std::vector<std::tuple<int,int,int> > image, int width, int height) {
	// There are many ways to convert to 'greyscale', since it isn't unique. Here we use a very simple one.
	std::vector<int> greyscale(width*height, 0);

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			std::tuple<int,int,int> pix = index_image(image, x, y);
			int r = std::get<0>(pix);
			int g = std::get<1>(pix);
			int b = std::get<2>(pix);
			index_image(greyscale, x, y) = 0.2126 * r + 0.7152 * g + 0.0722 * b;
		}
	}

	// The following part is required because otherwise P[0] becomes 0
	// and we get division by zero errors when computing the between-group variance.
	int min_val = 255;
	for (int v: greyscale) {
		if (v < min_val) {
			min_val = v;
		}
	}
	for (int &v: greyscale) {
		v -= min_val;
	}

	return greyscale;
}

std::vector<int> create_histogram(std::vector<int> greyscale, int width, int height) {
	std::vector<int> histogram(N, 0);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			histogram[index_image(greyscale,x,y)]+=1;
		}
	}
	return histogram;
}	

std::vector<double> memoize_P(std::vector<int> histogram, int width, int height) {
	std::vector<double> P(N, 0);
	int size = width*height;
	for (int z = 0; z < N; z++) {
		P[z] = ((double)histogram[z])/size;
	}
	return P;
}

double variance(std::vector<double> P) {
	double mu = 0.0;
	for (int z = 0; z < N; z++) {
		mu += z*P[z];
	}
	double total_variance = 0.0;
	for (int z = 0; z < N; z++) {
		total_variance += (z - mu)*(z - mu) * P[z];
	}
	return total_variance;
}

int automatic_threshold(std::vector<int> greyscale, int width, int height) {
	std::vector<int> histogram = create_histogram(greyscale, width, height);

	std::vector<double> P = memoize_P(histogram, width, height);
	//double total_variance = variance(P);
	std::vector<double> between_variance(N, 0.0);

	std::vector<double> q_0(N, 0.0);
	std::vector<double> mu_0(N, 0.0);
	std::vector<double> mu_1(N, 0.0);

	double mu = 0.0;
	for (int z = 0; z < N; z++) {
		mu += z*P[z];
	}

	q_0[0] = P[0];

	mu_0[0] = 0.0;
	mu_1[0] = mu/(1 - q_0[0]); // since mu = q_1*mu_0 + q_1*mu_1, mu_0[0] = 0, and q_1 = 1-q_0

	between_variance[0] = q_0[0]*(1 - q_0[0])*(mu_0[0] - mu_1[0])*(mu_0[0] - mu_1[0]);
	for (int z = 1; z < N; z++) {
		q_0[z] = P[z] + q_0[z-1];
		mu_0[z] = (z * P[z])/q_0[z] + (mu_0[z-1]*q_0[z-1])/q_0[z];
		mu_1[z] = (mu - q_0[z]*mu_0[z])/(1 - q_0[z]);
		between_variance[z] = q_0[z]*(1 - q_0[z])*(mu_0[z] - mu_1[z])*(mu_0[z] - mu_1[z]);
	}

	double max_between_variance = between_variance[0];
	int max_threshold = 0;
	for (int z = 0; z < N; z++) {
		double b = between_variance[z];
		if (b > max_between_variance) {
			max_threshold = z;
			max_between_variance = b;
		}
	}

	return max_threshold;
}

Mat threshold_image(std::vector<int> greyscale, int width, int height) {
	int threshold = automatic_threshold(greyscale, width, height);
	std::cout << "Threshold value from Otsu: " << threshold << std::endl;

	// because reflections and lighting and stuff
	int threshold_extra_constant = 30;

	Mat output(height, width, CV_8U);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (index_image(greyscale, x, y) <= threshold+threshold_extra_constant) {
				output.at<uchar>(y, x) = 255;
			}
			else {
				output.at<uchar>(y, x) = 0;
			}
		}
	}
	return output;
}

Mat connected_components(Mat binarized, std::vector<uint16_t> &components) {
	Mat output = Mat(binarized.rows, binarized.cols, CV_16U, Scalar(0));
	std::unordered_map<uint16_t, uint16_t> component_ids;
	uint16_t top_id = 1;

	for (int y = 0; y < output.rows; y++) {
		for (int x = 0; x < output.cols; x++) {
			if (binarized.at<uchar>(y,x) != 0) {
				continue;
			}
			int smallest_id = top_id;
			if (x-1 >= 0) {
				uint16_t above_id = output.at<uint16_t>(y,x-1);
				if ((binarized.at<uchar>(y, x-1) == 0) && (above_id < smallest_id)) {
					smallest_id = component_ids[above_id];
				}
			}
			if (y-1 >= 0) {
				uint16_t above_id = output.at<uint16_t>(y-1,x);
				if ((binarized.at<uchar>(y-1, x) == 0) && (above_id < smallest_id)) {
					smallest_id = component_ids[above_id];
				}
			}

			output.at<uint16_t>(y,x) = smallest_id;
			if (smallest_id == top_id) {
				component_ids[smallest_id] = smallest_id;
				top_id+=1;
			}

			if (x-1 >= 0) {
				uint16_t above_id = output.at<uint16_t>(y,x-1);
				if (smallest_id < above_id) {
					component_ids[above_id] = smallest_id;
				}
			}
			if (y-1 >= 0) {
				uint16_t above_id = output.at<uint16_t>(y-1,x);
				if (smallest_id < above_id) {
					component_ids[above_id] = smallest_id;
				}
			}
		}
	}

	std::unordered_map<uint16_t, uint16_t> new_ids;
	uint16_t new_top = 1;
	for (int y = 0; y < output.rows; y++) {
		for (int x = 0; x < output.cols; x++) {
			uint16_t id = output.at<uint16_t>(y,x);
			if (id == 0) {
				output.at<uint16_t>(y,x) = 65535;
			   	continue;
			}
			uint16_t fill_id = component_ids[id];
			while (component_ids[fill_id] != fill_id) {
				fill_id = component_ids[fill_id];
			}
			if (new_ids.count(fill_id) == 0) {
				new_ids[fill_id] = new_top;
				output.at<uint16_t>(y, x) = COMPONENT_SEPARATION_CONST*new_top;
				new_top+=1;
			}
			else {
				output.at<uint16_t>(y, x) = COMPONENT_SEPARATION_CONST*new_ids[fill_id];
			}
		}
	}

	for (auto kv: new_ids) {
		components.push_back(kv.second);
	}

	return output;
}

Mat mask_by_color(Mat componentized, uint16_t value) {
	Mat output = componentized.clone();
	for (int y = 0; y < output.rows; y++) {
		for (int x = 0; x < output.cols; x++) {
			uint16_t pix = componentized.at<uint16_t>(y, x);
			if (pix == value) {
				output.at<uint16_t>(y, x) = value;
			}
			else {
				output.at<uint16_t>(y,x) = 0;
			}
		}
	}
	return output;
}


/*
int main(int argc, char **argv) {
	if (argc != 2) {
		std::cout << "Usage: ./threshold [image]" << std::endl;
		return 1;
	}
	std::cout << argv[1] << std::endl;
	int width, height;
	std::vector<std::tuple<int,int,int> > image = get_image(argv[1], width, height);
	std::vector<int> greyscale = convert_to_greyscale(image, width, height);

	Mat output = threshold_image(greyscale, width, height);
	imwrite(std::string(argv[1]) + std::string(".threshold.png"), output);
	cv::imshow("threshold", output);

	std::vector<uint16_t> components;
	Mat component_output = connected_components(output, components);
	imwrite(std::string(argv[1]) + std::string(".connected_output.png"), component_output);

	//imshow("threshold_image", output);
	cv::imshow("component_output", component_output);

	Mat orientation = imread(argv[1], 1);
	for (uint16_t c: components) {
		Moments m = moments(mask_by_color(component_output, 7001*c), true);

		if (m.m00 < 1000) {
			continue;
		}
		int thickness = -1;
		int lineType = 8;

		double c_x = m.m10/m.m00;
		double c_y = m.m01/m.m00;

		circle( orientation,
			 Point(c_x, c_y),
			 10,
			 Scalar( 0, 0, 255 ),
			 thickness,
			 lineType );

		double axis = m.mu11/(m.mu20 - m.mu02);
		std::cout << "centroid: " << c_x << ", " << c_y << std::endl;
		std::cout << "axis of orientation: " << axis << std::endl;		
	}
	imwrite(std::string(argv[1]) + std::string(".orientation_output.png"), orientation);

	//imshow("Orientation", orientation);
	//waitKey(3);
}
*/