#include <iostream>
#include <vector>
#include <tuple>
#include <unordered_map>

#include <inttypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "threshold.hpp"

#define COMPONENT_SEPARATION_CONST 7001

#define SQ(x) (x)*(x)

using namespace cv;

Mat get_image(char *image_file, int &width, int &height) {
	Mat cv_img;
	cv_img = imread(image_file, 1);
	Mat cv_blur = cv_img.clone();
	GaussianBlur(cv_img, cv_blur, Size(3,3), 0, 0);

	width = cv_img.cols;
	height = cv_img.rows;

	return cv_blur;
}

Mat convert_to_greyscale(Mat image) {
	Mat greyscale;
	cvtColor(image, greyscale, CV_BGR2GRAY);

	// Sobel example from http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/sobel_derivatives/sobel_derivatives.html
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	Mat grad;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	// Gradient X
	Sobel( greyscale, grad_x, ddepth, 1, 0, CV_SCHARR, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );

	// Gradient Y
	Sobel( greyscale, grad_y, ddepth, 0, 1, CV_SCHARR, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y );

	// Total Gradient (approximate)
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	imwrite("sobel.png", grad);	

	// The following part is required because otherwise P[0] becomes 0
	// and we get division by zero errors when computing the between-group variance.
	uchar min_val = UCHAR_MAX;
	for (int y = 0; y < greyscale.rows; y++) {
		for (int x = 0; x < greyscale.cols; x++) {
			uchar v = greyscale.at<uchar>(y,x);
			if (v < min_val) {
					min_val = v;
			}
		}
	}

	for (int y = 0; y < greyscale.rows; y++) {
		for (int x = 0; x < greyscale.cols; x++) {
			greyscale.at<uchar>(y,x) -= min_val;
		}
	}


	Mat sub = greyscale-grad;
	imwrite("sub.png", sub);
	return sub;
}

std::vector<int> create_histogram(Mat greyscale) {
	int width = greyscale.cols;
	int height = greyscale.rows;
	std::vector<int> histogram(UCHAR_MAX, 0);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			histogram[greyscale.at<uchar>(y,x)]+=1;
		}
	}
	return histogram;
}	

std::vector<double> memoize_P(std::vector<int> histogram, int width, int height) {
	std::vector<double> P(UCHAR_MAX, 0);
	int size = width*height;
	for (int z = 0; z < UCHAR_MAX; z++) {
		P[z] = ((double)histogram[z])/size;
	}
	return P;
}

double variance(std::vector<double> P) {
	double mu = 0.0;
	for (int z = 0; z < UCHAR_MAX; z++) {
		mu += z*P[z];
	}
	double total_variance = 0.0;
	for (int z = 0; z < UCHAR_MAX; z++) {
		total_variance += (z - mu)*(z - mu) * P[z];
	}
	return total_variance;
}

int automatic_threshold(Mat greyscale) {
	int width = greyscale.cols;
	int height = greyscale.rows;
	std::vector<int> histogram = create_histogram(greyscale);

	std::vector<double> P = memoize_P(histogram, width, height);
	//double total_variance = variance(P);
	std::vector<double> between_variance(UCHAR_MAX, 0.0);

	std::vector<double> q_0(UCHAR_MAX, 0.0);
	std::vector<double> mu_0(UCHAR_MAX, 0.0);
	std::vector<double> mu_1(UCHAR_MAX, 0.0);

	double mu = 0.0;
	for (int z = 0; z < UCHAR_MAX; z++) {
		mu += z*P[z];
	}

	q_0[0] = P[0];

	mu_0[0] = 0.0;
	mu_1[0] = mu/(1 - q_0[0]); // since mu = q_1*mu_0 + q_1*mu_1, mu_0[0] = 0, and q_1 = 1-q_0

	between_variance[0] = q_0[0]*(1 - q_0[0])*(mu_0[0] - mu_1[0])*(mu_0[0] - mu_1[0]);
	for (int z = 1; z < UCHAR_MAX; z++) {
		q_0[z] = P[z] + q_0[z-1];
		mu_0[z] = (z * P[z])/q_0[z] + (mu_0[z-1]*q_0[z-1])/q_0[z];
		mu_1[z] = (mu - q_0[z]*mu_0[z])/(1 - q_0[z]);
		between_variance[z] = q_0[z]*(1 - q_0[z])*(mu_0[z] - mu_1[z])*(mu_0[z] - mu_1[z]);
	}

	double max_between_variance = between_variance[0];
	int max_threshold = 0;
	for (int z = 0; z < UCHAR_MAX; z++) {
		double b = between_variance[z];
		if (b > max_between_variance) {
			max_threshold = z;
			max_between_variance = b;
		}
	}

	return max_threshold;
}

Mat threshold_image(Mat greyscale) {
	int threshold = automatic_threshold(greyscale);
	//std::cout << "Threshold value from Otsu: " << threshold << std::endl;

	int width = greyscale.cols;
	int height = greyscale.rows;

	// because reflections and lighting and stuff
	int threshold_extra_constant = 10;

	Mat output(height, width, CV_8U);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (greyscale.at<uchar>(y,x) <= threshold+threshold_extra_constant) {
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
				output.at<uint16_t>(y,x) = UINT16_MAX;
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

Mat mask_by_component(Mat componentized, uint16_t value) {
	Mat output(componentized.size(), CV_8U);
	for (int y = 0; y < output.rows; y++) {
		for (int x = 0; x < output.cols; x++) {
			uint16_t pix = componentized.at<uint16_t>(y, x);
			if (pix == value) {
				output.at<uchar>(y, x) = UCHAR_MAX;
			}
			else {
				output.at<uchar>(y,x) = 0;
			}
		}
	}
	return output;
}

Scalar component_avg_color(Mat original, Mat component) {
	return mean(original, component);
}
/*
int main(int argc, char **argv) {
	if (argc != 2) {
		std::cout << "Usage: ./threshold [image]" << std::endl;
		return 1;
	}
	std::cout << argv[1] << std::endl;
	int width, height;
	Mat image = get_image(argv[1], width, height);
	Mat greyscale = convert_to_greyscale(image);

	Mat output = threshold_image(greyscale, width, height);
	imwrite(std::string(argv[1]) + std::string(".threshold.png"), output);

	std::vector<uint16_t> components;
	Mat component_output = connected_components(output, components);
	imwrite(std::string(argv[1]) + std::string(".connected_output.png"), component_output);

	Mat hsv_image(image.size(), image.type());
	cvtColor(image, hsv_image, CV_BGR2HSV);

	Mat orientation = image.clone();

	int i = 0;
	for (uint16_t c: components) {
		Mat masked = mask_by_component(component_output, COMPONENT_SEPARATION_CONST*c);
		Moments m = moments(masked, true);

		if (m.m00 < 4000) {
			continue;
		}
		i++;
		std::cout << "Component ID: " << i << std::endl;

		Scalar avg_color = component_avg_color(hsv_image, masked);
		std::cout << "Component HSV color: " << avg_color << std::endl;
		std::cout << "H^2 + S^2: " << SQ(avg_color[0]) + SQ(avg_color[1]) << std::endl;

		int thickness = -1;
		int lineType = 8;

		double c_x = m.m10/m.m00;
		double c_y = m.m01/m.m00;

		double axis = atan((2*m.mu11)/(m.mu20 - m.mu02))/2.0;

		// see http://www.via.cornell.edu/ece547/text/survey.pdf
		if (m.mu20 - m.mu02 <= 0) {
			axis+= M_PI/2.0;
		}

		circle( orientation,
			Point(c_x, c_y),
			10,
			Scalar(0, 0, 0),
			thickness,
			lineType );

		int axisLength = m.m00/300;

		line( orientation,
			Point(c_x, c_y),
			Point(c_x+axisLength*cos(axis), c_y+axisLength*sin(axis)),
			Scalar(0, 0, 0),
			10,
			lineType);

		int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontScale = 1;
		int textThickness = 3;
		char text[3];
		sprintf(text, "%d", i);
		putText(orientation, text, Point(c_x, c_y-20), fontFace, fontScale,
						Scalar::all(0), textThickness, 8);


		// I'm not sure if these calculations are correct
		double l_1 = m.mu20 - m.mu02 + sqrt(4*SQ(m.mu11) + SQ(m.mu20 - m.mu02));
		double l_2 = m.mu20 - m.mu02 - sqrt(4*SQ(m.mu11) + SQ(m.mu20 - m.mu02));
		double eccentricity = sqrt(1- l_2/l_1);

		std::cout << "centroid: " << c_x << ", " << c_y << std::endl;
		std::cout << "axis of orientation: " << axis << std::endl;
		std::cout << "eccentricity: " << eccentricity << std::endl;
		std::cout << std::endl;
	}
	imwrite(std::string(argv[1]) + std::string(".orientation_output.png"), orientation);
}
*/
