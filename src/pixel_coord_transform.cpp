#include <tf/transform_listener.h>

#include "pixel_coord_transform.hpp"

tf::Vector3 pixel_to_image_plane_transform(int x, int y, int c_x, int c_y, double f_x, double f_y, double z) {
	double _x = z*(x - c_x)/(-1)*f_x;
	double _y = z*(y - c_y)/f_y;
	double _z = z;	
	return tf::Vector3(_x, _y, _z);
}
