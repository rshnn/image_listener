#ifndef COORD_TRANSFORM_H_
#define COORD_TRANSFORM_H_

#include <tf/transform_listener.h>

// x, y are the pixel coordinates
// c_x, c_y are the pixel coordinates of the center/principal point
// f_x f_y are related to the size of the pixels in the global frame
// z is the z value of the coordinate
tf::Vector3 pixel_to_image_plane_transform(int x, int y, int p_x, int p_y, double s_x, double s_y, double z);

#endif // COORD_TRANSFORM_H_
