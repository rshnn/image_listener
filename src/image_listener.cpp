#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include "threshold.cpp"
#include <sensor_msgs/>
//#include <tuple>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  std::cout << "imageCallback" << std::endl;

  cv_bridge::CvImagePtr camera_image;
  try
  {
    camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int width, height;
  std::vector<std::tuple<int,int,int> > image = get_image_cvMat(camera_image->image, width, height);
  std::vector<int> greyscale = convert_to_greyscale(image, width, height);

  cv::Mat output = threshold_image(greyscale, width, height);
  
  std::vector<uint16_t> components;
  Mat component_output = connected_components(output, components);
  //imwrite(std::string(argv[1]) + std::string(".connected_output.png"), component_output);



  Mat orientation = camera_image->image; // imread(argv[1], 1);
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
    //std::cout << "centroid: " << c_x << ", " << c_y << std::endl;
    //std::cout << "axis of orientation: " << axis << std::endl; 

    cv::imshow("threshold_image", output);
    cv::imshow("component_output", component_output);
    cv::imshow("orientation", orientation);
    cv::waitKey(5);





    std::cout << "End imageCallback" << std::endl;   

  }



/* This is that other file work */
    // const int cap_red_upper = 255;
    // const int cap_green_upper = 130;
    // const int cap_blue_upper = 130;
    // const int cap_red_lower = 150;
    // const int cap_green_lower = 70;
    // const int cap_blue_lower = 70;
    // const int tube_red_upper = 255;
    // const int tube_green_upper = 255;
    // const int tube_blue_upper = 255;
    // const int tube_red_lower = 230;
    // const int tube_green_lower = 230;
    // const int tube_blue_lower = 230;

    // cv::Mat threshold_cap_mat, threshold_tube_mat, threshold_mat;
    // cv::inRange(camera_image->image,cv::Scalar(cap_blue_lower,cap_green_lower,cap_red_lower),cv::Scalar(cap_blue_upper,cap_green_upper,cap_red_upper),threshold_cap_mat);

    // cv::inRange(camera_image->image,cv::Scalar(tube_blue_lower,tube_green_lower,tube_red_lower),cv::Scalar(tube_blue_upper,tube_green_upper,tube_red_upper),threshold_tube_mat);

    // cv::imshow("Cap Image",threshold_cap_mat);
    // cv::imshow("Tube Image",threshold_tube_mat);
    // cv::waitKey(3);


    // std::vector<cv::Mat> image_channels;
    // cv::split(camera_image->image,image_channels);

    // threshold_mat = threshold_tube_mat + threshold_cap_mat;
    // image_channels[0] -= threshold_mat;
    // image_channels[1] -= threshold_mat;
    // image_channels[2] -= threshold_mat;   

    // cv::imshow("Combined", threshold_mat);

    // image_channels[1] += threshold_cap_mat*.5;
    // image_channels[1] += threshold_tube_mat;

    // cv::merge(image_channels,camera_image->image);
    
    // cv::Moments cap_moment,tube_moment;
    // cap_moment = moments(threshold_cap_mat);
    // tube_moment = moments(threshold_tube_mat);
    // double cap_x,cap_y,tube_x,tube_y,length_x,length_y,theta;
    // cap_y = cap_moment.m01/cap_moment.m00;
    // cap_x = cap_moment.m10/cap_moment.m00;
    // tube_x = tube_moment.m01/tube_moment.m00;
    // tube_y = tube_moment.m10/tube_moment.m00;
    // length_x = threshold_tube_mat.rows/2;
    // length_y = threshold_tube_mat.cols/2;
    // theta = atan2(tube_y-cap_y,tube_x-cap_x);

    // const double dpp = 0.0012;//.00147;
   
    // double error_y_ = length_y-tube_y;
    // double error_x_ = length_x-tube_x;
    // error_y_ *= dpp;
    // error_x_ *= dpp;
    

    // std::cout << error_x_ << ", " << error_y_ << ", " << tube_y-cap_y << ',' << tube_x-cap_x << ',' << theta*180/3.14159 << std::endl;

}


  
int main(int argc, char **argv)
{

  std::cout << "Image Listener initiated" << std::endl;

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/cameras/left_hand_camera/image", 1, imageCallback);
  


  ros::spin();
}