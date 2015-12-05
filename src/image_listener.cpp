#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>


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

    const int cap_red_upper = 255;
    const int cap_green_upper = 130;
    const int cap_blue_upper = 130;
    const int cap_red_lower = 150;
    const int cap_green_lower = 70;
    const int cap_blue_lower = 70;
    const int tube_red_upper = 255;
    const int tube_green_upper = 255;
    const int tube_blue_upper = 255;
    const int tube_red_lower = 230;
    const int tube_green_lower = 230;
    const int tube_blue_lower = 230;

    // cv::Mat threshold_cap_mat, threshold_tube_mat, threshold_mat;
    // cv::inRange(camera_image->image,cv::Scalar(cap_blue_lower,cap_green_lower,cap_red_lower),cv::Scalar(cap_blue_upper,cap_green_upper,cap_red_upper),threshold_cap_mat);

    // cv::inRange(camera_image->image,cv::Scalar(tube_blue_lower,tube_green_lower,tube_red_lower),cv::Scalar(tube_blue_upper,tube_green_upper,tube_red_upper),threshold_tube_mat);

    // cv::imshow("Cap Image",threshold_cap_mat);
    // cv::imshow("Tube Image",threshold_tube_mat);
    // cv::waitKey(3);



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